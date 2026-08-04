/*
 * Copyright (c) 2025 @amgskobo
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_iqs7211e

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zmk/keymap.h>
#include <zephyr/pm/device.h>
#include "iqs7211e_init.h"
#include "iqs7211e.h"

LOG_MODULE_REGISTER(iqs7211e, CONFIG_ZMK_LOG_LEVEL);

/* Spacing between the press and release edges of a generated click. */
#define IQS7211E_CLICK_EDGE_MS 20

static enum iqs7211e_gestures_event iqs7211e_get_touchpad_event(const struct iqs7211e_data *data);
static bool iqs7211e_init_state(struct iqs7211e_data *data);
static int iqs7211e_get_product_num(struct iqs7211e_data *data);
static int iqs7211e_read_info_flags(const struct iqs7211e_data *data, uint8_t *info_flags);
static int iqs7211e_check_reset(struct iqs7211e_data *data);
static int iqs7211e_acknowledge_reset(struct iqs7211e_data *data);
static bool iqs7211e_read_ati_active(struct iqs7211e_data *data);
static int iqs7211e_read_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *buf, size_t len);
static int iqs7211e_write_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, const uint8_t *data, size_t numBytes);
static void iqs7211e_work_handler(struct k_work *work);
static void iqs7211e_stationary_report_work_handler(struct k_work *work);
static void iqs7211e_click_work_handler(struct k_work *work);
static void iqs7211e_queue_clicks(struct iqs7211e_data *data, uint16_t button, uint8_t clicks);
static int iqs7211e_report_data(struct iqs7211e_data *data);
static void iqs7211e_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins);
static int iqs7211e_write_defaults(struct iqs7211e_data *data);
static int iqs7211e_sw_reset(struct iqs7211e_data *data);
#ifdef CONFIG_PM_DEVICE
static int iqs7211e_set_suspend_state(struct iqs7211e_data *data, bool suspend);
#endif
static int iqs7211e_run_ati(struct iqs7211e_data *data);
static int iqs7211e_queue_value_updates(struct iqs7211e_data *data);
static int iqs7211e_set_event_mode(struct iqs7211e_data *data);
static bool iqs7211e_layer_allowed(const uint8_t *layers, uint8_t layer_count);
static bool iqs7211e_scroll_trigger_layer_allowed(const struct iqs7211e_config *config);
static bool iqs7211e_stationary_report_layer_allowed(const struct iqs7211e_config *config);
static bool iqs7211e_stationary_report_allowed(const struct iqs7211e_data *data);
static void iqs7211e_stationary_report_release_touch(struct iqs7211e_data *data);
static uint8_t iqs7211e_get_bit(uint8_t byte, uint8_t pos);
static uint8_t iqs7211e_get_num_fingers_from_info_flags(const uint8_t *info_flags);
static uint8_t iqs7211e_get_num_fingers(const struct iqs7211e_data *data);
static int set_gpio_interrupt(const struct device *dev, const bool en);
static int iqs7211e_init(const struct device *dev);

static bool iqs7211e_init_state(struct iqs7211e_data *data)
{
    int ret = 0;
    switch (data->init_state)
    {
    case IQS7211E_INIT_VERIFY_PRODUCT:
    {
        int prod_num = iqs7211e_get_product_num(data);
        if (prod_num < 0)
        {
            /* A transient bus error is retried on the next RDY interrupt. */
            LOG_ERR("product number read failed: %d - retrying", prod_num);
        }
        else if (prod_num == IQS7211E_PRODUCT_NUM)
        {
            data->init_state = IQS7211E_INIT_READ_RESET;
        }
        else
        {
            /* A successful read of another product is not transient. */
            LOG_ERR("product number read %d, expected %d - disabling device",
                    prod_num, IQS7211E_PRODUCT_NUM);
            data->init_state = IQS7211E_INIT_NONE;
        }
        break;
    }

    case IQS7211E_INIT_READ_RESET:
        ret = iqs7211e_check_reset(data);
        if (ret < 0)
        {
            /* Retry the read instead of mistaking an I2C error for no reset. */
            break;
        }
        if (ret > 0)
        {
            data->init_state = IQS7211E_INIT_UPDATE_SETTINGS;
            data->reset_called = false;
        }
        else if (!data->reset_called)
        {
            data->init_state = IQS7211E_INIT_CHIP_RESET;
        }
        break;

    case IQS7211E_INIT_CHIP_RESET:
        if (!data->reset_called)
        {
            ret = iqs7211e_sw_reset(data);
            if (ret < 0)
            {
                break;
            }
            data->reset_called = true;
            data->init_state = IQS7211E_INIT_READ_RESET;
        }
        break;

    case IQS7211E_INIT_UPDATE_SETTINGS:
        ret = iqs7211e_write_defaults(data);
        if (ret < 0)
            break;
        data->init_state = IQS7211E_INIT_ACK_RESET;
        break;

    case IQS7211E_INIT_ACK_RESET:
        ret = iqs7211e_acknowledge_reset(data);
        if (ret < 0)
            break;
        data->init_state = IQS7211E_INIT_ATI;
        break;

    case IQS7211E_INIT_ATI:
        ret = iqs7211e_run_ati(data);
        if (ret < 0)
            break;
        data->init_state = IQS7211E_INIT_WAIT_FOR_ATI;
        break;

    case IQS7211E_INIT_WAIT_FOR_ATI:
        if (!iqs7211e_read_ati_active(data))
        {
            data->init_state = IQS7211E_INIT_READ_DATA;
        }
        break;

    case IQS7211E_INIT_READ_DATA:
        ret = iqs7211e_queue_value_updates(data);
        if (ret < 0)
            break;
        data->init_state = IQS7211E_INIT_ACTIVATE_EVENT_MODE;
        break;

    case IQS7211E_INIT_ACTIVATE_EVENT_MODE:
        ret = iqs7211e_set_event_mode(data);
        if (ret < 0)
            break;
        data->init_state = IQS7211E_INIT_DONE;
        LOG_DBG("IQS7211E initialization successful");
        break;
    case IQS7211E_INIT_DONE:
        return true;

    default:
        break;
    }
    // LOG_DBG("Transition to state: %d", data->init_state);
    return false;
}

/* Returns the product number, or a negative errno if the read failed. The
   return type must stay signed: truncating an errno into uint16_t made a bus
   error indistinguishable from a wrong part. */
static int iqs7211e_get_product_num(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t buf[2];
    int ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_PROD_NUM, buf, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to read product number");
        return ret;
    }
    return ((uint16_t)buf[1] << 8) | buf[0];
}

static int iqs7211e_read_info_flags(const struct iqs7211e_data *data, uint8_t *info_flags)
{
    const struct iqs7211e_config *config = data->dev->config;
    int ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_INFO_FLAGS, info_flags, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to read INFO_FLAGS register");
        return ret;
    }
    return 0;
}

static int iqs7211e_check_reset(struct iqs7211e_data *data)
{
    uint8_t info_flags[2];
    int ret;
    ret = iqs7211e_read_info_flags(data, info_flags);
    if (ret < 0)
    {
        return ret;
    }
    LOG_DBG("Info Flags: %02X %02X", info_flags[0], info_flags[1]);
    return (info_flags[0] & (1 << IQS7211E_SHOW_RESET_BIT)) != 0;
}

static int iqs7211e_sw_reset(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;
    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL, command, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to read system control register for reset");
        return ret;
    }

    command[1] |= (1 << IQS7211E_SW_RESET_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL, command, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to write system control register for reset");
        return ret;
    }
    LOG_DBG("IQS7211E software reset issued");
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int iqs7211e_set_suspend_state(struct iqs7211e_data *data, bool suspend)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2] = {SYSTEM_CONTROL_0, SYSTEM_CONTROL_1};

    if (suspend)
    {
        command[1] |= BIT(IQS7211E_SUSPEND_BIT);
    }
    else
    {
        command[1] &= ~BIT(IQS7211E_SUSPEND_BIT);
    }

    /*
     * While suspended, writing the cleared command forces an I2C communication
     * window and wakes the device as specified by the IQS7211E datasheet.
     * System Control has no persistent non-zero fields in this configuration,
     * so starting from the exported defaults also avoids replaying stale
     * one-shot reset/re-ATI commands read from the device.
     */
    int ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL,
                                   command, sizeof(command));
    if (ret < 0)
    {
        LOG_ERR("Failed to %s IQS7211E: %d", suspend ? "suspend" : "resume", ret);
        return ret;
    }

    LOG_DBG("IQS7211E hardware %s", suspend ? "suspended" : "resumed");
    return 0;
}
#endif

static int iqs7211e_acknowledge_reset(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;

    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL, command, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to read system control register during ACK reset");
        return ret;
    }

    command[0] |= (1 << IQS7211E_ACK_RESET_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL, command, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to write ACK reset to system control register");
        return ret;
    }
    LOG_DBG("IQS7211E reset acknowledged (ACK_RESET_BIT set)");
    return 0;
}

static int iqs7211e_run_ati(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;
    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL, command, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to read command reg for ATI");
        return ret;
    }

    command[0] |= (1 << IQS7211E_TP_RE_ATI_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_MM_SYS_CONTROL, command, 2);
    if (ret < 0)
    {
        LOG_ERR("Failed to write REATI command");
        return ret;
    }

    LOG_DBG("IQS7211E ATI triggered");
    return 0;
}

static bool iqs7211e_read_ati_active(struct iqs7211e_data *data)
{
    uint8_t info_flags[2];
    int ret = iqs7211e_read_info_flags(data, info_flags);
    if (ret < 0)
    {
        LOG_WRN("Failed to read info flags, assuming ATI still running");
        return true;
    }
    return !(info_flags[0] & (1 << IQS7211E_RE_ATI_OCCURRED_BIT));
}

static int iqs7211e_queue_value_updates(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    /*
     * 12 bytes covers 0x0E..0x13 in one transaction:
     *   0x0E gestures | 0x0F info flags | 0x10 X | 0x11 Y
     *   0x12 touch strength | 0x13 area   <- diagnostics, no extra I2C cost
     */
    uint8_t buf[12];
    int ret;
    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_GESTURES, buf, 12);
    if (ret < 0)
    {
        LOG_ERR("Failed to read GESTURES and FINGER_1 data");
        return ret;
    }

    data->gestures[0] = buf[0];
    data->gestures[1] = buf[1];

    data->info_flags[0] = buf[2];
    data->info_flags[1] = buf[3];

    data->finger_1_x = (buf[5] << 8) | buf[4];
    data->finger_1_y = (buf[7] << 8) | buf[6];

    data->finger_1_strength = (buf[9] << 8) | buf[8];
    data->finger_1_area = (buf[11] << 8) | buf[10];

    /*
     * A re-ATI re-seeds the channel references, so one firing while a finger is
     * near the pad would capture the baseline WITH the finger present and skew
     * every touch threshold afterwards. Tracked so it can be logged when tuning;
     * measurement showed zero re-ATI events in normal use, so it is not the
     * cause of hover-induced pointer drift.
     */
    data->prev_re_ati_occurred =
        (data->info_flags[0] & (1 << IQS7211E_RE_ATI_OCCURRED_BIT)) != 0;

    /*
     * Finger 2 is physically not supported on this 22x22mm module.
     * Disabling to optimize I2C traffic.
     *
    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_FINGER_2_X, buf, 4);
    if (ret < 0)
    {
        LOG_ERR("Failed to read FINGER_2 data");
        return ret;
    }
    data->finger_2_x = (buf[1] << 8) | buf[0];
    data->finger_2_y = (buf[3] << 8) | buf[2];
    */

    return 0;
}

static int iqs7211e_set_event_mode(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;
    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_CONFIG_SETTINGS, command, sizeof(command));
    if (ret < 0)
    {
        LOG_ERR("Failed to read system control register");
        return ret;
    }

    command[1] |= (1 << IQS7211E_EVENT_MODE_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_MM_CONFIG_SETTINGS, command, sizeof(command));
    if (ret < 0)
    {
        LOG_ERR("Failed to write system control register");
        return ret;
    }

    LOG_DBG("Event mode enabled");
    return 0;
}

static enum iqs7211e_gestures_event iqs7211e_get_touchpad_event(const struct iqs7211e_data *data)
{
    /*
     * Priority Check: Triple > Double > Single.
     * Higher-order gestures take precedence in case multiple flags are set
     * within the same I2C data packet, ensuring the most complete gesture is reported.
     */
    if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_TRIPLE_TAP_BIT))
    {
        return IQS7211E_GESTURE_TRIPLE_TAP;
    }
    else if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_DOUBLE_TAP_BIT))
    {
        return IQS7211E_GESTURE_DOUBLE_TAP;
    }
    else if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_SINGLE_TAP_BIT))
    {
        return IQS7211E_GESTURE_SINGLE_TAP;
    }
    else if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_PRESS_HOLD_BIT))
    {
        return IQS7211E_GESTURE_PRESS_HOLD;
    }
    else if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_PALM_GESTURE_BIT))
    {
        return IQS7211E_GESTURE_PALM_GESTURE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_X_POSITIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_X_POSITIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_X_NEGATIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_X_NEGATIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_Y_POSITIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_Y_POSITIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_Y_NEGATIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_Y_NEGATIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE;
    }
    else if (iqs7211e_get_bit(data->gestures[1], IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE_BIT))
    {
        return IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE;
    }
    else
    {
        return IQS7211E_GESTURE_NONE;
    }
}

static uint8_t iqs7211e_get_bit(uint8_t byte, uint8_t pos)
{
    return (byte >> pos) & 0x01;
}

static bool iqs7211e_layer_allowed(const uint8_t *layers, uint8_t layer_count)
{
    if (layer_count == 0)
    {
        return true;
    }

    uint8_t active_layer = zmk_keymap_highest_layer_active();

    for (uint8_t i = 0; i < layer_count; i++)
    {
        if (layers[i] == active_layer)
        {
            return true;
        }
    }

    return false;
}

static bool iqs7211e_scroll_trigger_layer_allowed(const struct iqs7211e_config *config)
{
    return iqs7211e_layer_allowed(config->scroll_trigger_layers, config->scroll_trigger_layer_count);
}

static bool iqs7211e_stationary_report_layer_allowed(const struct iqs7211e_config *config)
{
    return iqs7211e_layer_allowed(config->stationary_report_layers, config->stationary_report_layer_count);
}

static bool iqs7211e_stationary_report_allowed(const struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;

    return config->report_abs && config->stationary_report_interval_ms > 0 &&
           data->last_touched_state && iqs7211e_stationary_report_layer_allowed(config);
}

static uint8_t iqs7211e_get_num_fingers_from_info_flags(const uint8_t *info_flags)
{
    uint8_t byte = info_flags[1];
    uint8_t num = iqs7211e_get_bit(byte, IQS7211E_NUM_FINGERS_BIT_0) |
                  (iqs7211e_get_bit(byte, IQS7211E_NUM_FINGERS_BIT_1) << 1);
    return num;
}

static uint8_t iqs7211e_get_num_fingers(const struct iqs7211e_data *data)
{
    return iqs7211e_get_num_fingers_from_info_flags(data->info_flags);
}

static void iqs7211e_stationary_report_release_touch(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;

    if (data->last_touched_state)
    {
        input_report_key(data->dev, INPUT_BTN_TOUCH, false, false, K_FOREVER);
        data->last_touched_state = false;
    }

    if (data->start_tap == 1 && config->press_hold >= 0)
    {
        input_report_key(data->dev, INPUT_BTN_0 + config->press_hold, false, false, K_FOREVER);
    }
    data->start_tap = 0;

    if (config->report_abs)
    {
        input_report_abs(data->dev, INPUT_ABS_X, data->finger_1_prev_x, false, K_FOREVER);
        input_report_abs(data->dev, INPUT_ABS_Y, data->finger_1_prev_y, true, K_FOREVER);
    }

    if (data->is_scroll_layer_active && config->scroll_layer >= 0)
    {
        zmk_keymap_layer_deactivate(config->scroll_layer, false);
        data->is_scroll_layer_active = false;
        LOG_DBG("Scroll layer deactivated");
    }

    data->touch_count = 0;
    data->finger_1_prev_dx = 0;
    data->finger_1_prev_dy = 0;
}

static int iqs7211e_write_defaults(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    const struct i2c_dt_spec *i2c = &config->i2c;
    int ret;
    uint8_t buf[30];
    /* 1. ALP Compensation (0x1F - 0x20) */
    buf[0] = ALP_COMPENSATION_A_0;
    buf[1] = ALP_COMPENSATION_A_1;
    buf[2] = ALP_COMPENSATION_B_0;
    buf[3] = ALP_COMPENSATION_B_1;
    ret = iqs7211e_write_bytes(i2c, 0x1F, buf, 4);
    if (ret < 0)
        return ret;
    LOG_DBG("1. ALP Compensation written");

    /* 2. ATI Settings (0x21 - 0x27) */
    buf[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
    buf[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
    buf[2] = TP_COMPENSATION_DIV;
    buf[3] = TP_REF_DRIFT_LIMIT;
    buf[4] = TP_ATI_TARGET_0;
    buf[5] = TP_ATI_TARGET_1;
    buf[6] = TP_MIN_COUNT_REATI_0;
    buf[7] = TP_MIN_COUNT_REATI_1;
    buf[8] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
    buf[9] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
    buf[10] = ALP_COMPENSATION_DIV;
    buf[11] = ALP_LTA_DRIFT_LIMIT;
    buf[12] = ALP_ATI_TARGET_0;
    buf[13] = ALP_ATI_TARGET_1;
    ret = iqs7211e_write_bytes(i2c, 0x21, buf, 14);
    if (ret < 0)
        return ret;
    LOG_DBG("2. ATI Settings written");

    /* 3. Report Rates and Timings (0x28 - 0x32) */
    buf[0] = ACTIVE_MODE_REPORT_RATE_0;
    buf[1] = ACTIVE_MODE_REPORT_RATE_1;
    buf[2] = IDLE_TOUCH_MODE_REPORT_RATE_0;
    buf[3] = IDLE_TOUCH_MODE_REPORT_RATE_1;
    buf[4] = IDLE_MODE_REPORT_RATE_0;
    buf[5] = IDLE_MODE_REPORT_RATE_1;
    buf[6] = LP1_MODE_REPORT_RATE_0;
    buf[7] = LP1_MODE_REPORT_RATE_1;
    buf[8] = LP2_MODE_REPORT_RATE_0;
    buf[9] = LP2_MODE_REPORT_RATE_1;
    buf[10] = ACTIVE_MODE_TIMEOUT_0;
    buf[11] = ACTIVE_MODE_TIMEOUT_1;
    buf[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
    buf[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
    buf[14] = IDLE_MODE_TIMEOUT_0;
    buf[15] = IDLE_MODE_TIMEOUT_1;
    buf[16] = LP1_MODE_TIMEOUT_0;
    buf[17] = LP1_MODE_TIMEOUT_1;
    buf[18] = REATI_RETRY_TIME;
    buf[19] = REF_UPDATE_TIME;
    buf[20] = I2C_TIMEOUT_0;
    buf[21] = I2C_TIMEOUT_1;
    ret = iqs7211e_write_bytes(i2c, 0x28, buf, 22);
    if (ret < 0)
        return ret;
    LOG_DBG("3. Report rates and timings written");

    /* 4. System Control Settings (0x33 - 0x35) */
    buf[0] = SYSTEM_CONTROL_0;
    buf[1] = SYSTEM_CONTROL_1;
    buf[2] = CONFIG_SETTINGS0;
    buf[3] = CONFIG_SETTINGS1;
    buf[4] = OTHER_SETTINGS_0;
    buf[5] = OTHER_SETTINGS_1;
    ret = iqs7211e_write_bytes(i2c, 0x33, buf, 6);
    if (ret < 0)
        return ret;
    LOG_DBG("4. System control settings written");

    /* 5. ALP Setup (0x36 - 0x37) */
    buf[0] = ALP_SETUP_0;
    buf[1] = ALP_SETUP_1;
    buf[2] = ALP_TX_ENABLE_0;
    buf[3] = ALP_TX_ENABLE_1;
    ret = iqs7211e_write_bytes(i2c, 0x36, buf, 4);
    if (ret < 0)
        return ret;
    LOG_DBG("5. ALP Setup written");

    /* 6. Threshold Settings (0x38 - 0x3A) */
    buf[0] = TRACKPAD_TOUCH_SET_THRESHOLD;
    buf[1] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
    buf[2] = ALP_THRESHOLD_0;
    buf[3] = ALP_THRESHOLD_1;
    buf[4] = ALP_SET_DEBOUNCE;
    buf[5] = ALP_CLEAR_DEBOUNCE;
    ret = iqs7211e_write_bytes(i2c, 0x38, buf, 6);
    if (ret < 0)
        return ret;
    LOG_DBG("6. Threshold settings written");

    /* 7. Filter Betas (0x3B - 0x3C) */
    buf[0] = ALP_COUNT_BETA_LP1;
    buf[1] = ALP_LTA_BETA_LP1;
    buf[2] = ALP_COUNT_BETA_LP2;
    buf[3] = ALP_LTA_BETA_LP2;
    ret = iqs7211e_write_bytes(i2c, 0x3B, buf, 4);
    if (ret < 0)
        return ret;
    LOG_DBG("7. Filter Betas written");

    /* 8. Hardware Settings (0x3D - 0x40) */
    buf[0] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
    buf[1] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
    buf[2] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
    buf[3] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
    buf[4] = TRACKPAD_HARDWARE_SETTINGS_0;
    buf[5] = TRACKPAD_HARDWARE_SETTINGS_1;
    buf[6] = ALP_HARDWARE_SETTINGS_0;
    buf[7] = ALP_HARDWARE_SETTINGS_1;
    ret = iqs7211e_write_bytes(i2c, 0x3D, buf, 8);
    if (ret < 0)
        return ret;
    LOG_DBG("8. Hardware settings written");

    /* 9. TP Setup (0x41 - 0x49) */
    buf[0] = TRACKPAD_SETTINGS_0_0;
    buf[1] = TRACKPAD_SETTINGS_0_1;
    buf[2] = TRACKPAD_SETTINGS_1_0;
    buf[3] = TRACKPAD_SETTINGS_1_1;
    buf[4] = X_RESOLUTION_0;
    buf[5] = X_RESOLUTION_1;
    buf[6] = Y_RESOLUTION_0;
    buf[7] = Y_RESOLUTION_1;
    buf[8] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
    buf[9] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
    buf[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
    buf[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
    buf[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
    buf[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
    buf[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
    buf[15] = FINGER_SPLIT_FACTOR;
    buf[16] = X_TRIM_VALUE;
    buf[17] = Y_TRIM_VALUE;
    ret = iqs7211e_write_bytes(i2c, 0x41, buf, 18);
    if (ret < 0)
        return ret;
    LOG_DBG("9. TP Settings written");

    /* 10. Version Numbers (0x4A - 0x4A) */
    buf[0] = MINOR_VERSION;
    buf[1] = MAJOR_VERSION;
    ret = iqs7211e_write_bytes(i2c, 0x4A, buf, 2);
    if (ret < 0)
        return ret;
    LOG_DBG("10. Version numbers written");

    /* 11. Gesture Settings (0x4B - 0x55) */
    buf[0] = GESTURE_ENABLE_0;
    buf[1] = GESTURE_ENABLE_1;
    buf[2] = TAP_TOUCH_TIME_0;
    buf[3] = TAP_TOUCH_TIME_1;
    buf[4] = TAP_WAIT_TIME_0;
    buf[5] = TAP_WAIT_TIME_1;
    buf[6] = TAP_DISTANCE_0;
    buf[7] = TAP_DISTANCE_1;
    buf[8] = HOLD_TIME_0;
    buf[9] = HOLD_TIME_1;
    buf[10] = SWIPE_TIME_0;
    buf[11] = SWIPE_TIME_1;
    buf[12] = SWIPE_X_DISTANCE_0;
    buf[13] = SWIPE_X_DISTANCE_1;
    buf[14] = SWIPE_Y_DISTANCE_0;
    buf[15] = SWIPE_Y_DISTANCE_1;
    buf[16] = SWIPE_X_CONS_DIST_0;
    buf[17] = SWIPE_X_CONS_DIST_1;
    buf[18] = SWIPE_Y_CONS_DIST_0;
    buf[19] = SWIPE_Y_CONS_DIST_1;
    buf[20] = SWIPE_ANGLE;
    buf[21] = PALM_THRESHOLD;
    ret = iqs7211e_write_bytes(i2c, 0x4B, buf, 22);
    if (ret < 0)
        return ret;
    LOG_DBG("11. Gesture settings written");

    /* 12. RxTx Mapping (0x56 - 0x5C) */
    buf[0] = RX_TX_MAP_0;
    buf[1] = RX_TX_MAP_1;
    buf[2] = RX_TX_MAP_2;
    buf[3] = RX_TX_MAP_3;
    buf[4] = RX_TX_MAP_4;
    buf[5] = RX_TX_MAP_5;
    buf[6] = RX_TX_MAP_6;
    buf[7] = RX_TX_MAP_7;
    buf[8] = RX_TX_MAP_8;
    buf[9] = RX_TX_MAP_9;
    buf[10] = RX_TX_MAP_10;
    buf[11] = RX_TX_MAP_11;
    buf[12] = RX_TX_MAP_12;
    buf[13] = RX_TX_MAP_FILLER;
    ret = iqs7211e_write_bytes(i2c, 0x56, buf, 14);
    if (ret < 0)
        return ret;
    LOG_DBG("12. RxTx mapping written");

    /* 13. Allocation of channels into cycles 0-9 (0x5D - 0x6B) */
    buf[0] = PLACEHOLDER_0;
    buf[1] = CH_1_CYCLE_0;
    buf[2] = CH_2_CYCLE_0;
    buf[3] = PLACEHOLDER_1;
    buf[4] = CH_1_CYCLE_1;
    buf[5] = CH_2_CYCLE_1;
    buf[6] = PLACEHOLDER_2;
    buf[7] = CH_1_CYCLE_2;
    buf[8] = CH_2_CYCLE_2;
    buf[9] = PLACEHOLDER_3;
    buf[10] = CH_1_CYCLE_3;
    buf[11] = CH_2_CYCLE_3;
    buf[12] = PLACEHOLDER_4;
    buf[13] = CH_1_CYCLE_4;
    buf[14] = CH_2_CYCLE_4;
    buf[15] = PLACEHOLDER_5;
    buf[16] = CH_1_CYCLE_5;
    buf[17] = CH_2_CYCLE_5;
    buf[18] = PLACEHOLDER_6;
    buf[19] = CH_1_CYCLE_6;
    buf[20] = CH_2_CYCLE_6;
    buf[21] = PLACEHOLDER_7;
    buf[22] = CH_1_CYCLE_7;
    buf[23] = CH_2_CYCLE_7;
    buf[24] = PLACEHOLDER_8;
    buf[25] = CH_1_CYCLE_8;
    buf[26] = CH_2_CYCLE_8;
    buf[27] = PLACEHOLDER_9;
    buf[28] = CH_1_CYCLE_9;
    buf[29] = CH_2_CYCLE_9;
    ret = iqs7211e_write_bytes(i2c, 0x5D, buf, 30);
    if (ret < 0)
        return ret;
    LOG_DBG("13. Cycle 0-9 allocation written");

    /* 14. Allocation of channels into cycles 10-19 (0x6C - 0x7A) */
    buf[0] = PLACEHOLDER_10;
    buf[1] = CH_1_CYCLE_10;
    buf[2] = CH_2_CYCLE_10;
    buf[3] = PLACEHOLDER_11;
    buf[4] = CH_1_CYCLE_11;
    buf[5] = CH_2_CYCLE_11;
    buf[6] = PLACEHOLDER_12;
    buf[7] = CH_1_CYCLE_12;
    buf[8] = CH_2_CYCLE_12;
    buf[9] = PLACEHOLDER_13;
    buf[10] = CH_1_CYCLE_13;
    buf[11] = CH_2_CYCLE_13;
    buf[12] = PLACEHOLDER_14;
    buf[13] = CH_1_CYCLE_14;
    buf[14] = CH_2_CYCLE_14;
    buf[15] = PLACEHOLDER_15;
    buf[16] = CH_1_CYCLE_15;
    buf[17] = CH_2_CYCLE_15;
    buf[18] = PLACEHOLDER_16;
    buf[19] = CH_1_CYCLE_16;
    buf[20] = CH_2_CYCLE_16;
    buf[21] = PLACEHOLDER_17;
    buf[22] = CH_1_CYCLE_17;
    buf[23] = CH_2_CYCLE_17;
    buf[24] = PLACEHOLDER_18;
    buf[25] = CH_1_CYCLE_18;
    buf[26] = CH_2_CYCLE_18;
    buf[27] = PLACEHOLDER_19;
    buf[28] = CH_1_CYCLE_19;
    buf[29] = CH_2_CYCLE_19;
    ret = iqs7211e_write_bytes(i2c, 0x6C, buf, 30);
    if (ret < 0)
        return ret;
    LOG_DBG("14. Cycle 10-19 allocation written");

    /* 15. Allocation of channels into cycle 20 (0x7B - 0x7C) */
    /* Memory Map Position 0x7B - 0x7C */
    buf[0] = PLACEHOLDER_20;
    buf[1] = CH_1_CYCLE_20;
    buf[2] = CH_2_CYCLE_20;
    ret = iqs7211e_write_bytes(i2c, 0x7B, buf, 3);
    if (ret < 0)
        return ret;
    LOG_DBG("15. Write Cycle 20  Settings");
    return 0;
}

static int iqs7211e_read_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *data, size_t len)
{
    int ret = i2c_burst_read_dt(i2c, reg, data, len);
    if (ret < 0)
    {
        LOG_ERR("i2c_read failed at reg 0x%02X (%zu bytes): %d", reg, len, ret);
    }
    return ret;
}

static int iqs7211e_write_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, const uint8_t *data, size_t len)
{
    int ret = i2c_burst_write_dt(i2c, reg, data, len);
    if (ret < 0)
    {
        LOG_ERR("i2c_write failed at reg 0x%02X (%zu bytes): %d", reg, len, ret);
    }
    return ret;
}

static void iqs7211e_work_handler(struct k_work *work)
{
    struct iqs7211e_data *data = CONTAINER_OF(work, struct iqs7211e_data, work);
    if (atomic_get(&data->suspended))
    {
        return;
    }

    if (iqs7211e_init_state(data))
    {
        int ret = iqs7211e_report_data(data);
        if (ret < 0 && data->stationary_verify_pending)
        {
            LOG_WRN("Stationary touch verify failed; releasing touch");
            iqs7211e_stationary_report_release_touch(data);
        }
    }
    data->stationary_verify_pending = false;

    if (!atomic_get(&data->suspended) && set_gpio_interrupt(data->dev, true) < 0)
    {
        LOG_ERR("Failed to re-enable IQS7211E interrupt");
    }
}

/*
 * Emit one press or release edge, then reschedule until the requested number of
 * clicks has been played out. This runs on the system workqueue like the rest
 * of the driver, but yields between edges instead of sleeping, so a triple tap
 * no longer holds the queue for 120ms.
 */
static void iqs7211e_click_work_handler(struct k_work *work)
{
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(d_work, struct iqs7211e_data, click_work);

    if (atomic_get(&data->suspended) || data->click_edges == 0)
    {
        return;
    }

    /* An even number of edges left means the next one opens a click. */
    bool press = (data->click_edges % 2) == 0;
    input_report_key(data->dev, data->click_button, press, true, K_FOREVER);
    data->click_edges--;

    if (data->click_edges > 0)
    {
        k_work_reschedule(&data->click_work, K_MSEC(IQS7211E_CLICK_EDGE_MS));
    }
}

/*
 * Queue `clicks` press/release pairs on `button`. If a tap arrives while a
 * previous sequence is still playing, the old one is released first so the
 * button cannot be left stuck down.
 */
static void iqs7211e_queue_clicks(struct iqs7211e_data *data, uint16_t button, uint8_t clicks)
{
    if (atomic_get(&data->suspended))
    {
        return;
    }

    if (data->click_edges > 0)
    {
        /* Mid-sequence: an odd count means a press is currently outstanding. */
        if ((data->click_edges % 2) == 1)
        {
            input_report_key(data->dev, data->click_button, false, true, K_FOREVER);
        }
        data->click_edges = 0;
    }

    data->click_button = button;
    data->click_edges = clicks * 2;
    k_work_reschedule(&data->click_work, K_NO_WAIT);
}

static void iqs7211e_stationary_report_work_handler(struct k_work *work)
{
    struct k_work_delayable *d_work = k_work_delayable_from_work(work);
    struct iqs7211e_data *data = CONTAINER_OF(d_work, struct iqs7211e_data, stationary_report_work);
    const struct iqs7211e_config *config = data->dev->config;

    if (atomic_get(&data->suspended) || !iqs7211e_stationary_report_allowed(data))
    {
        return;
    }

    uint32_t now = k_uptime_get_32();
    if (config->stationary_touch_verify_interval_ms > 0 &&
        (uint32_t)(now - data->stationary_last_verify_uptime_ms) >=
            config->stationary_touch_verify_interval_ms)
    {
        /*
         * Verification must consume the complete report packet. A partial
         * INFO_FLAGS read followed by STOP can close an RDY window that was
         * opened for a gesture or coordinate event, losing that event before
         * the normal report work gets to it. Queue the regular report path
         * with the IRQ masked so it is the sole owner of this communication
         * window.
         */
        data->stationary_verify_pending = true;
        if (set_gpio_interrupt(data->dev, false) < 0 || k_work_submit(&data->work) < 0)
        {
            data->stationary_verify_pending = false;
            iqs7211e_stationary_report_release_touch(data);
            if (set_gpio_interrupt(data->dev, true) < 0)
            {
                LOG_ERR("Failed to restore IQS7211E interrupt after verify failure");
            }
        }
        return;
    }

    input_report_abs(data->dev, INPUT_ABS_X, data->finger_1_prev_x, false, K_FOREVER);
    input_report_abs(data->dev, INPUT_ABS_Y, data->finger_1_prev_y, true, K_FOREVER);

    if (iqs7211e_stationary_report_allowed(data))
    {
        k_work_reschedule(&data->stationary_report_work, K_MSEC(config->stationary_report_interval_ms));
    }
}

static int iqs7211e_report_data(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    int ret = iqs7211e_queue_value_updates(data);
    if (ret < 0)
    {
        return ret;
    }
    uint8_t num_fingers = iqs7211e_get_num_fingers(data);
    uint8_t gesture_event = iqs7211e_get_touchpad_event(data);

    /* 1. Canonicalize coordinates (Normalized to user orientation) */
    int16_t x;
    int16_t y;

    if (num_fingers > 0)
    {
        int16_t raw_x = data->finger_1_x;
        int16_t raw_y = data->finger_1_y;

        x = raw_x;
        y = raw_y;

        if (config->rotate_cw == 1)
        {
            /*
             * Rotation 90deg CW:
             * X = (MaxY) - raw_y
             * Y = raw_x
             */
            x = RESOLUTION_Y - raw_y;
            y = raw_x;
        }
        else if (config->rotate_cw == 2)
        {
            /*
             * Rotation 180deg CW:
             * X = (MaxX) - raw_x
             * Y = (MaxY) - raw_y
             */
            x = RESOLUTION_X - raw_x;
            y = RESOLUTION_Y - raw_y;
        }
        else if (config->rotate_cw == 3)
        {
            /*
             * Rotation 270deg CW:
             * X = raw_y
             * Y = (MaxX) - raw_x
             */
            x = raw_y;
            y = RESOLUTION_X - raw_x;
        }
    }
    else
    {
        /*
         * No fingers: reuse the last reported position so the pointer does not
         * jump to (0,0). finger_1_prev_* is stored after normalization, so it
         * must not be rotated a second time - doing so mirrors the release
         * coordinate whenever rotate-cw is non-zero, and disagrees with the
         * stationary-report paths, which use finger_1_prev_* directly.
         */
        x = data->finger_1_prev_x;
        y = data->finger_1_prev_y;
    }

    LOG_DBG("Fingers: %d, Gesture: %d, Mode: %s", num_fingers, gesture_event, config->report_abs ? "Abs" : "Rel");
    /* Finger 2 reporting is disabled for this hardware profile */
    LOG_DBG("Raw: F1(X=%d, Y=%d) | Norm: X=%d, Y=%d",
            data->finger_1_x, data->finger_1_y, x, y);

    // Skip first frame setup for smoothing
    uint8_t skip_count = 1;

    /* 2. Movement Calculation (Only for relative mode) */
    int16_t dx = 0, dy = 0, smooth_dx = 0, smooth_dy = 0;
    if (!config->report_abs)
    {
        if (num_fingers > 0)
        {
            dx = (data->touch_count == 0) ? 0 : (x - data->finger_1_prev_x);
            dy = (data->touch_count == 0) ? 0 : (y - data->finger_1_prev_y);
        }
        else
        {
            /* On release, maintain last velocity for inertia initialization */
            dx = data->finger_1_prev_dx;
            dy = data->finger_1_prev_dy;
        }

        /*
         * Use division, not an arithmetic shift. `>> 1` rounds towards
         * negative infinity, so an odd sum loses half a count in the positive
         * direction but gains half a count in the negative one. That leaves a
         * constant negative offset of about 0.25 counts per report on every
         * axis that is moving, which is large enough to cancel out - or even
         * invert - slow movement. Division truncates towards zero, so both
         * directions are attenuated equally.
         */
        smooth_dx = (dx + data->finger_1_prev_dx) / 2;
        smooth_dy = (dy + data->finger_1_prev_dy) / 2;
    }

    /* 3. Input Reporting and Synchronization */
    if (num_fingers > 0)
    {
        /* --- Path: Touch Active --- */

        /* 3.1. Touch State Toggle */
        if (!data->last_touched_state)
        {
            input_report_key(data->dev, INPUT_BTN_TOUCH, true, false, K_FOREVER);
            data->last_touched_state = true;
        }

        /* 3.2. Scroll Layer Detection */
        if (data->touch_count <= 2 && config->scroll_layer >= 0 && !data->is_scroll_layer_active &&
            iqs7211e_scroll_trigger_layer_allowed(config))
        {
            /* Compare against MaxX - padding */
            if (x > RESOLUTION_X - config->scroll_start)
            {
                zmk_keymap_layer_activate(config->scroll_layer, false);
                data->is_scroll_layer_active = true;
                LOG_DBG("Scroll layer activated");
            }
        }

        /* 3.3. Gesture / Button Processing (Skip if scrolling) */
        if (!data->is_scroll_layer_active)
        {
            switch (gesture_event)
            {
            case IQS7211E_GESTURE_PRESS_HOLD:
                if (config->press_hold >= 0 && data->start_tap == 0)
                {
                    input_report_key(data->dev, INPUT_BTN_0 + config->press_hold, true, true, K_FOREVER);
                    data->start_tap = 1;
                }
                break;
            case IQS7211E_GESTURE_SINGLE_TAP:
                if (config->single_tap >= 0)
                {
                    iqs7211e_queue_clicks(data, INPUT_BTN_0 + config->single_tap, 1);
                }
                break;
            case IQS7211E_GESTURE_DOUBLE_TAP:
                if (config->double_tap >= 0)
                {
                    iqs7211e_queue_clicks(data, INPUT_BTN_0 + config->double_tap, 2);
                }
                break;
            case IQS7211E_GESTURE_TRIPLE_TAP:
                if (config->triple_tap >= 0)
                {
                    iqs7211e_queue_clicks(data, INPUT_BTN_0 + config->triple_tap, 3);
                }
                break;
            default:
                break;
            }
        }

        /* 3.4. Coordinate Reporting */
        if (config->report_abs)
        {
            input_report_abs(data->dev, INPUT_ABS_X, x, false, K_FOREVER);
            input_report_abs(data->dev, INPUT_ABS_Y, y, true, K_FOREVER);
        }
        else if (data->touch_count >= skip_count)
        {
            input_report_rel(data->dev, INPUT_REL_X, smooth_dx, false, K_FOREVER);
            input_report_rel(data->dev, INPUT_REL_Y, smooth_dy, true, K_FOREVER);
        }
        else
        {
            /* First frame: sync ON state with zero movement */
            input_report_rel(data->dev, INPUT_REL_X, 0, true, K_FOREVER);
        }

        /* 3.5. Update History */
        if (data->touch_count < 255)
        {
            data->touch_count++;
        }
    }
    else
    {
        /* --- Path: Touch Released --- */

        /* 4.1. Touch State Toggle */
        if (data->last_touched_state)
        {
            input_report_key(data->dev, INPUT_BTN_TOUCH, false, false, K_FOREVER);
            data->last_touched_state = false;
            k_work_cancel_delayable(&data->stationary_report_work);
        }

        /* 4.2. Process Release Gestures (Taps, etc.) - Only if not scrolling */
        if (!data->is_scroll_layer_active)
        {
            switch (gesture_event)
            {
            case IQS7211E_GESTURE_SINGLE_TAP:
                if (config->single_tap >= 0)
                {
                    iqs7211e_queue_clicks(data, INPUT_BTN_0 + config->single_tap, 1);
                }
                break;
            case IQS7211E_GESTURE_DOUBLE_TAP:
                if (config->double_tap >= 0)
                {
                    iqs7211e_queue_clicks(data, INPUT_BTN_0 + config->double_tap, 2);
                }
                break;
            case IQS7211E_GESTURE_TRIPLE_TAP:
                if (config->triple_tap >= 0)
                {
                    iqs7211e_queue_clicks(data, INPUT_BTN_0 + config->triple_tap, 3);
                }
                break;
            default:
                break;
            }
        }

        /* 4.3. Release press-hold if active (Always do this on lift-off) */
        if (data->start_tap == 1)
        {
            input_report_key(data->dev, INPUT_BTN_0 + config->press_hold, false, true, K_FOREVER);
            data->start_tap = 0;
        }

        /* 4.4. Reporting and Sync */
        if (config->report_abs)
        {
            input_report_abs(data->dev, INPUT_ABS_X, x, false, K_FOREVER);
            input_report_abs(data->dev, INPUT_ABS_Y, y, true, K_FOREVER);
        }
        else
        {
            /* Sync OFF state with final movement deltas for inertia */
            input_report_rel(data->dev, INPUT_REL_X, smooth_dx, false, K_FOREVER);
            input_report_rel(data->dev, INPUT_REL_Y, smooth_dy, true, K_FOREVER);
        }

        /* 4.4. Scroll Layer Cleanup (At the very end of switching) */
        if (data->is_scroll_layer_active)
        {
            zmk_keymap_layer_deactivate(config->scroll_layer, false);
            data->is_scroll_layer_active = false;
            LOG_DBG("Scroll layer deactivated");
        }

        /* 4.5. Update History */
        data->touch_count = 0;
        data->finger_1_prev_dx = 0;
        data->finger_1_prev_dy = 0;
    }

    /* 5. Common History Update */
    data->finger_1_prev_x = x;
    data->finger_1_prev_y = y;
    if (!config->report_abs)
    {
        data->finger_1_prev_dx = dx;
        data->finger_1_prev_dy = dy;
    }

    if (config->stationary_report_interval_ms > 0)
    {
        if (num_fingers > 0 && iqs7211e_stationary_report_allowed(data))
        {
            data->stationary_last_verify_uptime_ms = k_uptime_get_32();
            k_work_reschedule(&data->stationary_report_work, K_MSEC(config->stationary_report_interval_ms));
        }
        else
        {
            k_work_cancel_delayable(&data->stationary_report_work);
        }
    }

    return 0;
}

static int set_gpio_interrupt(const struct device *dev, const bool en)
{
    const struct iqs7211e_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_EDGE_FALLING : GPIO_INT_DISABLE);
    if (ret < 0)
    {
        LOG_ERR("Failed to set interrupt");
        return ret;
    }
    return 0;
}

static void iqs7211e_gpio_callback(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    struct iqs7211e_data *data = CONTAINER_OF(cb, struct iqs7211e_data, gpio_cb);
    if (atomic_get(&data->suspended))
    {
        return;
    }

    if (set_gpio_interrupt(data->dev, false) < 0 || k_work_submit(&data->work) < 0)
    {
        LOG_ERR("Failed to queue IQS7211E interrupt work");
        if (!atomic_get(&data->suspended))
        {
            set_gpio_interrupt(data->dev, true);
        }
    }
}

static int iqs7211e_init(const struct device *dev)
{
    const struct iqs7211e_config *config = dev->config;
    struct iqs7211e_data *data = dev->data;
    int ret;

    if (!device_is_ready(config->i2c.bus))
    {
        LOG_ERR("I2C bus not ready: %s", config->i2c.bus->name);
        return -ENODEV;
    }

    if (!device_is_ready(config->irq_gpio.port))
    {
        LOG_ERR("IRQ GPIO port not ready: %s", config->irq_gpio.port->name);
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (ret < 0)
    {
        LOG_ERR("Failed to configure IRQ pin: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->gpio_cb, iqs7211e_gpio_callback, BIT(config->irq_gpio.pin));
    ret = gpio_add_callback(config->irq_gpio.port, &data->gpio_cb);
    if (ret < 0)
    {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    data->init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    data->touch_count = 0;
    data->start_tap = 0;
    data->is_scroll_layer_active = false;
    data->last_touched_state = false;
    data->stationary_verify_pending = false;
    data->stationary_last_verify_uptime_ms = 0;
    data->dev = dev;
    atomic_clear(&data->suspended);
    data->sensor_suspended = false;

    k_work_init(&data->work, iqs7211e_work_handler);
    k_work_init_delayable(&data->stationary_report_work, iqs7211e_stationary_report_work_handler);
    k_work_init_delayable(&data->click_work, iqs7211e_click_work_handler);
    data->click_edges = 0;
    ret = set_gpio_interrupt(data->dev, true);
    if (ret < 0)
    {
        return ret;
    }

    LOG_INF("IQS7211E driver initialized successfully");
    return 0;
}

#ifdef CONFIG_PM_DEVICE
static int iqs7211e_pm_action(const struct device *dev, enum pm_device_action action)
{
    struct iqs7211e_data *data = dev->data;
    const struct iqs7211e_config *config = dev->config;
    int ret;

    switch (action)
    {
    case PM_DEVICE_ACTION_SUSPEND:
        /* Block every producer before draining work and mutable report state. */
        atomic_set(&data->suspended, 1);
        ret = set_gpio_interrupt(dev, false);
        if (ret < 0)
        {
            atomic_clear(&data->suspended);
            return ret;
        }

        k_work_cancel_sync(&data->work, &data->work_sync);
        /* A running handler may have re-enabled the IRQ before it completed. */
        ret = set_gpio_interrupt(dev, false);
        if (ret < 0)
        {
            atomic_clear(&data->suspended);
            return ret;
        }

        k_work_cancel_delayable_sync(&data->click_work, &data->click_work_sync);
        if ((data->click_edges % 2) == 1)
        {
            /* A press was outstanding - do not suspend with the button held. */
            input_report_key(dev, data->click_button, false, true, K_FOREVER);
        }
        data->click_edges = 0;
        k_work_cancel_delayable_sync(&data->stationary_report_work, &data->stationary_report_work_sync);

        bool touch_was_active = data->last_touched_state;
        iqs7211e_stationary_report_release_touch(data);
        if (touch_was_active && !config->report_abs)
        {
            /* Flush the queued BTN_TOUCH release in relative mode. */
            input_report_rel(dev, INPUT_REL_X, 0, true, K_FOREVER);
        }

        data->stationary_verify_pending = false;
        data->stationary_last_verify_uptime_ms = 0;

        if (data->init_state != IQS7211E_INIT_NONE)
        {
            ret = iqs7211e_set_suspend_state(data, true);
            if (ret < 0)
            {
                atomic_clear(&data->suspended);
                set_gpio_interrupt(dev, true);
                return ret;
            }
            data->sensor_suspended = true;
        }

        return 0;
    case PM_DEVICE_ACTION_RESUME:
        if (data->sensor_suspended)
        {
            ret = iqs7211e_set_suspend_state(data, false);
            if (ret < 0)
            {
                return ret;
            }
            data->sensor_suspended = false;
        }

        data->init_state = IQS7211E_INIT_VERIFY_PRODUCT;
        data->reset_called = false;
        data->touch_count = 0;
        data->start_tap = 0;
        data->is_scroll_layer_active = false;
        data->last_touched_state = false;
        data->stationary_verify_pending = false;
        data->stationary_last_verify_uptime_ms = 0;
        data->click_edges = 0;
        atomic_clear(&data->suspended);
        LOG_DBG("IQS7211E device resumed ");
        return set_gpio_interrupt(dev, true);
    default:
        return -ENOTSUP;
    }
}
#endif // #ifdef CONFIG_PM_DEVICE

#define IQS7211E_SCROLL_TRIGGER_LAYERS(inst)                                                   \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, scroll_trigger_layers),                             \
                (static const uint8_t iqs7211e_scroll_trigger_layers_##inst[] =                  \
                     DT_INST_PROP(inst, scroll_trigger_layers);),                                 \
                ())

#define IQS7211E_STATIONARY_REPORT_LAYERS(inst)                                                 \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, stationary_report_layers),                           \
                (static const uint8_t iqs7211e_stationary_report_layers_##inst[] =                \
                     DT_INST_PROP(inst, stationary_report_layers);),                               \
                ())

#define IQS7211E_VALIDATE(inst)                                                                 \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, single_tap, -1) >= -1 &&                                 \
                     DT_INST_PROP_OR(inst, single_tap, -1) <= INT8_MAX,                          \
                 "single-tap must fit in int8_t and be at least -1");                           \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, double_tap, -1) >= -1 &&                                 \
                     DT_INST_PROP_OR(inst, double_tap, -1) <= INT8_MAX,                          \
                 "double-tap must fit in int8_t and be at least -1");                           \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, triple_tap, -1) >= -1 &&                                 \
                     DT_INST_PROP_OR(inst, triple_tap, -1) <= INT8_MAX,                          \
                 "triple-tap must fit in int8_t and be at least -1");                           \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, scroll_layer, -1) >= -1 &&                               \
                     DT_INST_PROP_OR(inst, scroll_layer, -1) <= INT8_MAX,                        \
                 "scroll-layer must fit in int8_t and be at least -1");                         \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, scroll_start, 40) >= 0 &&                                \
                     DT_INST_PROP_OR(inst, scroll_start, 40) <= RESOLUTION_X,                    \
                 "scroll-start must be within the coordinate range");                          \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, stationary_report_interval_ms, 0) >= 0 &&                 \
                     DT_INST_PROP_OR(inst, stationary_report_interval_ms, 0) <= UINT16_MAX,      \
                 "stationary-report-interval-ms must fit in uint16_t");                         \
    BUILD_ASSERT(DT_INST_PROP_OR(inst, stationary_touch_verify_interval_ms, 120) >= 0 &&         \
                     DT_INST_PROP_OR(inst, stationary_touch_verify_interval_ms, 120) <=          \
                         UINT16_MAX,                                                             \
                 "stationary-touch-verify-interval-ms must fit in uint16_t")

#define IQS7211E_DEFINE(inst)                                                                   \
    IQS7211E_VALIDATE(inst);                                                                    \
    IQS7211E_SCROLL_TRIGGER_LAYERS(inst)                                                        \
    IQS7211E_STATIONARY_REPORT_LAYERS(inst)                                                     \
    static struct iqs7211e_data iqs7211e_data_##inst;                                           \
    static const struct iqs7211e_config iqs7211e_config_##inst = {                              \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                      \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),                                     \
        .single_tap = DT_INST_PROP_OR(inst, single_tap, -1),                                    \
        .double_tap = DT_INST_PROP_OR(inst, double_tap, -1),                                    \
        .triple_tap = DT_INST_PROP_OR(inst, triple_tap, -1),                                    \
        .press_hold = -1,                                                                       \
        .scroll_layer = DT_INST_PROP_OR(inst, scroll_layer, -1),                                \
        .scroll_start = DT_INST_PROP_OR(inst, scroll_start, 40),                                \
        .scroll_trigger_layers = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, scroll_trigger_layers), \
                                             (iqs7211e_scroll_trigger_layers_##inst), (NULL)),   \
        .scroll_trigger_layer_count = DT_INST_PROP_LEN_OR(inst, scroll_trigger_layers, 0),       \
        .stationary_report_layers = COND_CODE_1(                                                \
            DT_INST_NODE_HAS_PROP(inst, stationary_report_layers),                               \
            (iqs7211e_stationary_report_layers_##inst), (NULL)),                                 \
        .stationary_report_layer_count = DT_INST_PROP_LEN_OR(inst, stationary_report_layers, 0), \
        .rotate_cw = DT_INST_PROP_OR(inst, rotate_cw, 0),                                       \
        .report_abs = DT_INST_PROP(inst, report_abs),                                           \
        .stationary_report_interval_ms = DT_INST_PROP_OR(inst, stationary_report_interval_ms, 0),\
        .stationary_touch_verify_interval_ms =                                                   \
            DT_INST_PROP_OR(inst, stationary_touch_verify_interval_ms, 120),                     \
    };                                                                                          \
    PM_DEVICE_DT_INST_DEFINE(inst, iqs7211e_pm_action);            \
    DEVICE_DT_INST_DEFINE(inst,                                    \
                          &iqs7211e_init,                          \
                          PM_DEVICE_DT_INST_GET(inst),             \
                          &iqs7211e_data_##inst,                   \
                          &iqs7211e_config_##inst,                 \
                          POST_KERNEL,                             \
                          CONFIG_INPUT_INIT_PRIORITY,              \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS7211E_DEFINE)
