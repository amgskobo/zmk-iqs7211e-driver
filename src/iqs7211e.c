// iqs7211e.c
#define DT_DRV_COMPAT azoteq_iqs7211e

#include <zephyr/device.h>
#include <zephyr/input/input.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include "iqs7211e_init.h"
#include "iqs7211e.h"

LOG_MODULE_REGISTER(iqs7211e, CONFIG_INPUT_LOG_LEVEL);

static bool iqs7211e_init_state(struct iqs7211e_data *data);
static uint16_t iqs7211e_get_product_num(struct iqs7211e_data *data);
static int iqs7211e_read_info_flags(const struct iqs7211e_data *data, uint8_t *info_flags);
static bool iqs7211e_check_reset(struct iqs7211e_data *data);
static int iqs7211e_acknowledge_reset(struct iqs7211e_data *data);
static bool iqs7211e_read_ati_active(struct iqs7211e_data *data);
static int iqs7211e_read_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *buf, size_t len);
static int iqs7211e_write_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, const uint8_t *data, size_t numBytes);
static void iqs7211e_work_handler(struct k_work *work);
static void iqs7211e_report_data(struct iqs7211e_data *data);
static void set_interrupt(const struct device *dev, const bool en);
static void iqs7211e_gpio_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins);
static int iqs7211e_write_defaults(struct iqs7211e_data *data);
static int iqs7211e_sw_reset(struct iqs7211e_data *data);
static int iqs7211e_run_ati(struct iqs7211e_data *data);
static void iqs7211e_queue_value_updates(struct iqs7211e_data *data);
static int iqs7211e_set_event_mode(struct iqs7211e_data *data);
static enum iqs7211e_power_mode iqs7211e_get_power_mode(const struct iqs7211e_data *data);
static enum iqs7211e_gestures_event iqs7211e_get_touchpad_event(const struct iqs7211e_data *data);
static uint8_t iqs7211e_get_bit(uint8_t byte, uint8_t pos);
static uint8_t iqs7211e_get_num_fingers(const struct iqs7211e_data *data);
static int16_t calc_delta(uint16_t current, uint16_t prev);
int iqs7211e_init(const struct device *dev);

static bool iqs7211e_init_state(struct iqs7211e_data *data)
{
    switch (data->init_state)
    {
    case IQS7211E_INIT_VERIFY_PRODUCT:
        uint16_t prod_num = iqs7211e_get_product_num(data);
        if (prod_num == IQS7211E_PRODUCT_NUM)
        {
            data->init_state = IQS7211E_INIT_READ_RESET;
        }
        else
        {
            LOG_INF("IQS7211E IQS7211E_INIT_NONE");
            data->init_state = IQS7211E_INIT_NONE;
        }
        break;

    case IQS7211E_INIT_READ_RESET:

        if (iqs7211e_check_reset(data))
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
            int ret = iqs7211e_sw_reset(data);
            if (ret != 0)
            {
                break;
            }
            data->reset_called = true;
            data->init_state = IQS7211E_INIT_READ_RESET;
        }
        break;

    case IQS7211E_INIT_UPDATE_SETTINGS:

        iqs7211e_write_defaults(data);
        data->init_state = IQS7211E_INIT_ACK_RESET;
        break;

    case IQS7211E_INIT_ACK_RESET:
        iqs7211e_acknowledge_reset(data);
        data->init_state = IQS7211E_INIT_ATI;
        break;

    case IQS7211E_INIT_ATI:
        iqs7211e_run_ati(data);
        data->init_state = IQS7211E_INIT_WAIT_FOR_ATI;
        break;

    case IQS7211E_INIT_WAIT_FOR_ATI:
        if (!iqs7211e_read_ati_active(data))
        {
            data->init_state = IQS7211E_INIT_READ_DATA;
        }
        break;

    case IQS7211E_INIT_READ_DATA:
        iqs7211e_queue_value_updates(data);
        data->init_state = IQS7211E_INIT_ACTIVATE_EVENT_MODE;
        break;

    case IQS7211E_INIT_ACTIVATE_EVENT_MODE:
        iqs7211e_set_event_mode(data);
        data->init_state = IQS7211E_INIT_DONE;
        break;
    case IQS7211E_INIT_DONE:
        LOG_INF("IQS7211E initialization DONE");
        return true;

    default:
        break;
    }
    LOG_DBG("Transition to state: %d", data->init_state);
    return false;
}

static uint16_t iqs7211e_get_product_num(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t buf[2];
    int ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_PROD_NUM, buf, 2);
    if (ret != 0)
    {
        LOG_ERR("Failed to read product number");
        return 0;
    }
    return ((uint16_t)buf[1] << 8) | buf[0];
}

static int iqs7211e_read_info_flags(const struct iqs7211e_data *data, uint8_t *info_flags)
{
    const struct iqs7211e_config *config = data->dev->config;
    int ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_INFO_FLAGS_REG, info_flags, 2);
    if (ret)
    {
        LOG_ERR("Failed to read INFO_FLAGS register");
        return ret;
    }
    return 0;
}

static bool iqs7211e_check_reset(struct iqs7211e_data *data)
{
    uint8_t info_flags[2];
    int ret;

    ret = iqs7211e_read_info_flags(data, info_flags);
    if (ret)
    {
        return false;
    }
    LOG_DBG("Info Flags: %02X %02X", info_flags[0], info_flags[1]);
    return (info_flags[0] & (1 << IQS7211E_SHOW_RESET_BIT)) != 0;
}

static int iqs7211e_sw_reset(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;
    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_SYSTEM_CONTROL_REG, command, 2);
    if (ret)
    {
        LOG_ERR("Failed to read system control register for reset");
        return ret;
    }

    command[1] |= (1 << IQS7211E_SW_RESET_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_SYSTEM_CONTROL_REG, command, 2);
    if (ret)
    {
        LOG_ERR("Failed to write system control register for reset");
        return ret;
    }
    LOG_INF("IQS7211E software reset issued");
    return 0;
}

static int iqs7211e_acknowledge_reset(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;

    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_SYSTEM_CONTROL_REG, command, 2);
    if (ret)
    {
        LOG_ERR("Failed to read system control register during ACK reset");
        return ret;
    }

    command[0] |= (1 << IQS7211E_ACK_RESET_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_SYSTEM_CONTROL_REG, command, 2);
    if (ret)
    {
        LOG_ERR("Failed to write ACK reset to system control register");
        return ret;
    }
    LOG_INF("IQS7211E reset acknowledged (ACK_RESET_BIT set)");
    return 0;
}

int iqs7211e_run_ati(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;

    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_SYSTEM_CONTROL_REG, command, 2);
    if (ret)
    {
        LOG_ERR("Failed to read command reg for ATI");
        return ret;
    }

    command[0] |= (1 << IQS7211E_TP_RE_ATI_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_SYSTEM_CONTROL_REG, command, 2);
    if (ret)
    {
        LOG_ERR("Failed to write REATI command");
        return ret;
    }

    LOG_INF("IQS7211E ATI triggered");
    return 0;
}

static bool iqs7211e_read_ati_active(struct iqs7211e_data *data)
{
    uint8_t info_flags[2];
    int ret = iqs7211e_read_info_flags(data, info_flags);

    if (ret != 0)
    {
        LOG_WRN("Failed to read info flags, assuming ATI still running");
        return true;
    }

    return !(info_flags[0] & (1 << IQS7211E_RE_ATI_OCCURRED_BIT));
}

static void iqs7211e_queue_value_updates(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t buf[8];
    int ret;

    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_GESTURES, buf, 8);
    if (ret != 0)
    {
        LOG_ERR("Failed to read GESTURES and FINGER_1 data");
        return;
    }

    data->gestures[0] = buf[0];
    data->gestures[1] = buf[1];

    data->info_flags[0] = buf[2];
    data->info_flags[1] = buf[3];

    data->finger_1_x = (buf[5] << 8) | buf[4];
    data->finger_1_y = (buf[7] << 8) | buf[6];

    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_FINGER_2_X, buf, 4);
    if (ret != 0)
    {
        LOG_ERR("Failed to read FINGER_2 data");
        return;
    }

    data->finger_2_x = (buf[1] << 8) | buf[0];
    data->finger_2_y = (buf[3] << 8) | buf[2];

    LOG_DBG("Gestures: %02X %02X, InfoFlags: %02X %02X", data->gestures[0], data->gestures[1], data->info_flags[0], data->info_flags[1]);
    LOG_DBG("F1_X: %d, F1_Y: %d", data->finger_1_x, data->finger_1_y);
    LOG_DBG("F2_X: %d, F2_Y: %d", data->finger_2_x, data->finger_2_y);
}

static int iqs7211e_set_event_mode(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    uint8_t command[2];
    int ret;

    ret = iqs7211e_read_bytes(&config->i2c, IQS7211E_MM_CONFIG_SETTINGS, command, sizeof(command));
    if (ret)
    {
        LOG_ERR("Failed to read system control register");
        return ret;
    }

    command[1] |= (1 << IQS7211E_EVENT_MODE_BIT);

    ret = iqs7211e_write_bytes(&config->i2c, IQS7211E_MM_CONFIG_SETTINGS, command, sizeof(command));
    if (ret)
    {
        LOG_ERR("Failed to write system control register");
        return ret;
    }

    LOG_INF("Event mode enabled");
    return 0;
}

static enum iqs7211e_power_mode iqs7211e_get_power_mode(const struct iqs7211e_data *data)
{
    uint8_t info_flags[2];
    int ret = iqs7211e_read_info_flags(data, info_flags);
    if (ret != 0)
    {
        return IQS7211E_POWER_UNKNOWN;
    }

    uint8_t buffer = iqs7211e_get_bit(info_flags[0], IQS7211E_CHARGING_MODE_BIT_0);
    buffer |= iqs7211e_get_bit(info_flags[0], IQS7211E_CHARGING_MODE_BIT_1) << 1;
    buffer |= iqs7211e_get_bit(info_flags[0], IQS7211E_CHARGING_MODE_BIT_2) << 2;

    if (buffer == IQS7211E_ACTIVE_BITS)
    {
        return IQS7211E_ACTIVE;
    }
    else if (buffer == IQS7211E_IDLE_TOUCH_BITS)
    {
        return IQS7211E_IDLE_TOUCH;
    }
    else if (buffer == IQS7211E_IDLE_BITS)
    {
        return IQS7211E_IDLE;
    }
    else if (buffer == IQS7211E_LP1_BITS)
    {
        return IQS7211E_LP1;
    }
    else if (buffer == IQS7211E_LP2_BITS)
    {
        return IQS7211E_LP2;
    }
    else
    {
        return IQS7211E_POWER_UNKNOWN;
    }
}

static enum iqs7211e_gestures_event iqs7211e_get_touchpad_event(const struct iqs7211e_data *data)
{
    if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_SINGLE_TAP_BIT))
    {
        return IQS7211E_GESTURE_SINGLE_TAP;
    }
    else if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_DOUBLE_TAP_BIT))
    {
        return IQS7211E_GESTURE_DOUBLE_TAP;
    }
    else if (iqs7211e_get_bit(data->gestures[0], IQS7211E_GESTURE_TRIPLE_TAP_BIT))
    {
        return IQS7211E_GESTURE_TRIPLE_TAP;
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

static uint8_t iqs7211e_get_num_fingers(const struct iqs7211e_data *data)
{
    uint8_t info_flags[2];
    int ret = iqs7211e_read_info_flags(data, info_flags);
    if (ret != 0)
    {
        return 0xFF; // 255 invalid number
    }

    uint8_t byte = info_flags[1];
    uint8_t num = iqs7211e_get_bit(byte, IQS7211E_NUM_FINGERS_BIT_0) |
                  (iqs7211e_get_bit(byte, IQS7211E_NUM_FINGERS_BIT_1) << 1);
    return num;
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
    if (ret)
        return ret;
    LOG_INF("1. ALP Compensation written");

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
    if (ret)
        return ret;
    LOG_INF("2. ATI Settings written");

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
    if (ret)
        return ret;
    LOG_INF("3. Report rates and timings written");

    /* 4. System Control Settings (0x33 - 0x35) */
    buf[0] = SYSTEM_CONTROL_0;
    buf[1] = SYSTEM_CONTROL_1;
    buf[2] = CONFIG_SETTINGS0;
    buf[3] = CONFIG_SETTINGS1;
    buf[4] = OTHER_SETTINGS_0;
    buf[5] = OTHER_SETTINGS_1;
    ret = iqs7211e_write_bytes(i2c, 0x33, buf, 6);
    if (ret)
        return ret;
    LOG_INF("4. System control settings written");

    /* 5. ALP Setup (0x36 - 0x37) */
    buf[0] = ALP_SETUP_0;
    buf[1] = ALP_SETUP_1;
    buf[2] = ALP_TX_ENABLE_0;
    buf[3] = ALP_TX_ENABLE_1;
    ret = iqs7211e_write_bytes(i2c, 0x36, buf, 4);
    if (ret)
        return ret;
    LOG_INF("5. ALP Setup written");

    /* 6. Threshold Settings (0x38 - 0x3A) */
    buf[0] = TRACKPAD_TOUCH_SET_THRESHOLD;
    buf[1] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
    buf[2] = ALP_THRESHOLD_0;
    buf[3] = ALP_THRESHOLD_1;
    buf[4] = ALP_SET_DEBOUNCE;
    buf[5] = ALP_CLEAR_DEBOUNCE;
    ret = iqs7211e_write_bytes(i2c, 0x38, buf, 6);
    if (ret)
        return ret;
    LOG_INF("6. Threshold settings written");

    /* 7. Filter Betas (0x3B - 0x3C) */
    buf[0] = ALP_COUNT_BETA_LP1;
    buf[1] = ALP_LTA_BETA_LP1;
    buf[2] = ALP_COUNT_BETA_LP2;
    buf[3] = ALP_LTA_BETA_LP2;
    ret = iqs7211e_write_bytes(i2c, 0x3B, buf, 4);
    if (ret)
        return ret;
    LOG_INF("7. Filter Betas written");

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
    if (ret)
        return ret;
    LOG_INF("8. Hardware settings written");

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
    if (ret)
        return ret;
    LOG_INF("9. TP Settings written");

    /* 10. Version Numbers (0x4A - 0x4A) */
    buf[0] = MINOR_VERSION;
    buf[1] = MAJOR_VERSION;
    ret = iqs7211e_write_bytes(i2c, 0x4A, buf, 2);
    if (ret)
        return ret;
    LOG_INF("10. Version numbers written");

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
    if (ret)
        return ret;
    LOG_INF("11. Gesture settings written");

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
    if (ret)
        return ret;
    LOG_INF("12. RxTx mapping written");

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
    if (ret)
        return ret;
    LOG_INF("13. Cycle 0-9 allocation written");

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
    if (ret)
        return ret;
    LOG_INF("14. Cycle 10-19 allocation written");

    /* 15. Allocation of channels into cycle 20 (0x7B - 0x7C) */
    /* Memory Map Position 0x7B - 0x7C */
    buf[0] = PLACEHOLDER_20;
    buf[1] = CH_1_CYCLE_20;
    buf[2] = CH_2_CYCLE_20;
    ret = iqs7211e_write_bytes(i2c, 0x7B, buf, 3);
    if (ret)
        return ret;
    LOG_INF("15. Write Cycle 20  Settings");

    return 0;
}

static int iqs7211e_read_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *buf, size_t len)
{
    int ret = i2c_write_read(i2c->bus, i2c->addr, &reg, 1, buf, len);
    if (ret)
    {
        LOG_ERR("i2c_read failed at reg 0x%02X (%zu bytes): %d", reg, len, ret);
    }
    return ret;
}

static int iqs7211e_write_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, const uint8_t *data, size_t numBytes)
{
    uint8_t buffer[numBytes + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, numBytes);
    int ret = i2c_write(i2c->bus, buffer, numBytes + 1, i2c->addr);
    if (ret)
    {
        LOG_ERR("i2c_write failed at reg 0x%02X (%zu bytes): %d", reg, numBytes, ret);
    }
    return ret;
}
static void iqs7211e_work_handler(struct k_work *work)
{
    struct iqs7211e_data *data = CONTAINER_OF(work, struct iqs7211e_data, work);
    if (iqs7211e_init_state(data))
    {
        iqs7211e_report_data(data);
    }
    set_interrupt(data->dev, true);
}

static void iqs7211e_report_data(struct iqs7211e_data *data)
{
    const struct iqs7211e_config *config = data->dev->config;
    iqs7211e_queue_value_updates(data);
    uint8_t num_fingers = iqs7211e_get_num_fingers(data);
    uint8_t gesture_event = iqs7211e_get_touchpad_event(data);
    LOG_INF("Fingers: %d, Gesture: %d", num_fingers, gesture_event);
    LOG_INF("Finger 1: X=%d, Y=%d", data->finger_1_x, data->finger_1_y);
    LOG_INF("Finger 2: X=%d, Y=%d", data->finger_2_x, data->finger_2_y);

    switch (gesture_event)
    {
    case IQS7211E_GESTURE_SINGLE_TAP:
        if (config->single_tap > 0)
        {
            input_report_key(data->dev, INPUT_BTN_0 + config->single_tap - 1, true, true, K_FOREVER);
            input_report_key(data->dev, INPUT_BTN_0 + config->single_tap - 1, false, true, K_FOREVER);
        }
        break;
    case IQS7211E_GESTURE_DOUBLE_TAP:
        if (config->double_tap > 0)
        {
            for (int i = 0; i < 2; i++)
            {
                input_report_key(data->dev, INPUT_BTN_0 + config->double_tap - 1, true, true, K_FOREVER);
                input_report_key(data->dev, INPUT_BTN_0 + config->double_tap - 1, false, true, K_FOREVER);
            }
        }
        break;
    case IQS7211E_GESTURE_TRIPLE_TAP:
        if (config->triple_tap > 0)
        {
            for (int i = 0; i < 3; i++)
            {
                input_report_key(data->dev, INPUT_BTN_0 + config->triple_tap - 1, true, true, K_FOREVER);
                input_report_key(data->dev, INPUT_BTN_0 + config->triple_tap - 1, false, true, K_FOREVER);
            }
        }
        break;
    case IQS7211E_GESTURE_PRESS_HOLD:
        if (config->press_hold > 0 && data->start_tap == 0)
        {
            input_report_key(data->dev, INPUT_BTN_0 + config->press_hold - 1, true, true, K_FOREVER);
            data->start_tap = 1;
        }
        break;
    default:
        break;
    }

    if (num_fingers == 0 && data->start_tap == 1)
    {
        input_report_key(data->dev, INPUT_BTN_0 + config->press_hold - 1, false, true, K_FOREVER);
        data->start_tap = 0;
    }

    if (num_fingers != 0 && config->scroll_layer > 0 && data->finger_1_x > SCROLL_START_X && !data->is_scroll_layer_active)
    {
        zmk_keymap_layer_deactivate(config->scroll_layer);
        data->is_scroll_layer_active = true;
        LOG_INF("Scroll layer deactivated");
    }

    if (num_fingers == 0 && data->is_scroll_layer_active)
    {
        zmk_keymap_layer_deactivate(config->scroll_layer);
        data->is_scroll_layer_active = false;
        LOG_INF("Scroll layer deactivated");
    }

    if (data->finger_1_prev_x == 0)
    {
        data->finger_1_prev_x = data->finger_1_x;
        data->finger_1_prev_dx = 0;
    }
    if (data->finger_1_prev_y == 0)
    {
        data->finger_1_prev_y = data->finger_1_y;
        data->finger_1_prev_dy = 0;
    }

    int16_t dx = calc_delta(data->finger_1_x, data->finger_1_prev_x);
    int16_t dy = calc_delta(data->finger_1_y, data->finger_1_prev_y);

    // smooth_dx = (dx + data->finger_1_prev_dx) / 2;
    int16_t smooth_dx = (dx + data->finger_1_prev_dx) >> 1;
    int16_t smooth_dy = (dy + data->finger_1_prev_dy) >> 1;

    if (num_fingers == 0)
    {
        data->touch_count = 0;
    }
    // Skip over 50 signal difference
    if (((dx - data->finger_1_prev_dx) > 50 && data->finger_1_prev_dx < 0) ||
        ((dx - data->finger_1_prev_dx) < -50 && data->finger_1_prev_dx > 0) ||
        ((dy - data->finger_1_prev_dy) > 50 && data->finger_1_prev_dy < 0) ||
        ((dy - data->finger_1_prev_dy) < -50 && data->finger_1_prev_dy > 0))
    {
        data->touch_count = 0;
    }
    data->touch_count++;

    if (data->touch_count > 3 && gesture_event == IQS7211E_GESTURE_NONE)
    {
        input_report_rel(data->dev, INPUT_REL_X, report_dx, false, K_NO_WAIT);
        input_report_rel(data->dev, INPUT_REL_Y, report_dy, true, K_NO_WAIT); // sync=true
    }
    data->finger_1_prev_x = data->finger_1_x;
    data->finger_1_prev_y = data->finger_1_y;
    data->finger_1_prev_dx = dx;
    data->finger_1_prev_dy = dy;
}

static int16_t calc_delta(uint16_t current, uint16_t prev)
{
    int delta = (int)current - (int)prev;
    if (delta > 32767)
        delta -= 65536;
    else if (delta < -32768)
        delta += 65536;
    return (int16_t)delta;
}

static void set_interrupt(const struct device *dev, const bool en)
{
    const struct iqs7211e_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_EDGE_FALLING : GPIO_INT_DISABLE);
    if (ret < 0)
    {
        LOG_ERR("Failed to set interrupt");
    }
}

static void iqs7211e_gpio_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    struct iqs7211e_data *data = CONTAINER_OF(cb, struct iqs7211e_data, gpio_cb);
    set_interrupt(data->dev, false);
    k_work_submit(&data->work);
}

int iqs7211e_init(const struct device *dev)
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
    if (ret)
    {
        LOG_ERR("Failed to configure IRQ pin: %d", ret);
        return ret;
    }

    gpio_init_callback(&data->gpio_cb, iqs7211e_gpio_callback, BIT(config->irq_gpio.pin));
    ret = gpio_add_callback(config->irq_gpio.port, &data->gpio_cb);
    if (ret)
    {
        LOG_ERR("Failed to add GPIO callback: %d", ret);
        return ret;
    }
    data->init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    data->dev = dev;

    k_work_init(&data->work, iqs7211e_work_handler);
    set_interrupt(data->dev, true);

    LOG_INF("IQS7211E driver initialized successfully");
    return 0;
}

#define IQS7211E_DEFINE(inst)                                           \
    static struct iqs7211e_data iqs7211e_data_##inst;                   \
    static const struct iqs7211e_config iqs7211e_config_##inst = {      \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                              \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),             \
        .single_tap = DT_PROP_OR(DT_DRV_INST(inst), single_tap, 0),     \
        .double_tap = DT_PROP_OR(DT_DRV_INST(inst), double_tap, 0),     \
        .triple_tap = DT_PROP_OR(DT_DRV_INST(inst), triple_tap, 0),     \
        .press_hold = DT_PROP_OR(DT_DRV_INST(inst), press_hold, 0),     \
        .scroll_layer = DT_PROP_OR(DT_DRV_INST(inst), scroll_layer, 0), \
    };                                                                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(inst,                                         \
                          &iqs7211e_init,                               \
                          NULL,                                         \
                          &iqs7211e_data_##inst,                        \
                          &iqs7211e_config_##inst,                      \
                          POST_KERNEL,                                  \
                          CONFIG_INPUT_INIT_PRIORITY,                   \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(IQS7211E_DEFINE)
