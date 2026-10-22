/*
 * Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 *
 * iqs7211e.c's setup state machine, bus helpers, RDY interrupt and recovery,
 * touch verification, click queueing and power management, against a model
 * of the sensor's memory map, built with CONFIG_PM_DEVICE. The driver's own header supplies its structures.
 */
#define CONFIG_PM_DEVICE 1
#define CONFIG_INPUT_MODE_THREAD 1
#define CONFIG_IQS7211E_WORKQUEUE_PRIORITY 5

#include <stdlib.h>
#include "iqs7211e_init.h"
#include "iqs7211e.h"
#include "iqs7211e_runtime.h"

#define IQS7211E_CLICK_EDGE_MS 20
#define IQS7211E_RDY_RECHECK_MS 1
#define IQS7211E_RDY_FAST_RECHECK_LIMIT 4
#define IQS7211E_RDY_RECHECK_BACKOFF_MS 20
#define IQS7211E_RESUME_FAST_RETRY_MS 20
#define IQS7211E_RESUME_FAST_RETRY_LIMIT 4
#define IQS7211E_RESUME_RETRY_BACKOFF_MS 1000
#define IQS7211E_BOOT_KICK_MS 200
#define IQS7211E_BOOT_KICK_RETRY_MS 200
#define IQS7211E_BOOT_KICK_ATTEMPTS 3
#define IQS7211E_BOOT_KICK_SETTLE_MS 50
enum pm_device_action { PM_DEVICE_ACTION_SUSPEND, PM_DEVICE_ACTION_RESUME,
                        PM_DEVICE_ACTION_TURN_OFF };

/* The driver's file-scope state and its forward declarations. */
static char iqs7211e_work_queue_stack[1024];
static struct k_work_q iqs7211e_work_queue;
static bool iqs7211e_work_queue_started;
static struct k_thread input_thread;
const k_tid_t input = &input_thread;
static struct device sensor;
static struct iqs7211e_data idle_data;
static struct device idle_sensor = {.name = "idle", .data = &idle_data};
static const struct device *const iqs7211e_devices[] = {&idle_sensor, &sensor};

static enum iqs7211e_gestures_event iqs7211e_get_touchpad_event(const struct iqs7211e_data *data);
static bool iqs7211e_init_state(struct iqs7211e_data *data);
static int iqs7211e_get_product_num(struct iqs7211e_data *data);
static int iqs7211e_read_info_flags(const struct iqs7211e_data *data, uint8_t *info_flags);
static int iqs7211e_check_reset(struct iqs7211e_data *data);
static int iqs7211e_acknowledge_reset(struct iqs7211e_data *data);
static bool iqs7211e_read_ati_active(struct iqs7211e_data *data);
static int iqs7211e_read_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, uint8_t *buf, size_t len);
static int iqs7211e_write_bytes(const struct i2c_dt_spec *i2c, uint8_t reg, const uint8_t *data,
                                size_t numBytes);
static void iqs7211e_work_handler(struct k_work *work);
static void iqs7211e_touch_verify_work_handler(struct k_work *work);
static void iqs7211e_click_work_handler(struct k_work *work);
static void iqs7211e_rdy_recheck_work_handler(struct k_work *work);
static void iqs7211e_boot_kick_work_handler(struct k_work *work);
static void iqs7211e_pm_release_work_handler(struct k_work *work);
static int iqs7211e_release_click(struct iqs7211e_data *data);
static void iqs7211e_queue_clicks(struct iqs7211e_data *data, uint16_t button, uint8_t clicks);
static int iqs7211e_report_abs_coordinates(struct iqs7211e_data *data, int16_t x, int16_t y);
static int iqs7211e_report_rel_coordinates(struct iqs7211e_data *data, int16_t dx, int16_t dy);
static int iqs7211e_report_data(struct iqs7211e_data *data);
static void iqs7211e_gpio_callback(const struct device *port, struct gpio_callback *cb,
                                   gpio_port_pins_t pins);
static int iqs7211e_write_defaults(struct iqs7211e_data *data);
static int iqs7211e_sw_reset(struct iqs7211e_data *data);
static int iqs7211e_set_suspend_state(struct iqs7211e_data *data, bool suspend);
static int iqs7211e_run_ati(struct iqs7211e_data *data);
static int iqs7211e_queue_value_updates(struct iqs7211e_data *data);
static int iqs7211e_set_event_mode(struct iqs7211e_data *data);
static bool iqs7211e_touch_verify_chain_alive(const struct iqs7211e_data *data);
static int iqs7211e_release_touch(struct iqs7211e_data *data);
static int iqs7211e_begin_runtime_reinitialization(struct iqs7211e_data *data);
static uint8_t iqs7211e_get_bit(uint8_t byte, uint8_t pos);
static uint8_t iqs7211e_get_num_fingers_from_info_flags(const uint8_t *info_flags);
static uint8_t iqs7211e_get_num_fingers(const struct iqs7211e_data *data);
static int set_gpio_interrupt(const struct device *dev, const bool en);
static int iqs7211e_init(const struct device *dev);
/* Called by functions lifted ahead of them. */
static int iqs7211e_reschedule_work(struct k_work_delayable *work, k_timeout_t delay);
static int iqs7211e_enable_interrupt_and_recheck(struct iqs7211e_data *data);

/* DRIVER_FUNCTIONS */

static struct device bus = {.name = "i2c0", .ready = true};
static struct device gpio_port = {.name = "gpio0", .ready = true};
static struct iqs7211e_config config = {
    .i2c = {&bus, 0x56},
    .irq_gpio = {&gpio_port, 9, 0},
    .single_tap = 0,
    .double_tap = 1,
    .triple_tap = 2,
    .report_abs = true,
    .jitter_deadband = 8,
    .touch_verify_interval_ms = 120,
};
static struct iqs7211e_data data;

static void run(struct k_work_delayable *work) {
    work->scheduled = false;
    work->work.handler(&work->work);
}

static void run_work(void) {
    data.work.pending = false;
    data.work.handler(&data.work);
}

static void set_product(uint16_t product) {
    REG(IQS7211E_MM_PROD_NUM)[0] = (uint8_t)product;
    REG(IQS7211E_MM_PROD_NUM)[1] = (uint8_t)(product >> 8);
}

static void rdy_edge(void) { added_callback->handler(&gpio_port, added_callback, BIT(9)); }
static void suspend_now(void) { atomic_set(&data.suspended, 1); }


/* ----- The settings image ----- */

/*
 * Every byte of every block the setup writes, against the name the settings
 * header gives it - with host touch verification on and off.
 */
static void test_default_settings(void) {
    for (int verify = 1; verify >= 0; verify--) {
        config.touch_verify_interval_ms = verify ? 120 : 0;
    /* Generated from iqs7211e_write_defaults(): register, then the byte each
     * position must hold, by the name the settings header gives it. */
#define HOST_VERIFY (config.touch_verify_interval_ms > 0U)
    const struct { uint8_t reg; uint8_t len; uint8_t bytes[30]; } defaults[] = {
        {0x1F, 4, {
            ALP_COMPENSATION_A_0,
            ALP_COMPENSATION_A_1,
            ALP_COMPENSATION_B_0,
            ALP_COMPENSATION_B_1,
        }},
        {0x21, 14, {
            TP_ATI_MULTIPLIERS_DIVIDERS_0,
            TP_ATI_MULTIPLIERS_DIVIDERS_1,
            TP_COMPENSATION_DIV,
            TP_REF_DRIFT_LIMIT,
            TP_ATI_TARGET_0,
            TP_ATI_TARGET_1,
            TP_MIN_COUNT_REATI_0,
            TP_MIN_COUNT_REATI_1,
            ALP_ATI_MULTIPLIERS_DIVIDERS_0,
            ALP_ATI_MULTIPLIERS_DIVIDERS_1,
            ALP_COMPENSATION_DIV,
            ALP_LTA_DRIFT_LIMIT,
            ALP_ATI_TARGET_0,
            ALP_ATI_TARGET_1,
        }},
        {0x28, 22, {
            ACTIVE_MODE_REPORT_RATE_0,
            ACTIVE_MODE_REPORT_RATE_1,
            IDLE_TOUCH_MODE_REPORT_RATE_0,
            IDLE_TOUCH_MODE_REPORT_RATE_1,
            IDLE_MODE_REPORT_RATE_0,
            IDLE_MODE_REPORT_RATE_1,
            LP1_MODE_REPORT_RATE_0,
            LP1_MODE_REPORT_RATE_1,
            LP2_MODE_REPORT_RATE_0,
            LP2_MODE_REPORT_RATE_1,
            ACTIVE_MODE_TIMEOUT_0,
            ACTIVE_MODE_TIMEOUT_1,
            HOST_VERIFY ? IDLE_TOUCH_MODE_TIMEOUT_HOST_VERIFY_0 : IDLE_TOUCH_MODE_TIMEOUT_FALLBACK_0,
            HOST_VERIFY ? IDLE_TOUCH_MODE_TIMEOUT_HOST_VERIFY_1 : IDLE_TOUCH_MODE_TIMEOUT_FALLBACK_1,
            IDLE_MODE_TIMEOUT_0,
            IDLE_MODE_TIMEOUT_1,
            LP1_MODE_TIMEOUT_0,
            LP1_MODE_TIMEOUT_1,
            REATI_RETRY_TIME,
            REF_UPDATE_TIME,
            I2C_TIMEOUT_0,
            I2C_TIMEOUT_1,
        }},
        {0x33, 6, {
            SYSTEM_CONTROL_0,
            SYSTEM_CONTROL_1,
            CONFIG_SETTINGS0,
            CONFIG_SETTINGS1,
            OTHER_SETTINGS_0,
            OTHER_SETTINGS_1,
        }},
        {0x36, 4, {
            ALP_SETUP_0,
            ALP_SETUP_1,
            ALP_TX_ENABLE_0,
            ALP_TX_ENABLE_1,
        }},
        {0x38, 6, {
            TRACKPAD_TOUCH_SET_THRESHOLD,
            TRACKPAD_TOUCH_CLEAR_THRESHOLD,
            ALP_THRESHOLD_0,
            ALP_THRESHOLD_1,
            ALP_SET_DEBOUNCE,
            ALP_CLEAR_DEBOUNCE,
        }},
        {0x3B, 4, {
            ALP_COUNT_BETA_LP1,
            ALP_LTA_BETA_LP1,
            ALP_COUNT_BETA_LP2,
            ALP_LTA_BETA_LP2,
        }},
        {0x3D, 8, {
            TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH,
            TP_CONVERSION_FREQUENCY_FRACTION_VALUE,
            ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH,
            ALP_CONVERSION_FREQUENCY_FRACTION_VALUE,
            TRACKPAD_HARDWARE_SETTINGS_0,
            TRACKPAD_HARDWARE_SETTINGS_1,
            ALP_HARDWARE_SETTINGS_0,
            ALP_HARDWARE_SETTINGS_1,
        }},
        {0x41, 18, {
            TRACKPAD_SETTINGS_0_0,
            TRACKPAD_SETTINGS_0_1,
            TRACKPAD_SETTINGS_1_0,
            TRACKPAD_SETTINGS_1_1,
            X_RESOLUTION_0,
            X_RESOLUTION_1,
            Y_RESOLUTION_0,
            Y_RESOLUTION_1,
            XY_DYNAMIC_FILTER_BOTTOM_SPEED_0,
            XY_DYNAMIC_FILTER_BOTTOM_SPEED_1,
            XY_DYNAMIC_FILTER_TOP_SPEED_0,
            XY_DYNAMIC_FILTER_TOP_SPEED_1,
            XY_DYNAMIC_FILTER_BOTTOM_BETA,
            XY_DYNAMIC_FILTER_STATIC_FILTER_BETA,
            STATIONARY_TOUCH_MOV_THRESHOLD,
            FINGER_SPLIT_FACTOR,
            X_TRIM_VALUE,
            Y_TRIM_VALUE,
        }},
        {0x4A, 2, {
            MINOR_VERSION,
            MAJOR_VERSION,
        }},
        {0x4B, 22, {
            GESTURE_ENABLE_0,
            GESTURE_ENABLE_1,
            TAP_TOUCH_TIME_0,
            TAP_TOUCH_TIME_1,
            TAP_WAIT_TIME_0,
            TAP_WAIT_TIME_1,
            TAP_DISTANCE_0,
            TAP_DISTANCE_1,
            HOLD_TIME_0,
            HOLD_TIME_1,
            SWIPE_TIME_0,
            SWIPE_TIME_1,
            SWIPE_X_DISTANCE_0,
            SWIPE_X_DISTANCE_1,
            SWIPE_Y_DISTANCE_0,
            SWIPE_Y_DISTANCE_1,
            SWIPE_X_CONS_DIST_0,
            SWIPE_X_CONS_DIST_1,
            SWIPE_Y_CONS_DIST_0,
            SWIPE_Y_CONS_DIST_1,
            SWIPE_ANGLE,
            PALM_THRESHOLD,
        }},
        {0x56, 14, {
            RX_TX_MAP_0,
            RX_TX_MAP_1,
            RX_TX_MAP_2,
            RX_TX_MAP_3,
            RX_TX_MAP_4,
            RX_TX_MAP_5,
            RX_TX_MAP_6,
            RX_TX_MAP_7,
            RX_TX_MAP_8,
            RX_TX_MAP_9,
            RX_TX_MAP_10,
            RX_TX_MAP_11,
            RX_TX_MAP_12,
            RX_TX_MAP_FILLER,
        }},
        {0x5D, 30, {
            PLACEHOLDER_0,
            CH_1_CYCLE_0,
            CH_2_CYCLE_0,
            PLACEHOLDER_1,
            CH_1_CYCLE_1,
            CH_2_CYCLE_1,
            PLACEHOLDER_2,
            CH_1_CYCLE_2,
            CH_2_CYCLE_2,
            PLACEHOLDER_3,
            CH_1_CYCLE_3,
            CH_2_CYCLE_3,
            PLACEHOLDER_4,
            CH_1_CYCLE_4,
            CH_2_CYCLE_4,
            PLACEHOLDER_5,
            CH_1_CYCLE_5,
            CH_2_CYCLE_5,
            PLACEHOLDER_6,
            CH_1_CYCLE_6,
            CH_2_CYCLE_6,
            PLACEHOLDER_7,
            CH_1_CYCLE_7,
            CH_2_CYCLE_7,
            PLACEHOLDER_8,
            CH_1_CYCLE_8,
            CH_2_CYCLE_8,
            PLACEHOLDER_9,
            CH_1_CYCLE_9,
            CH_2_CYCLE_9,
        }},
        {0x6C, 30, {
            PLACEHOLDER_10,
            CH_1_CYCLE_10,
            CH_2_CYCLE_10,
            PLACEHOLDER_11,
            CH_1_CYCLE_11,
            CH_2_CYCLE_11,
            PLACEHOLDER_12,
            CH_1_CYCLE_12,
            CH_2_CYCLE_12,
            PLACEHOLDER_13,
            CH_1_CYCLE_13,
            CH_2_CYCLE_13,
            PLACEHOLDER_14,
            CH_1_CYCLE_14,
            CH_2_CYCLE_14,
            PLACEHOLDER_15,
            CH_1_CYCLE_15,
            CH_2_CYCLE_15,
            PLACEHOLDER_16,
            CH_1_CYCLE_16,
            CH_2_CYCLE_16,
            PLACEHOLDER_17,
            CH_1_CYCLE_17,
            CH_2_CYCLE_17,
            PLACEHOLDER_18,
            CH_1_CYCLE_18,
            CH_2_CYCLE_18,
            PLACEHOLDER_19,
            CH_1_CYCLE_19,
            CH_2_CYCLE_19,
        }},
        {0x7B, 3, {
            PLACEHOLDER_20,
            CH_1_CYCLE_20,
            CH_2_CYCLE_20,
        }},
    };

        memset(regs, 0xa5, sizeof(regs));
        const int writes = i2c_writes;
        assert(iqs7211e_write_defaults(&data) == 0);
        assert(i2c_writes == writes + (int)ARRAY_SIZE(defaults));
        for (size_t b = 0; b < ARRAY_SIZE(defaults); b++) {
            for (size_t i = 0; i < defaults[b].len; i++) {
                if (REG(defaults[b].reg)[i] != defaults[b].bytes[i]) {
                    fprintf(stderr, "register 0x%02x byte %zu: 0x%02x, want 0x%02x\n",
                            defaults[b].reg, i, REG(defaults[b].reg)[i], defaults[b].bytes[i]);
                    assert(false);
                }
            }
        }
    }
    config.touch_verify_interval_ms = 120;
}
#undef HOST_VERIFY

/* ----- Init and the work queue ----- */

static void test_init(void) {
    bus.ready = false;
    assert(iqs7211e_init(&sensor) == -ENODEV);
    bus.ready = true;
    gpio_port.ready = false;
    assert(iqs7211e_init(&sensor) == -ENODEV);
    gpio_port.ready = true;
    pin_configure_result = -EIO;
    assert(iqs7211e_init(&sensor) == -EIO);
    pin_configure_result = 0;
    add_callback_result = -ENOMEM;
    assert(iqs7211e_init(&sensor) == -ENOMEM);
    add_callback_result = 0;
    /* The first arm of RDY can fail: init reports it. */
    irq_configure_result = -EIO;
    assert(iqs7211e_init(&sensor) == -EIO && !data.boot_kick_work.scheduled);
    irq_configure_result = 0;

    data.last_touched_state = data.touch_verify_pending = data.sensor_suspended = true;
    data.touch_count = 3;
    data.boot_irq_seen = 1;
    data.suspended = data.rdy_recheck_attempts = 3;
    data.sensor_resume_attempts = data.boot_kick_attempts = 3;
    data.pm_release_ret = -3;
    data.click_edges = 3;
    assert(iqs7211e_init(&sensor) == 0);
    assert(!data.last_touched_state && !data.touch_verify_pending && !data.sensor_suspended);
    assert(data.touch_count == 0 && data.click_edges == 0 && data.pm_release_ret == 0);
    assert(data.boot_irq_seen == 0);
    assert(data.suspended == 0 && data.rdy_recheck_attempts == 0);
    assert(data.sensor_resume_attempts == 0 && data.boot_kick_attempts == 0);
    assert(data.click_work.work.handler == iqs7211e_click_work_handler);
    assert(data.boot_kick_work.work.handler == iqs7211e_boot_kick_work_handler);
    assert(iqs7211e_work_queue_started && iqs7211e_work_queue.priority == 5);
    assert(data.init_state == IQS7211E_INIT_VERIFY_PRODUCT && data.dev == &sensor);
    assert(irq_mode == GPIO_INT_EDGE_TO_ACTIVE && atomic_get(&data.irq_enabled));
    assert(data.rdy_recheck_work.scheduled && data.rdy_recheck_work.delay_ms == 1);
    assert(data.boot_kick_work.scheduled && data.boot_kick_work.delay_ms == IQS7211E_BOOT_KICK_MS);
    /* The queue is started once, however many sensors init. */
    iqs7211e_start_work_queue();

    /* Queue failures are logged and returned. */
    const int errors = log_errors;
    submit_result = -EBUSY;
    assert(iqs7211e_submit_work(&data.work) == -EBUSY);
    submit_result = 0;
    reschedule_result = -EINVAL;
    assert(iqs7211e_reschedule_work(&data.click_work, K_NO_WAIT) == -EINVAL);
    reschedule_result = 0;
    assert(log_errors == errors + 2);
    data.work.pending = true;
    assert(iqs7211e_submit_work(&data.work) == 0);
    data.work.pending = false;
    data.click_work.scheduled = true;
    assert(iqs7211e_reschedule_work(&data.click_work, K_NO_WAIT) == 0);
    data.click_work.scheduled = false;
    assert(log_errors == errors + 2);
}

/* ----- The setup sequence ----- */

static void step(void) { assert(!iqs7211e_init_state(&data)); }

static void test_setup(void) {
    /* The product number: a failed read retries, a wrong part stops for good. */
    data.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    read_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_VERIFY_PRODUCT);
    set_product(0x1234);
    step();
    assert(data.init_state == IQS7211E_INIT_NONE);
    data.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    set_product(0);
    step();
    assert(data.init_state == IQS7211E_INIT_NONE);
    step(); /* nothing to do once disabled */
    data.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    set_product(IQS7211E_PRODUCT_NUM);
    step();
    assert(data.init_state == IQS7211E_INIT_READ_RESET);

    /* No reset seen: issue one, and again if it is not seen either. */
    read_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_READ_RESET);
    REG(IQS7211E_MM_INFO_FLAGS)[0] = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_CHIP_RESET);
    read_fail_after = 0;
    step();
    write_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_CHIP_RESET && !data.reset_called);
    memset(REG(IQS7211E_MM_SYS_CONTROL), 0, 2);
    step();
    assert(data.reset_called && data.init_state == IQS7211E_INIT_READ_RESET);
    assert(REG(IQS7211E_MM_SYS_CONTROL)[1] & (1 << IQS7211E_SW_RESET_BIT));
    step();
    assert(!data.reset_called && data.init_state == IQS7211E_INIT_READ_RESET && log_warnings > 0);
    data.init_state = IQS7211E_INIT_CHIP_RESET;
    data.reset_called = true;
    step(); /* already issued: wait */
    assert(data.init_state == IQS7211E_INIT_CHIP_RESET);
    data.init_state = IQS7211E_INIT_READ_RESET;
    REG(IQS7211E_MM_INFO_FLAGS)[0] = IQS7211E_RUNTIME_SHOW_RESET_MASK;
    step();
    assert(data.init_state == IQS7211E_INIT_UPDATE_SETTINGS && !data.reset_called);

    /* The settings go out in fifteen blocks; any one failing retries all. */
    for (int block = 0; block < 15; block++) {
        write_fail_after = block;
        step();
        assert(data.init_state == IQS7211E_INIT_UPDATE_SETTINGS);
    }
    const int writes = i2c_writes;
    step();
    assert(data.init_state == IQS7211E_INIT_ACK_RESET && i2c_writes == writes + 15);
    assert(last_write_reg == 0x7B && last_write_len == 3);
    /* The idle-touch timeout depends on whether the host verifies touches. */
    const uint8_t verified = REG(0x28)[12];
    config.touch_verify_interval_ms = 0;
    assert(iqs7211e_write_defaults(&data) == 0);
    assert(REG(0x28)[12] == IDLE_TOUCH_MODE_TIMEOUT_FALLBACK_0 &&
           REG(0x28)[13] == IDLE_TOUCH_MODE_TIMEOUT_FALLBACK_1);
    assert(verified == IDLE_TOUCH_MODE_TIMEOUT_HOST_VERIFY_0);
    config.touch_verify_interval_ms = 120;

    /* Acknowledge the reset, then run ATI; either half of each can fail. */
    read_fail_after = 0;
    step();
    write_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_ACK_RESET);
    memset(REG(IQS7211E_MM_SYS_CONTROL), 0, 2);
    step();
    assert(data.init_state == IQS7211E_INIT_ATI &&
           (REG(IQS7211E_MM_SYS_CONTROL)[0] & (1 << IQS7211E_ACK_RESET_BIT)));
    read_fail_after = 0;
    step();
    write_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_ATI);
    memset(REG(IQS7211E_MM_SYS_CONTROL), 0, 2);
    step();
    assert(data.init_state == IQS7211E_INIT_WAIT_FOR_ATI);
    assert(REG(IQS7211E_MM_SYS_CONTROL)[0] & (1 << IQS7211E_TP_RE_ATI_BIT));

    /* ATI runs until the part says it has; an unreadable flag means running. */
    read_fail_after = 0;
    step();
    REG(IQS7211E_MM_INFO_FLAGS)[0] = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_WAIT_FOR_ATI);
    REG(IQS7211E_MM_INFO_FLAGS)[0] = 1 << IQS7211E_RE_ATI_OCCURRED_BIT;
    step();
    assert(data.init_state == IQS7211E_INIT_READ_DATA);

    /* One read of the report, then event mode. */
    read_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_READ_DATA);
    for (int i = 0; i < 12; i++) {
        REG(IQS7211E_MM_GESTURES)[i] = (uint8_t)(0x10 + i);
    }
    step();
    assert(data.init_state == IQS7211E_INIT_ACTIVATE_EVENT_MODE);
    assert(data.gestures[0] == 0x10 && data.gestures[1] == 0x11);
    assert(data.info_flags[0] == 0x12 && data.info_flags[1] == 0x13);
    assert(data.finger_1_x == 0x1514 && data.finger_1_y == 0x1716);
    assert(data.finger_1_touch_strength == 0x1918 && data.finger_1_area == 0x1b1a);
    read_fail_after = 0;
    step();
    write_fail_after = 0;
    step();
    assert(data.init_state == IQS7211E_INIT_ACTIVATE_EVENT_MODE);
    memset(REG(IQS7211E_MM_CONFIG_SETTINGS), 0, 2);
    step();
    assert(data.init_state == IQS7211E_INIT_DONE &&
           (REG(IQS7211E_MM_CONFIG_SETTINGS)[1] & (1 << IQS7211E_EVENT_MODE_BIT)));
    assert(iqs7211e_init_state(&data));

    /* A state with no step of its own does nothing. */
    data.init_state = IQS7211E_INIT_ACTIVATE_STREAM_MODE;
    step();
    data.init_state = IQS7211E_INIT_DONE;
}

static void test_readers(void) {
    /* Gestures, highest priority first. */
    static const struct { uint8_t byte; uint8_t bit; int event; } gestures[] = {
        {0, IQS7211E_GESTURE_TRIPLE_TAP_BIT, IQS7211E_GESTURE_TRIPLE_TAP},
        {0, IQS7211E_GESTURE_DOUBLE_TAP_BIT, IQS7211E_GESTURE_DOUBLE_TAP},
        {0, IQS7211E_GESTURE_SINGLE_TAP_BIT, IQS7211E_GESTURE_SINGLE_TAP},
        {0, IQS7211E_GESTURE_PALM_GESTURE_BIT, IQS7211E_GESTURE_PALM_GESTURE},
        {1, IQS7211E_GESTURE_SWIPE_X_POSITIVE_BIT, IQS7211E_GESTURE_SWIPE_X_POSITIVE},
        {1, IQS7211E_GESTURE_SWIPE_X_NEGATIVE_BIT, IQS7211E_GESTURE_SWIPE_X_NEGATIVE},
        {1, IQS7211E_GESTURE_SWIPE_Y_POSITIVE_BIT, IQS7211E_GESTURE_SWIPE_Y_POSITIVE},
        {1, IQS7211E_GESTURE_SWIPE_Y_NEGATIVE_BIT, IQS7211E_GESTURE_SWIPE_Y_NEGATIVE},
        {1, IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE_BIT, IQS7211E_GESTURE_SWIPE_HOLD_X_POSITIVE},
        {1, IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE_BIT, IQS7211E_GESTURE_SWIPE_HOLD_X_NEGATIVE},
        {1, IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE_BIT, IQS7211E_GESTURE_SWIPE_HOLD_Y_POSITIVE},
        {1, IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE_BIT, IQS7211E_GESTURE_SWIPE_HOLD_Y_NEGATIVE},
    };
    for (size_t i = 0; i < ARRAY_SIZE(gestures); i++) {
        data.gestures[0] = data.gestures[1] = 0;
        data.gestures[gestures[i].byte] = (uint8_t)(1U << gestures[i].bit);
        assert((int)iqs7211e_get_touchpad_event(&data) == gestures[i].event);
    }
    /* Triple wins over everything else in the same packet. */
    data.gestures[0] = 0xff;
    data.gestures[1] = 0xff;
    assert(iqs7211e_get_touchpad_event(&data) == IQS7211E_GESTURE_TRIPLE_TAP);
    data.gestures[0] = data.gestures[1] = 0;
    assert(iqs7211e_get_touchpad_event(&data) == IQS7211E_GESTURE_NONE);

    data.info_flags[1] = 0x03;
    assert(iqs7211e_get_num_fingers(&data) == 3);
    data.info_flags[1] = 0x00;
    assert(iqs7211e_get_num_fingers(&data) == 0);
}

/* ----- RDY: the interrupt, the report work and the level recheck ----- */

static void test_interrupt(void) {
    /* An edge masks RDY and hands the window to the report work, and ends
     * any recheck back-off. */
    atomic_set(&data.irq_enabled, 1);
    atomic_set(&data.rdy_recheck_attempts, 3);
    rdy_edge();
    assert(data.rdy_recheck_attempts == 0);
    assert(irq_mode == GPIO_INT_DISABLE && !atomic_get(&data.irq_enabled) && data.work.pending);
    data.work.pending = false;

    /* While suspended an edge is only counted. */
    suspend_now();
    atomic_clear(&data.boot_irq_seen);
    rdy_edge();
    assert(data.boot_irq_seen == 1 && !data.work.pending);
    atomic_clear(&data.suspended);

    /* Masking or queueing can fail: RDY is re-armed rather than left deaf,
     * and a failure to re-arm is reported too. */
    irq_fail_after = 0;
    rdy_edge();
    assert(atomic_get(&data.irq_enabled) && !data.work.pending);
    submit_result = -EBUSY;
    rdy_edge();
    assert(atomic_get(&data.irq_enabled));
    irq_fail_after = 1; /* the mask works, the re-arm does not */
    const int errors = log_errors;
    rdy_edge();
    assert(log_errors >= errors + 2 && !atomic_get(&data.irq_enabled));
    /* A suspend landing as RDY is masked keeps it masked. */
    atomic_set(&data.irq_enabled, 1);
    on_mask = suspend_now;
    rdy_edge();
    assert(!atomic_get(&data.irq_enabled));
    atomic_clear(&data.suspended);
    submit_result = 0;

    /* A failed configure leaves the published state as it was. */
    atomic_set(&data.irq_enabled, 1);
    irq_fail_after = 0;
    assert(set_gpio_interrupt(&sensor, false) == -EIO && atomic_get(&data.irq_enabled));
}

static void test_work(void) {
    /* Suspended: nothing runs. */
    suspend_now();
    run_work();
    atomic_clear(&data.suspended);

    /* Setup not finished: one step, then RDY is re-armed. */
    data.init_state = IQS7211E_INIT_VERIFY_PRODUCT;
    set_product(IQS7211E_PRODUCT_NUM);
    rdy_level = 1;
    run_work();
    assert(data.init_state == IQS7211E_INIT_READ_RESET && atomic_get(&data.irq_enabled));

    /* Set up: a report, counted; the RDY sample can fail and is logged. */
    data.init_state = IQS7211E_INIT_DONE;
    memset(&REG(IQS7211E_MM_GESTURES)[0], 0, 12);
    rdy_level = 0;
    rdy_raw_result = -EIO;
    run_work();
    rdy_raw_result = 0;

    /* A failed report during a touch verification releases the touch. */
    data.touch_verify_pending = true;
    data.last_touched_state = true;
    read_fail_after = 0;
    run_work();
    assert(!data.last_touched_state);
    assert(!data.touch_verify_pending);
    /* ...but not outside one. */
    data.last_touched_state = true;
    read_fail_after = 0;
    run_work();
    assert(data.last_touched_state);
    /* ...nor when the report lands. */
    uint8_t *packet = REG(IQS7211E_MM_GESTURES);
    packet[3] = 1 << IQS7211E_NUM_FINGERS_BIT_0;
    packet[4] = packet[6] = 100; /* x, y */
    packet[8] = 50;              /* strength */
    packet[10] = 5;              /* area */
    data.touch_verify_pending = true;
    run_work();
    assert(data.last_touched_state);
    assert(!data.touch_verify_pending);
    memset(packet, 0, 12);
    data.touch_count = 0;
    data.last_touched_state = false;
    data.touch_release_pending = false;

    /* Suspended while the work ran: RDY is left masked. Failing to re-arm is
     * logged. */
    atomic_set(&data.irq_enabled, 0);
    on_read = suspend_now;
    run_work();
    assert(!atomic_get(&data.irq_enabled));
    atomic_clear(&data.suspended);
    irq_fail_after = 0;
    const int errors = log_errors;
    run_work();
    assert(log_errors == errors + 2);


}

/* The callback firing as RDY is armed, because its level is already active. */
static void active_at_arm(void) { rdy_edge(); }

static void test_rdy_recheck(void) {
    atomic_clear(&data.rdy_recheck_attempts);

    /* Suspended: nothing. */
    suspend_now();
    run(&data.rdy_recheck_work);
    atomic_clear(&data.suspended);
    /* A sensor that did not wake is woken first; while that fails, the
     * recheck backs off and does nothing else. */
    data.sensor_suspended = true;
    write_fail_after = 0;
    atomic_set(&data.rdy_recheck_attempts, 1);
    atomic_set(&data.irq_enabled, 1);
    run(&data.rdy_recheck_work);
    assert(data.rdy_recheck_attempts == 1);
    assert(data.sensor_resume_attempts == 1 &&
           data.rdy_recheck_work.delay_ms == IQS7211E_RESUME_FAST_RETRY_MS);
    for (int i = 2; i <= IQS7211E_RESUME_FAST_RETRY_LIMIT; i++) {
        write_fail_after = 0;
        run(&data.rdy_recheck_work);
    }
    assert(data.rdy_recheck_work.delay_ms == IQS7211E_RESUME_RETRY_BACKOFF_MS);
    data.sensor_resume_attempts = UINT8_MAX;
    write_fail_after = 0;
    run(&data.rdy_recheck_work);
    assert(data.sensor_resume_attempts == UINT8_MAX);
    rdy_level = 0;
    REG(IQS7211E_MM_SYS_CONTROL)[1] = BIT(IQS7211E_SUSPEND_BIT);
    run(&data.rdy_recheck_work);
    assert(!data.sensor_suspended && data.sensor_resume_attempts == 0);
    assert(!(REG(IQS7211E_MM_SYS_CONTROL)[1] & BIT(IQS7211E_SUSPEND_BIT)));
    assert(data.rdy_recheck_attempts == 0);

    /* An interrupt the GPIO controller never took is armed now; failing, it
     * retries; an active level fires at once and leaves it masked. */
    atomic_set(&data.irq_enabled, 0);
    irq_fail_after = 0;
    run(&data.rdy_recheck_work);
    assert(!atomic_get(&data.irq_enabled) && data.rdy_recheck_attempts == 1);
    on_arm = active_at_arm;
    run(&data.rdy_recheck_work);
    assert(!atomic_get(&data.irq_enabled) && data.work.pending);
    data.work.pending = false;
    atomic_set(&data.irq_enabled, 0);
    atomic_set(&data.rdy_recheck_attempts, 1);
    run(&data.rdy_recheck_work); /* armed, level inactive: done */
    assert(atomic_get(&data.irq_enabled) && data.rdy_recheck_attempts == 0);

    /* A level that cannot be read retries. */
    rdy_result = -EIO;
    run(&data.rdy_recheck_work);
    assert(data.rdy_recheck_attempts == 1 && data.rdy_recheck_work.scheduled);
    rdy_result = 0;

    /* An active level missed while masked is recovered, fast at first and
     * then backing off. */
    rdy_level = 1;
    for (int i = 0; i < IQS7211E_RDY_FAST_RECHECK_LIMIT + 1; i++) {
        atomic_set(&data.irq_enabled, 1);
        const int arms = irq_arms;
        run(&data.rdy_recheck_work);
        assert(data.work.pending && !atomic_get(&data.irq_enabled) && irq_arms == arms);
        data.work.pending = false;
    }
    assert(data.rdy_recheck_attempts == IQS7211E_RDY_FAST_RECHECK_LIMIT);
    assert(iqs7211e_rdy_recheck_delay(&data).ms == IQS7211E_RDY_RECHECK_BACKOFF_MS);
    iqs7211e_retry_rdy_recheck(&data); /* already at the limit */
    assert(data.rdy_recheck_attempts == IQS7211E_RDY_FAST_RECHECK_LIMIT);
    /* The report work already queued takes the level as it is: RDY stays
     * masked, from the recheck and from an edge alike. */
    atomic_set(&data.irq_enabled, 1);
    data.work.pending = true;
    run(&data.rdy_recheck_work);
    assert(!atomic_get(&data.irq_enabled));
    atomic_set(&data.irq_enabled, 1);
    rdy_edge();
    assert(!atomic_get(&data.irq_enabled) && data.work.pending);
    data.work.pending = false;
    suspend_now();
    data.rdy_recheck_work.scheduled = false;
    iqs7211e_schedule_rdy_recheck(&data);
    assert(!data.rdy_recheck_work.scheduled);
    atomic_clear(&data.suspended);

    /* Masking can fail; so can queueing, after which RDY is re-armed unless a
     * suspend landed, and a failed re-arm is logged. */
    atomic_set(&data.irq_enabled, 1);
    irq_fail_after = 0;
    atomic_set(&data.rdy_recheck_attempts, 1);
    data.rdy_recheck_work.scheduled = false;
    run(&data.rdy_recheck_work);
    /* one attempt for the active level, one more for the failed mask */
    assert(!data.work.pending && data.rdy_recheck_attempts == 3 && data.rdy_recheck_work.scheduled);
    submit_result = -EBUSY;
    run(&data.rdy_recheck_work);
    assert(atomic_get(&data.irq_enabled));
    irq_fail_after = 1;
    run(&data.rdy_recheck_work);
    assert(!atomic_get(&data.irq_enabled));
    /* A suspend landing as RDY is masked leaves it masked. */
    atomic_set(&data.irq_enabled, 1);
    on_mask = suspend_now;
    submit_result = -EBUSY;
    run(&data.rdy_recheck_work);
    assert(!atomic_get(&data.irq_enabled));
    atomic_clear(&data.suspended);
    submit_result = 0;
    rdy_level = 0;
    atomic_clear(&data.rdy_recheck_attempts);
}

/* ----- Touch verification and clicks ----- */

static void test_touch_verify(void) {
    /* Nothing to verify: suspended, the feature off, or no touch. */
    suspend_now();
    run(&data.touch_verify_work);
    atomic_clear(&data.suspended);
    data.last_touched_state = false;
    run(&data.touch_verify_work);
    assert(!data.touch_verify_pending);

    /* A touch: the regular report work reads the whole packet. */
    data.last_touched_state = true;
    atomic_set(&data.irq_enabled, 1);
    atomic_set(&data.rdy_recheck_attempts, 3);
    run(&data.touch_verify_work);
    assert(data.rdy_recheck_attempts == 0);
    assert(data.touch_verify_pending && data.work.pending && !atomic_get(&data.irq_enabled));
    data.work.pending = false;

    /* The report work already queued: the touch stays. */
    data.work.pending = true;
    run(&data.touch_verify_work);
    assert(data.touch_verify_pending && data.last_touched_state);
    data.work.pending = false;
    /* It cannot be queued: the touch is released and RDY re-armed. */
    input_count = 0;
    submit_result = -EBUSY;
    run(&data.touch_verify_work);
    assert(!data.touch_verify_pending && !data.last_touched_state && atomic_get(&data.irq_enabled));
    assert(input_count == 3 && inputs[0].code == INPUT_BTN_TOUCH && inputs[0].value == 0);
    /* ...and the mask failing is treated the same; so is a failed re-arm. */
    data.last_touched_state = true;
    irq_fail_after = 0;
    submit_result = 0;
    run(&data.touch_verify_work);
    assert(!data.last_touched_state);
    data.last_touched_state = true;
    submit_result = -EBUSY;
    irq_fail_after = 1; /* the mask works, the re-arm does not */
    const int errors = log_errors;
    run(&data.touch_verify_work);
    assert(log_errors >= errors + 2 && !atomic_get(&data.irq_enabled));
    /* A suspend landing as RDY is masked keeps it masked. */
    data.last_touched_state = true;
    atomic_set(&data.irq_enabled, 1);
    on_mask = suspend_now;
    run(&data.touch_verify_work);
    assert(!data.last_touched_state && !atomic_get(&data.irq_enabled));
    atomic_clear(&data.suspended);
    submit_result = 0;
    config.touch_verify_interval_ms = 0;
    data.last_touched_state = true;
    run(&data.touch_verify_work); /* switched off */
    assert(data.last_touched_state);
    data.last_touched_state = false;
    config.touch_verify_interval_ms = 120;
}

static void test_clicks(void) {
    /* Suspended: no clicks. */
    suspend_now();
    iqs7211e_queue_clicks(&data, INPUT_BTN_0, 2);
    assert(data.click_edges == 0);
    atomic_clear(&data.suspended);

    iqs7211e_queue_clicks(&data, INPUT_BTN_0, 2);
    assert(data.click_edges == 4 && data.click_work.scheduled && data.click_button == INPUT_BTN_0);

    /* A new tap while a press is outstanding releases it first. */
    data.click_edges = 3;
    input_count = 0;
    iqs7211e_queue_clicks(&data, INPUT_BTN_1, 1);
    assert(input_count == 1 && inputs[0].value == 0 && data.click_edges == 2);
    data.click_edges = 1;
    input_count = 0;
    iqs7211e_queue_clicks(&data, INPUT_BTN_1, 1);
    assert(input_count == 1 && inputs[0].value == 0 && data.click_edges == 2);
    /* If that release fails, the new tap is dropped. */
    data.click_edges = 3;
    input_fail_after = 0;
    iqs7211e_queue_clicks(&data, INPUT_BTN_2, 3);
    assert(data.click_edges == 3 && data.click_button == INPUT_BTN_1);
    data.click_edges = 0;
    /* If it cannot be scheduled, nothing is owed. */
    reschedule_result = -EINVAL;
    iqs7211e_queue_clicks(&data, INPUT_BTN_2, 3);
    assert(data.click_edges == 0);
    reschedule_result = 0;
}

/* ----- Power management ----- */

static void touch_and_click(void) {
    data.last_touched_state = true;
    data.click_edges = 3;
    data.click_button = INPUT_BTN_0;
}

static void test_pm_release(void) {
    /* On the driver's own queue the release runs inline; elsewhere it is
     * handed over and waited for. */
    touch_and_click();
    current_thread = &iqs7211e_work_queue.thread;
    assert(iqs7211e_release_before_suspend(&data) == 0 && data.click_edges == 0);
    current_thread = NULL;
    touch_and_click();
    assert(iqs7211e_release_before_suspend(&data) == 0 && flushes == 1);
    touch_and_click();
    data.pm_release_work.pending = true;
    assert(iqs7211e_release_before_suspend(&data) == 0 && flushes == 2);
    assert(data.click_edges == 0 && !data.last_touched_state);
    data.pm_release_ret = -5;
    data.last_touched_state = false;
    data.click_edges = 0;
    assert(iqs7211e_release_before_suspend(&data) == 0);
    submit_result = -EBUSY;
    assert(iqs7211e_release_before_suspend(&data) == -EBUSY);
    submit_result = 0;

    /* The first failure is what is reported. */
    touch_and_click();
    input_fail_after = 0;
    iqs7211e_pm_release_work_handler(&data.pm_release_work);
    assert(data.pm_release_ret < 0);
    data.click_edges = 0;
    data.last_touched_state = true;
    input_fail_after = 0;
    iqs7211e_pm_release_work_handler(&data.pm_release_work);
    assert(data.pm_release_ret < 0);
    touch_and_click();
    input_fail_all = true; /* both fail: the click's error is the one kept */
    iqs7211e_pm_release_work_handler(&data.pm_release_work);
    assert(data.pm_release_ret == -EAGAIN);
    input_fail_all = false;
    input_fail_after = 1;
    data.touch_release_pending = true;
    iqs7211e_pm_release_work_handler(&data.pm_release_work);
    input_fail_after = -1;
    data.touch_release_pending = false;
    data.last_touched_state = false;
    data.click_edges = 0;
    iqs7211e_pm_release_work_handler(&data.pm_release_work);
    assert(data.pm_release_ret == 0);
}

static void test_pm(void) {
    data.init_state = IQS7211E_INIT_DONE;
    data.sensor_suspended = false;

    /* A suspend that cannot mask RDY is refused and undone. */
    irq_fail_after = 0;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EIO);
    assert(!atomic_get(&data.suspended));

    /* The second mask failing, or the releases failing, restores the driver:
     * owed releases and a live verify chain are rescheduled, RDY re-armed. */
    irq_fail_after = 1;
    touch_and_click();
    data.touch_verify_pending = true;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EIO);
    assert(!data.touch_verify_pending);
    assert(!atomic_get(&data.suspended) && data.click_work.scheduled &&
           data.touch_verify_work.scheduled);
    data.touch_verify_work.scheduled = data.click_work.scheduled = false;
    data.last_touched_state = false;
    data.click_edges = 1;
    irq_fail_after = 1;
    atomic_set(&data.rdy_recheck_attempts, 3);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EIO);
    assert(data.click_work.scheduled && data.rdy_recheck_attempts == 0);
    data.click_work.scheduled = false;
    data.click_edges = 0;
    data.touch_release_pending = true;
    submit_result = -EBUSY;
    irq_fail_after = 2; /* and the re-arm after it fails too */
    const int errors = log_errors;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EBUSY);
    assert(data.click_work.scheduled && log_errors > errors);
    submit_result = 0;
    data.touch_release_pending = false;
    /* Nothing owed and no touch: only RDY is re-armed. */
    data.click_work.scheduled = false;
    irq_fail_after = 1;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EIO);
    assert(!data.click_work.scheduled && !atomic_get(&data.suspended));

    /* A clean suspend puts the sensor to sleep too; a dead bus only warns. */
    write_fail_after = 0;
    const int arms = irq_arms;
    data.touch_verify_pending = true;
    data.work.pending = true;
    data.click_work.scheduled = true;
    data.sensor_resume_attempts = 4;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(!data.work.pending && !data.click_work.scheduled && data.sensor_resume_attempts == 0);
    assert(atomic_get(&data.suspended) && !data.sensor_suspended);
    assert(irq_arms == arms && irq_mode == GPIO_INT_DISABLE && !data.touch_verify_pending);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(data.sensor_suspended && (REG(IQS7211E_MM_SYS_CONTROL)[1] & BIT(IQS7211E_SUSPEND_BIT)));
    assert(REG(IQS7211E_MM_SYS_CONTROL)[0] == SYSTEM_CONTROL_0 &&
           REG(IQS7211E_MM_SYS_CONTROL)[1] == (uint8_t)(SYSTEM_CONTROL_1 | BIT(IQS7211E_SUSPEND_BIT)));
    /* Asleep already, or never set up: no write. */
    const int writes = i2c_writes;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0 && i2c_writes == writes);
    data.init_state = IQS7211E_INIT_NONE;
    data.sensor_suspended = false;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0 && i2c_writes == writes);
    data.sensor_suspended = true;

    /* Resume wakes the sensor, best effort, and starts setup again. */
    write_fail_after = 0;
    data.reset_called = data.last_touched_state = data.touch_verify_pending = true;
    data.touch_count = 4;
    data.click_edges = 3;
    atomic_set(&data.rdy_recheck_attempts, 3);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(data.touch_count == 0 && data.click_edges == 0);
    assert(!data.reset_called && !data.last_touched_state && !data.touch_verify_pending);
    assert(data.rdy_recheck_attempts == 0);
    assert(data.sensor_suspended && data.sensor_resume_attempts == 1);
    assert(!atomic_get(&data.suspended) && data.init_state == IQS7211E_INIT_VERIFY_PRODUCT);
    data.sensor_resume_attempts = UINT8_MAX;
    write_fail_after = 0;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(data.sensor_resume_attempts == UINT8_MAX);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(!data.sensor_suspended && data.sensor_resume_attempts == 0);
    assert(!(REG(IQS7211E_MM_SYS_CONTROL)[1] & BIT(IQS7211E_SUSPEND_BIT)));
    assert(REG(IQS7211E_MM_SYS_CONTROL)[0] == SYSTEM_CONTROL_0 &&
           REG(IQS7211E_MM_SYS_CONTROL)[1] ==
               (uint8_t)(SYSTEM_CONTROL_1 & ~BIT(IQS7211E_SUSPEND_BIT)));
    /* A GPIO controller not back yet: the resume still succeeds and the
     * recheck keeps trying to arm RDY. */
    irq_fail_after = 0;
    data.rdy_recheck_work.scheduled = false;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(data.rdy_recheck_work.scheduled);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_TURN_OFF) == -ENOTSUP);
    assert(iqs7211e_retry_sensor_resume(&data));
}

static void test_early_pm(void) {
    /* The early device suspends every ready, awake sensor for soft-off. */
    assert(iqs7211e_early_pm_init(NULL) == 0);
    assert(iqs7211e_early_pm_action(NULL, PM_DEVICE_ACTION_RESUME) == 0);
    assert(iqs7211e_early_pm_action(NULL, PM_DEVICE_ACTION_TURN_OFF) == -ENOTSUP);
    sensor.ready = false;
    assert(iqs7211e_early_pm_action(NULL, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(!atomic_get(&data.suspended));
    sensor.ready = true;
    data.init_state = IQS7211E_INIT_DONE;
    ready_checks = 0;
    assert(iqs7211e_early_pm_action(NULL, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(ready_checks >= 2 && ready_checked[0] == &idle_sensor && ready_checked[1] == &sensor);
    assert(atomic_get(&data.suspended) && data.sensor_suspended);
    /* Its own turn then finds it suspended and leaves it. */
    assert(iqs7211e_early_pm_action(NULL, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
}

/* PM transitions across a retained Event Mode, an interrupted setup, and
 * the edge that an unsuccessful initial interrupt mask can otherwise lose. */
static void fresh_pm_fixture(void) {
    current_thread = NULL;
    memset(&data, 0, sizeof(data));
    memset(regs, 0, sizeof(regs));
    read_fail_after = write_fail_after = irq_fail_after = input_fail_after = -1;
    irq_configure_result = 0;
    submit_result = reschedule_result = 0;
    input_fail_all = false;
    on_arm = on_mask = on_irq_failure = NULL;
    rdy_level = rdy_result = rdy_raw_result = 0;
    input_count = 0;
    assert(iqs7211e_init(&sensor) == 0);
    data.init_state = IQS7211E_INIT_DONE;
    data.boot_kick_work.scheduled = false;
    data.rdy_recheck_work.scheduled = false;
}

static void test_failed_initial_mask_recovers_missed_edge(void) {
    fresh_pm_fixture();
    rdy_level = 1;
    data.touch_verify_pending = true;
    irq_fail_after = 0;
    on_irq_failure = rdy_edge;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EIO);
    assert(!atomic_get(&data.suspended));
    assert(data.touch_verify_pending && "failed first mask must retain pending report ownership");
    assert(data.rdy_recheck_work.scheduled && "failed suspend must recover a missed RDY edge");
    run(&data.rdy_recheck_work);
    assert(data.work.pending && !atomic_get(&data.irq_enabled));
}

static void test_resume_retains_completed_setup(void) {
    fresh_pm_fixture();
    REG(IQS7211E_MM_CONFIG_SETTINGS)[1] = BIT(IQS7211E_EVENT_MODE_BIT);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(data.init_state == IQS7211E_INIT_DONE && "resume must retain completed sensor setup");
    assert(!data.sensor_suspended && !data.boot_kick_work.scheduled);
    assert(REG(IQS7211E_MM_CONFIG_SETTINGS)[1] & BIT(IQS7211E_EVENT_MODE_BIT));
    int reads = i2c_reads;
    run_work();
    assert(data.init_state == IQS7211E_INIT_DONE && i2c_reads == reads + 1);
    /* An actual sensor reset during sleep still takes the full reset path. */
    REG(IQS7211E_MM_GESTURES)[2] = IQS7211E_RUNTIME_SHOW_RESET_MASK;
    run_work();
    assert(data.init_state == IQS7211E_INIT_UPDATE_SETTINGS && input_count == 0);

    /* A temporary bus failure must not turn a completed setup into cold init. */
    fresh_pm_fixture();
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0);
    write_fail_after = 0;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(data.sensor_suspended && data.init_state == IQS7211E_INIT_DONE);
    assert(data.rdy_recheck_work.scheduled && !data.boot_kick_work.scheduled);
    run(&data.rdy_recheck_work);
    assert(!data.sensor_suspended && data.init_state == IQS7211E_INIT_DONE);
    assert(atomic_get(&data.irq_enabled));
}

static void test_resume_rearms_interrupted_setup(void) {
    fresh_pm_fixture();
    data.init_state = IQS7211E_INIT_READ_DATA;
    data.reset_called = true;
    data.boot_kick_attempts = IQS7211E_BOOT_KICK_ATTEMPTS;
    atomic_set(&data.boot_irq_seen, 1);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0);
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    assert(data.init_state == IQS7211E_INIT_VERIFY_PRODUCT && !data.reset_called);
    assert(data.boot_kick_work.scheduled && "interrupted setup needs its silent-sensor recovery");
    assert(data.boot_kick_work.delay_ms == IQS7211E_BOOT_KICK_MS);
    assert(data.boot_kick_attempts == 0 && atomic_get(&data.boot_irq_seen) == 0);
}

static void test_pm_rejects_own_queue_before_any_side_effect(void) {
    fresh_pm_fixture();
    touch_and_click();
    const int old_irq_mode = irq_mode;
    current_thread = &iqs7211e_work_queue.thread;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EWOULDBLOCK &&
           "PM must not synchronously cancel its currently executing queue");
    assert(!data.suspended && data.last_touched_state && data.click_edges == 3);
    assert(irq_mode == old_irq_mode && input_count == 0);
    current_thread = input;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EWOULDBLOCK &&
           "PM must not wait for a release while blocking its input consumer");
    assert(!data.suspended && data.last_touched_state && data.click_edges == 3);
    assert(irq_mode == old_irq_mode && input_count == 0);
    current_thread = NULL;
}

static void test_runtime_rearm_failure_keeps_recovery_alive(void) {
    fresh_pm_fixture();
    assert(set_gpio_interrupt(&sensor, false) == 0);
    irq_fail_after = 0;
    run_work();
    assert(!atomic_get(&data.irq_enabled));
    assert(data.rdy_recheck_work.scheduled && "runtime IRQ arm failure must retain recovery");
    run(&data.rdy_recheck_work);
    assert(atomic_get(&data.irq_enabled));

    fresh_pm_fixture();
    data.touch_release_pending = true;
    submit_result = -EBUSY;
    irq_fail_after = 2;
    assert(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == -EBUSY);
    assert(!atomic_get(&data.suspended) && !atomic_get(&data.irq_enabled));
    assert(data.rdy_recheck_work.scheduled && "rejected suspend must retry a failed IRQ restore");
    submit_result = 0;
    run(&data.rdy_recheck_work);
    assert(atomic_get(&data.irq_enabled));
}

static bool run_pm_regression(const char *name) {
    if (strcmp(name, "failed-mask") == 0) test_failed_initial_mask_recovers_missed_edge();
    else if (strcmp(name, "retained-resume") == 0) test_resume_retains_completed_setup();
    else if (strcmp(name, "interrupted-resume") == 0) test_resume_rearms_interrupted_setup();
    else if (strcmp(name, "own-queue-pm") == 0) test_pm_rejects_own_queue_before_any_side_effect();
    else if (strcmp(name, "runtime-rearm") == 0) test_runtime_rearm_failure_keeps_recovery_alive();
    else return false;
    return true;
}

int main(void) {
    sensor = (struct device){.name = "trackpad", .config = &config, .data = &data, .ready = true};
    const char *selected = getenv("IQS_TEST_CASE");
    if (selected != NULL) {
        assert(run_pm_regression(selected) && "unknown IQS_TEST_CASE");
        puts("iqs7211e named PM regression: PASS");
        return 0;
    }
    test_init();
    test_setup();
    test_default_settings();
    test_readers();
    test_interrupt();
    test_work();
    test_rdy_recheck();
    test_touch_verify();
    test_clicks();
    test_pm_release();
    test_pm();
    test_early_pm();
    test_failed_initial_mask_recovers_missed_edge();
    test_resume_retains_completed_setup();
    test_resume_rearms_interrupted_setup();
    test_pm_rejects_own_queue_before_any_side_effect();
    test_runtime_rearm_failure_keeps_recovery_alive();
    puts("iqs7211e driver: PASS");
    return 0;
}
