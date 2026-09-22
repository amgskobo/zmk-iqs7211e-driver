/*
 * Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "iqs7211e_runtime.h"
#include "iqs7211e_filter.h"

typedef int atomic_t;
typedef int atomic_val_t;
struct k_work { int unused; };
struct k_work_delayable { struct k_work work; };
struct device { const void *config; void *data; };
struct iqs7211e_config {
    int i2c;
    bool report_abs;
    int rotate_cw, jitter_deadband;
    int single_tap, double_tap, triple_tap;
    uint16_t touch_verify_interval_ms;
};
struct iqs7211e_data {
    const struct device *dev;
    struct k_work_delayable touch_verify_work, click_work, boot_kick_work, rdy_recheck_work;
    atomic_t suspended, rdy_recheck_attempts, diagnostic_irq_count;
    uint8_t boot_kick_attempts;
    bool last_touched_state, touch_release_pending;
    bool reset_called, touch_verify_pending;
    int finger_1_prev_x, finger_1_prev_y, finger_1_prev_dx, finger_1_prev_dy;
    int touch_count, click_edges, click_button, init_state;
    uint8_t info_flags[2];
    uint16_t finger_1_x, finger_1_y, finger_1_touch_strength, finger_1_area;
    struct iqs7211e_axis_filter finger_1_filter_x, finger_1_filter_y;
};
#define RESOLUTION_X 1024
#define RESOLUTION_Y 1024
#define INPUT_REL_X 0
#define INPUT_REL_Y 1
#define INPUT_BTN_0 256
#define IQS7211E_GESTURE_NONE 0
#define IQS7211E_GESTURE_SINGLE_TAP 1
#define IQS7211E_GESTURE_DOUBLE_TAP 2
#define IQS7211E_GESTURE_TRIPLE_TAP 3
#define INPUT_ABS_X 0
#define INPUT_ABS_Y 1
#define INPUT_BTN_TOUCH 330
#define IQS7211E_INIT_VERIFY_PRODUCT 1
#define IQS7211E_INIT_UPDATE_SETTINGS 4
#define IQS7211E_MM_SYS_CONTROL 0x33
#define IQS7211E_SW_RESET_BIT 1
#define SYSTEM_CONTROL_0 0x00
#define SYSTEM_CONTROL_1 0x00
#define IQS7211E_BOOT_KICK_RETRY_MS 200
#define IQS7211E_BOOT_KICK_ATTEMPTS 3
#define IQS7211E_BOOT_KICK_SETTLE_MS 50
#define IQS7211E_CLICK_EDGE_MS 20
#define K_FOREVER 0
#define K_MSEC(ms) (ms)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define LOG_INF(...) ((void)0)
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
static int atomic_get(atomic_t *v) { return *v; }
static void atomic_clear(atomic_t *v) { *v = 0; }
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w) {
    return CONTAINER_OF(w, struct k_work_delayable, work);
}
static void k_work_cancel_delayable(struct k_work_delayable *w) { (void)w; }
static int scheduled, release_attempts, presses, fail_key, fail_abs;
static struct k_work_delayable *last_scheduled;
static int last_delay;
static int iqs7211e_reschedule_work(struct k_work_delayable *w, int ms) {
    last_scheduled = w; last_delay = ms; scheduled++; return 0;
}
/* The RDY interrupt: whether it is armed, and how often it was re-armed. */
static bool irq_armed = true;
static int rearms;
static int set_gpio_interrupt(const struct device *dev, bool en) {
    (void)dev; irq_armed = en; return 0;
}
static int iqs7211e_enable_interrupt_and_recheck(void *data) {
    (void)data; irq_armed = true; rearms++; return 0;
}
static void iqs7211e_note_work_queue_stack_usage(void) {}
/* The one write the boot kick makes, and a way to make it fail. */
static int writes, fail_write, last_reg;
static uint8_t last_write[2];
static int iqs7211e_write_bytes(const int *i2c, uint8_t reg, const uint8_t *buf, size_t len) {
    (void)i2c;
    assert(len == 2);
    writes++;
    last_reg = reg;
    last_write[0] = buf[0];
    last_write[1] = buf[1];
    if (fail_write) { fail_write--; return -5; }
    return 0;
}
static int input_report_key(const struct device *dev, int code, bool value, bool sync, int wait) {
    (void)dev; (void)code; (void)sync; (void)wait;
    if (value) presses++; else release_attempts++;
    if (fail_key) { fail_key--; return -5; }
    return 0;
}
static int input_report_abs(const struct device *dev, int code, int value, bool sync, int wait) {
    (void)dev; (void)code; (void)value; (void)sync; (void)wait;
    if (fail_abs) { fail_abs--; return -5; }
    return 0;
}
static int input_report_rel(const struct device *dev, int code, int value, bool sync, int wait) {
    (void)dev; (void)code; (void)value; (void)sync; (void)wait; return 0;
}
static int fingers, gesture, queued_clicks;
static int iqs7211e_queue_value_updates(struct iqs7211e_data *d) { (void)d; return 0; }
static int iqs7211e_get_num_fingers(struct iqs7211e_data *d) { (void)d; return fingers; }
static int iqs7211e_get_touchpad_event(struct iqs7211e_data *d) { (void)d; return gesture; }
static void iqs7211e_queue_clicks(struct iqs7211e_data *d, uint16_t button, uint8_t clicks) {
    (void)d; (void)button; queued_clicks += clicks;
}

/* DRIVER_FUNCTIONS */

/*
 * A sensor that kept its power through the SoC's reset raises no RDY: the boot
 * kick resets it once, retries a failed write a bounded number of times, and
 * leaves alone a part that has spoken or a driver that has moved on.
 */
static void test_boot_kick(const struct device *dev) {
    struct iqs7211e_data d = {.dev = dev, .init_state = IQS7211E_INIT_VERIFY_PRODUCT};

    /*
     * Silent since boot: one forced write, reset bit set, suspend bit clear,
     * with RDY masked across it and left to the recheck work to re-arm once
     * the reset has had time to start.
     */
    writes = 0; scheduled = 0; irq_armed = true; rearms = 0;
    iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
    assert(writes == 1 && scheduled == 1);
    assert(last_reg == IQS7211E_MM_SYS_CONTROL);
    assert(last_write[0] == 0x00 && last_write[1] == (1 << IQS7211E_SW_RESET_BIT));
    assert(!irq_armed && rearms == 0);
    assert(last_scheduled == &d.rdy_recheck_work && last_delay == IQS7211E_BOOT_KICK_SETTLE_MS);

    /* An RDY edge since init: the part is talking, nothing is written. */
    d = (struct iqs7211e_data){.dev = dev, .init_state = IQS7211E_INIT_VERIFY_PRODUCT,
                               .diagnostic_irq_count = 1};
    writes = 0;
    iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
    assert(writes == 0);

    /* Setup has moved on, or the driver is suspended: nothing either. */
    d = (struct iqs7211e_data){.dev = dev, .init_state = IQS7211E_INIT_UPDATE_SETTINGS};
    iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
    d = (struct iqs7211e_data){.dev = dev, .init_state = IQS7211E_INIT_VERIFY_PRODUCT,
                               .suspended = 1};
    iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
    assert(writes == 0);

    /*
     * A failed write is retried, and given up on after the last attempt; RDY
     * is re-armed after every failure, so a part that wakes by itself is
     * still heard.
     */
    d = (struct iqs7211e_data){.dev = dev, .init_state = IQS7211E_INIT_VERIFY_PRODUCT};
    writes = 0; scheduled = 0; rearms = 0; fail_write = IQS7211E_BOOT_KICK_ATTEMPTS;
    for (int i = 0; i < IQS7211E_BOOT_KICK_ATTEMPTS; i++) {
        iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
        assert(irq_armed);
    }
    assert(writes == IQS7211E_BOOT_KICK_ATTEMPTS);
    assert(scheduled == IQS7211E_BOOT_KICK_ATTEMPTS - 1);
    assert(last_scheduled == &d.boot_kick_work);
    assert(rearms == IQS7211E_BOOT_KICK_ATTEMPTS);

    /* A write that succeeds on a retry is not repeated. */
    d = (struct iqs7211e_data){.dev = dev, .init_state = IQS7211E_INIT_VERIFY_PRODUCT};
    writes = 0; scheduled = 0; fail_write = 1;
    iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
    iqs7211e_boot_kick_work_handler(&d.boot_kick_work.work);
    assert(writes == 2 && scheduled == 2 && fail_write == 0);
    assert(last_scheduled == &d.rdy_recheck_work && !irq_armed);
}

int main(void) {
    const struct iqs7211e_config config = {
        .report_abs = true, .single_tap = 0, .double_tap = 0, .triple_tap = 0,
    };
    const struct device dev = {.config = &config};
    struct iqs7211e_data d = {.dev = &dev, .click_edges = 5};
    fail_key = 1;
    assert(iqs7211e_begin_runtime_reinitialization(&d) == -5);
    assert(d.click_edges == 1 && scheduled > 0);
    assert(d.init_state == IQS7211E_INIT_UPDATE_SETTINGS);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(d.click_edges == 0 && presses == 0 && release_attempts == 2);
    d.click_edges = 4;
    assert(iqs7211e_begin_runtime_reinitialization(&d) == 0);
    assert(d.click_edges == 0 && presses == 0);

    d.last_touched_state = true;
    fail_key = 2;
    assert(iqs7211e_release_touch(&d) == -5);
    assert(!d.last_touched_state && d.touch_release_pending);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(d.touch_release_pending);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(!d.touch_release_pending);
    int attempts = release_attempts;
    assert(iqs7211e_release_touch(&d) == 0);
    assert(release_attempts == attempts);

    d.last_touched_state = true;
    fail_abs = 1;
    assert(iqs7211e_release_touch(&d) == -5);
    assert(d.touch_release_pending);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(!d.touch_release_pending);
    assert(iqs7211e_begin_runtime_reinitialization(&d) == 0);

    d = (struct iqs7211e_data){.dev = &dev, .finger_1_x = 500, .finger_1_y = 500,
                              .finger_1_touch_strength = 100, .finger_1_area = 2};
    queued_clicks = 0;
    fingers = 1; gesture = 0;
    assert(iqs7211e_report_data(&d) == 0);
    fingers = 0; gesture = IQS7211E_GESTURE_SINGLE_TAP;
    assert(iqs7211e_report_data(&d) == 0 && queued_clicks == 1);

    fingers = 1; gesture = 0;
    assert(iqs7211e_report_data(&d) == 0);
    fingers = 0; fail_key = 1;
    assert(iqs7211e_report_data(&d) == -5 && d.touch_release_pending);
    fingers = 1; gesture = 0;
    assert(iqs7211e_report_data(&d) == 0);
    assert(!d.touch_release_pending && d.last_touched_state);

    test_boot_kick(&dev);
    puts("iqs7211e runtime fault-injection tests passed");
}
