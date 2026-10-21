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
    atomic_t suspended, rdy_recheck_attempts, boot_irq_seen;
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
static struct k_work_delayable *cancelled[16];
static int cancels;
static void k_work_cancel_delayable(struct k_work_delayable *w) { cancelled[cancels++ % 16] = w; }
static bool was_cancelled(int since, const struct k_work_delayable *w) {
    for (int i = since; i < cancels; i++) {
        if (cancelled[i % 16] == w) {
            return true;
        }
    }
    return false;
}
static int scheduled, release_attempts, presses, fail_key, fail_abs, fail_rel, fail_queue;
static int fail_schedule;
static struct k_work_delayable *last_scheduled;
static int last_delay;
static int iqs7211e_reschedule_work(struct k_work_delayable *w, int ms) {
    last_scheduled = w; last_delay = ms; scheduled++;
    if (fail_schedule) { fail_schedule--; return -5; }
    return 0;
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
struct report { char type; int code, value; bool sync; };
static struct report reports[16];
static int report_count;
static void note(char type, int code, int value, bool sync) {
    if (report_count < (int)(sizeof(reports) / sizeof(reports[0]))) {
        reports[report_count] = (struct report){type, code, value, sync};
    }
    report_count++;
}
/* What a failed ABS report returns, so two failures can be told apart. */
static int abs_error = -5;
static int input_report_key(const struct device *dev, int code, bool value, bool sync, int wait) {
    (void)dev; (void)wait;
    note('K', code, value, sync);
    if (value) presses++; else release_attempts++;
    if (fail_key) { fail_key--; return -5; }
    return 0;
}
static int input_report_abs(const struct device *dev, int code, int value, bool sync, int wait) {
    (void)dev; (void)wait;
    note('A', code, value, sync);
    if (fail_abs) { fail_abs--; return abs_error; }
    return 0;
}
static int input_report_rel(const struct device *dev, int code, int value, bool sync, int wait) {
    (void)dev; (void)wait;
    note('R', code, value, sync);
    if (fail_rel) { fail_rel--; return -5; }
    return 0;
}
static int fingers, gesture, queued_clicks, queued_button = -1;
static int iqs7211e_queue_value_updates(struct iqs7211e_data *d) {
    (void)d;
    if (fail_queue) { fail_queue--; return -5; }
    return 0;
}
static int iqs7211e_get_num_fingers(struct iqs7211e_data *d) { (void)d; return fingers; }
static int iqs7211e_get_touchpad_event(struct iqs7211e_data *d) { (void)d; return gesture; }
static void iqs7211e_queue_clicks(struct iqs7211e_data *d, uint16_t button, uint8_t clicks) {
    (void)d; queued_button = button; queued_clicks += clicks;
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
                               .boot_irq_seen = 1};
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

static void test_report_modes(struct iqs7211e_config *config, const struct device *dev) {
    struct iqs7211e_data d = {.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                             .finger_1_touch_strength = 100, .finger_1_area = 2};
    fingers = 1; gesture = IQS7211E_GESTURE_NONE;
    fail_queue = 1;
    assert(iqs7211e_report_data(&d) == -5 && !d.last_touched_state);
    d.info_flags[0] = IQS7211E_RUNTIME_SHOW_RESET_MASK;
    assert(iqs7211e_report_data(&d) == 0);
    d.info_flags[0] = 0;

    d = (struct iqs7211e_data){.dev = dev, .last_touched_state = true};
    fail_key = 1;
    assert(iqs7211e_begin_runtime_reinitialization(&d) == -5);
    d = (struct iqs7211e_data){.dev = dev, .last_touched_state = true,
                              .click_edges = 1};
    fail_key = 2;
    assert(iqs7211e_begin_runtime_reinitialization(&d) == -5);
    d = (struct iqs7211e_data){.dev = dev, .last_touched_state = true,
                              .suspended = 1};
    fail_key = 1;
    assert(iqs7211e_release_touch(&d) == -5 && d.touch_release_pending);
    config->touch_verify_interval_ms = 20;
    d = (struct iqs7211e_data){.dev = dev};
    assert(!iqs7211e_touch_verify_chain_alive(&d));
    config->touch_verify_interval_ms = 0;
    d = (struct iqs7211e_data){.dev = dev, .touch_release_pending = true};
    fail_key = 1;
    assert(iqs7211e_report_data(&d) == -5);

    for (int rotation = 1; rotation <= 3; rotation++) {
        d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                                  .finger_1_touch_strength = 100, .finger_1_area = 2};
        config->rotate_cw = rotation;
        assert(iqs7211e_report_data(&d) == 0);
        d.finger_1_x = 510;
        d.finger_1_y = 610;
        assert(iqs7211e_report_data(&d) == 0);
        d.finger_1_touch_strength = 0;
        assert(iqs7211e_report_data(&d) == 0 && d.last_touched_state);
    }
    config->rotate_cw = 0;

    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_area = 2};
    assert(iqs7211e_report_data(&d) == 0 && !d.last_touched_state);
    d.finger_1_touch_strength = 100;
    gesture = IQS7211E_GESTURE_SINGLE_TAP;
    assert(iqs7211e_report_data(&d) == 0);
    gesture = IQS7211E_GESTURE_DOUBLE_TAP;
    assert(iqs7211e_report_data(&d) == 0);
    gesture = IQS7211E_GESTURE_TRIPLE_TAP;
    assert(iqs7211e_report_data(&d) == 0);

    config->single_tap = -1;
    config->double_tap = -1;
    config->triple_tap = -1;
    for (int event = IQS7211E_GESTURE_SINGLE_TAP;
         event <= IQS7211E_GESTURE_TRIPLE_TAP; event++) {
        fingers = 1;
        gesture = event;
        assert(iqs7211e_report_data(&d) == 0);
        fingers = 0;
        assert(iqs7211e_report_data(&d) == 0);
    }
    config->single_tap = 0;
    config->double_tap = 0;
    config->triple_tap = 0;
    fingers = 0;
    gesture = IQS7211E_GESTURE_DOUBLE_TAP;
    assert(iqs7211e_report_data(&d) == 0);
    gesture = IQS7211E_GESTURE_TRIPLE_TAP;
    assert(iqs7211e_report_data(&d) == 0);

    fingers = 1; gesture = IQS7211E_GESTURE_NONE;
    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_touch_strength = 100, .finger_1_area = 2};
    fail_key = 1;
    assert(iqs7211e_report_data(&d) == -5);
    config->touch_verify_interval_ms = 20;
    assert(iqs7211e_report_data(&d) == 0);
    assert(last_scheduled == &d.touch_verify_work);
    config->touch_verify_interval_ms = 0;
    fingers = 0;
    fail_abs = 1;
    assert(iqs7211e_report_data(&d) == -5 && d.touch_release_pending);
    fingers = 1;
    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_touch_strength = 100, .finger_1_area = 2};
    fail_abs = 1;
    assert(iqs7211e_report_data(&d) == -5);
    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_touch_strength = 100, .finger_1_area = 2,
                              .touch_count = 255, .last_touched_state = true};
    assert(iqs7211e_report_data(&d) == 0 && d.touch_count == 255);

    config->report_abs = false;
    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_touch_strength = 100, .finger_1_area = 2};
    assert(iqs7211e_report_data(&d) == 0);
    d.finger_1_x = 520;
    assert(iqs7211e_report_data(&d) == 0);
    d.finger_1_x = 530;
    fail_rel = 1;
    assert(iqs7211e_report_data(&d) == -5);
    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_touch_strength = 100, .finger_1_area = 2};
    assert(iqs7211e_report_data(&d) == 0);
    fingers = 0;
    assert(iqs7211e_report_data(&d) == 0);
    fingers = 1;
    d = (struct iqs7211e_data){.dev = dev, .finger_1_x = 500, .finger_1_y = 600,
                              .finger_1_touch_strength = 100, .finger_1_area = 2};
    fail_rel = 1;
    assert(iqs7211e_report_data(&d) == -5);
    assert(iqs7211e_report_rel_coordinates(&d, 1, 2) == 0);
    fail_rel = 1;
    assert(iqs7211e_report_rel_coordinates(&d, 1, 2) == -5);
    config->report_abs = true;
}

/* The reports since the last call, exactly: type, code, value and sync. */
static void expect_reports(int count, const struct report *want) {
    if (report_count != count) {
        fprintf(stderr, "want %d reports, got %d\n", count, report_count);
        assert(false);
    }
    for (int i = 0; i < count; i++) {
        const struct report *got = &reports[i];
        if (got->type != want[i].type || got->code != want[i].code ||
            got->value != want[i].value || got->sync != want[i].sync) {
            fprintf(stderr, "report %d: want %c %d %d %d, got %c %d %d %d\n", i, want[i].type,
                    want[i].code, want[i].value, want[i].sync, got->type, got->code,
                    got->value, got->sync);
            assert(false);
        }
    }
    report_count = 0;
}
#define EXPECT(...)                                                                        \
    do {                                                                                   \
        const struct report want[] = {__VA_ARGS__};                                        \
        expect_reports((int)(sizeof(want) / sizeof(want[0])), want);                       \
    } while (0)
#define EXPECT_NONE() expect_reports(0, NULL)

static struct iqs7211e_data touching(const struct device *dev, uint16_t x, uint16_t y) {
    return (struct iqs7211e_data){.dev = dev, .finger_1_x = x, .finger_1_y = y,
                                  .finger_1_touch_strength = 100, .finger_1_area = 2};
}

/* What goes out, report by report, for a contact in each mode. */
static __attribute__((unused)) void test_report_streams(struct iqs7211e_config *config,
                                                       const struct device *dev) {
    config->report_abs = true;
    config->rotate_cw = 0;
    config->jitter_deadband = 0;
    config->touch_verify_interval_ms = 0;
    config->single_tap = config->double_tap = config->triple_tap = 0;
    fingers = 1;
    gesture = IQS7211E_GESTURE_NONE;
    report_count = 0;

    /* Absolute: the touch edge, then the pair, synced on Y. The median
     * filter trails by a frame. */
    struct iqs7211e_data d = touching(dev, 500, 600);
    const int before = scheduled;
    assert(iqs7211e_report_data(&d) == 0 && d.touch_count == 1 && d.last_touched_state);
    EXPECT({'K', INPUT_BTN_TOUCH, 1, false}, {'A', INPUT_ABS_X, 500, false},
           {'A', INPUT_ABS_Y, 600, true});
    assert(scheduled == before); /* no touch verification configured */
    d.finger_1_x = 510;
    d.finger_1_y = 610;
    assert(iqs7211e_report_data(&d) == 0 && d.touch_count == 2);
    EXPECT({'A', INPUT_ABS_X, 500, false}, {'A', INPUT_ABS_Y, 600, true});
    d.finger_1_x = 520;
    d.finger_1_y = 620;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'A', INPUT_ABS_X, 510, false}, {'A', INPUT_ABS_Y, 610, true});
    /* The lift: the release edge, then the last pair to flush it; the
     * verification timer goes with the contact. */
    fingers = 0;
    int since = cancels;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'K', INPUT_BTN_TOUCH, 0, false}, {'A', INPUT_ABS_X, 510, false},
           {'A', INPUT_ABS_Y, 610, true});
    assert(!d.last_touched_state && !d.touch_release_pending && d.touch_count == 0);
    assert(was_cancelled(since, &d.touch_verify_work));
    /* A show-reset is read from the first flag byte only. */
    d.info_flags[1] = IQS7211E_RUNTIME_SHOW_RESET_MASK;
    d.init_state = 0;
    assert(iqs7211e_report_data(&d) == 0 && d.init_state == 0);
    d.info_flags[1] = 0;
    assert(scheduled == before);
    /* Nothing touching and nothing to release: nothing goes out. */
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT_NONE();

    /* Each rotation turns the first sample. */
    static const int turned[4][2] = {{500, 600}, {424, 500}, {524, 424}, {600, 524}};
    for (int rotation = 1; rotation <= 3; rotation++) {
        config->rotate_cw = rotation;
        fingers = 1;
        d = touching(dev, 500, 600);
        assert(iqs7211e_report_data(&d) == 0);
        EXPECT({'K', INPUT_BTN_TOUCH, 1, false}, {'A', INPUT_ABS_X, turned[rotation][0], false},
               {'A', INPUT_ABS_Y, turned[rotation][1], true});
    }
    config->rotate_cw = 0;

    /* A tap on button 0 is a tap, while touching and at the lift. */
    for (int event = IQS7211E_GESTURE_SINGLE_TAP; event <= IQS7211E_GESTURE_TRIPLE_TAP;
         event++) {
        for (int touch = 1; touch >= 0; touch--) {
            fingers = touch;
            gesture = event;
            queued_clicks = 0;
            queued_button = -1;
            d = touching(dev, 500, 600);
            assert(iqs7211e_report_data(&d) == 0);
            assert(queued_clicks == event && queued_button == INPUT_BTN_0);
            config->single_tap = 1;
            config->double_tap = 2;
            config->triple_tap = 3;
            queued_button = -1;
            d = touching(dev, 500, 600);
            assert(iqs7211e_report_data(&d) == 0);
            assert(queued_button == INPUT_BTN_0 + event);
            config->single_tap = config->double_tap = config->triple_tap = 0;
            report_count = 0;
        }
    }
    gesture = IQS7211E_GESTURE_NONE;

    /* Relative: a synced zero first, then the averaged deltas of the
     * filtered position. */
    config->report_abs = false;
    fingers = 1;
    d = touching(dev, 500, 600);
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, true});
    d.finger_1_x = 520;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 0, true});
    d.finger_1_x = 540;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 10, false}, {'R', INPUT_REL_Y, 0, true});
    d.finger_1_x = 560;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 20, false}, {'R', INPUT_REL_Y, 0, true});
    /* No touch key in relative mode, not even at the lift, and the velocity
     * history goes with the contact. */
    fingers = 0;
    assert(iqs7211e_report_data(&d) == 0 && !d.last_touched_state);
    EXPECT_NONE();
    assert(d.finger_1_prev_dx == 0 && d.finger_1_prev_dy == 0);
    /* The same along Y. */
    fingers = 1;
    d = touching(dev, 500, 600);
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, true});
    d.finger_1_y = 620;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 0, true});
    d.finger_1_y = 640;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 10, true});
    d.finger_1_y = 660;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 20, true});
    assert(d.finger_1_prev_dy == 20);
    fingers = 0;
    assert(iqs7211e_report_data(&d) == 0);
    assert(d.finger_1_prev_dy == 0);
    config->report_abs = true;

    /* The release path owns the last coordinates, and reschedules only a
     * release that failed. */
    d = touching(dev, 0, 0);
    d.last_touched_state = true;
    d.finger_1_prev_x = 11;
    d.finger_1_prev_y = 22;
    d.touch_count = 5;
    d.finger_1_prev_dx = 3;
    d.finger_1_prev_dy = 4;
    int count = scheduled;
    since = cancels;
    assert(iqs7211e_release_touch(&d) == 0 && !d.touch_release_pending);
    assert(d.touch_count == 0 && d.finger_1_prev_dx == 0 && d.finger_1_prev_dy == 0);
    assert(was_cancelled(since, &d.touch_verify_work));
    EXPECT({'K', INPUT_BTN_TOUCH, 0, false}, {'A', INPUT_ABS_X, 11, false},
           {'A', INPUT_ABS_Y, 22, true});
    assert(scheduled == count);
    d.last_touched_state = true;
    fail_key = 1;
    assert(iqs7211e_release_touch(&d) == -5 && d.touch_release_pending);
    EXPECT({'K', INPUT_BTN_TOUCH, 0, false});
    assert(scheduled == count + 1 && last_scheduled == &d.click_work &&
           last_delay == IQS7211E_CLICK_EDGE_MS);

    /* A reset releases an owed click, synced, and clears the setup flags; the
     * first failure is the one reported. */
    d = touching(dev, 0, 0);
    d.click_edges = 3;
    d.click_button = INPUT_BTN_0 + 1;
    d.reset_called = true;
    d.touch_verify_pending = true;
    d.rdy_recheck_attempts = 3;
    count = scheduled;
    since = cancels;
    assert(iqs7211e_begin_runtime_reinitialization(&d) == 0);
    assert(was_cancelled(since, &d.click_work) && d.rdy_recheck_attempts == 0);
    EXPECT({'K', INPUT_BTN_0 + 1, 0, true});
    assert(d.click_edges == 0 && !d.reset_called && !d.touch_verify_pending);
    assert(scheduled == count);
    d.click_edges = 1;
    d.last_touched_state = true;
    fail_key = 1;
    fail_abs = 1;
    abs_error = -7;
    assert(iqs7211e_begin_runtime_reinitialization(&d) == -5);
    abs_error = -5;
    report_count = 0;

    /* The click worker: each edge synced, and nothing rescheduled after the
     * last one. */
    d = touching(dev, 0, 0);
    d.click_button = INPUT_BTN_0 + 2;
    d.click_edges = 2;
    count = scheduled;
    iqs7211e_click_work_handler(&d.click_work.work);
    EXPECT({'K', INPUT_BTN_0 + 2, 1, true});
    assert(scheduled == count + 1);
    iqs7211e_click_work_handler(&d.click_work.work);
    EXPECT({'K', INPUT_BTN_0 + 2, 0, true});
    assert(scheduled == count + 1 && d.click_edges == 0);
    /* A failed press that is rescheduled leaves the click to the retry. */
    d.click_edges = 2;
    fail_key = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
    EXPECT({'K', INPUT_BTN_0 + 2, 1, true});
    assert(d.click_edges == 2);
    report_count = 0;
    /* ...but if the retry cannot be scheduled either, nothing is owed. */
    fail_key = 1;
    fail_schedule = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(d.click_edges == 0);
    report_count = 0;
    /* A press whose next edge cannot be scheduled is released at once. */
    d.click_edges = 2;
    fail_schedule = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
    EXPECT({'K', INPUT_BTN_0 + 2, 1, true}, {'K', INPUT_BTN_0 + 2, 0, true});
    assert(d.click_edges == 0);

    /* One millisecond is a verification interval. */
    config->touch_verify_interval_ms = 1;
    d.last_touched_state = true;
    assert(iqs7211e_touch_verify_chain_alive(&d));
    config->touch_verify_interval_ms = 0;
}

static void test_click_worker_failures(const struct device *dev) {
    struct iqs7211e_data d = {.dev = dev, .click_edges = 2};
    d.suspended = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(d.click_edges == 2);
    d.suspended = 0;
    fail_key = 1;
    fail_schedule = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
    d.click_edges = 2;
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(d.click_edges == 1);
    fail_key = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
    d.click_edges = 2;
    fail_schedule = 1;
    iqs7211e_click_work_handler(&d.click_work.work);
}

#if defined(CONFIG_IQS7211E_FILTER_BYPASS)
static void test_unfiltered_report_streams(struct iqs7211e_config *config,
                                          const struct device *dev) {
    config->report_abs = false;
    config->rotate_cw = 0;
    config->jitter_deadband = 4;
    config->touch_verify_interval_ms = 0;
    config->single_tap = config->double_tap = config->triple_tap = 0;
    fingers = 1;
    gesture = IQS7211E_GESTURE_NONE;
    report_count = 0;
    struct iqs7211e_data d = touching(dev, 500, 600);
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, true});
    /* One-unit motion passes immediately, including reversal and a stop. */
    d.finger_1_x = 501;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 1, false}, {'R', INPUT_REL_Y, 0, true});
    d.finger_1_x = 500;
    d.finger_1_y = 602;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, -1, false}, {'R', INPUT_REL_Y, 2, true});
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 0, true});
    /* Invalid coordinates hold the last valid point; no spike or drift. */
    d.finger_1_x = UINT16_MAX;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 0, true});
    d.finger_1_x = 503;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 3, false}, {'R', INPUT_REL_Y, 0, true});
    /* Other devices/contacts retain independent position and velocity. */
    struct iqs7211e_data other = touching(dev, 100, 200);
    assert(iqs7211e_report_data(&other) == 0);
    EXPECT({'R', INPUT_REL_X, 0, true});
    other.finger_1_y = 199;
    assert(iqs7211e_report_data(&other) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, -1, true});
    assert(d.finger_1_prev_x == 503 && d.finger_1_prev_y == 602);
    fingers = 0;
    assert(iqs7211e_report_data(&d) == 0 && d.touch_count == 0);
    EXPECT_NONE();
    fingers = 1;
    d.finger_1_x = 800;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, true});
    /* Invalid initial contact and low-strength frames cannot invent motion. */
    struct iqs7211e_data invalid = touching(dev, 1025, 500);
    assert(iqs7211e_report_data(&invalid) == 0 && invalid.touch_count == 0);
    EXPECT_NONE();
    d.finger_1_touch_strength = 0;
    d.finger_1_x = 0;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'R', INPUT_REL_X, 0, false}, {'R', INPUT_REL_Y, 0, true});
    /* Absolute mode keeps rotation and balanced contact edges. */
    config->report_abs = true;
    config->rotate_cw = 1;
    d = touching(dev, 500, 600);
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'K', INPUT_BTN_TOUCH, 1, false}, {'A', INPUT_ABS_X, 424, false},
           {'A', INPUT_ABS_Y, 500, true});
    d.finger_1_x = 501;
    d.finger_1_y = 602;
    assert(iqs7211e_report_data(&d) == 0);
    EXPECT({'A', INPUT_ABS_X, 422, false}, {'A', INPUT_ABS_Y, 501, true});
    fingers = 0;
    assert(iqs7211e_report_data(&d) == 0 && !d.last_touched_state);
    EXPECT({'K', INPUT_BTN_TOUCH, 0, false}, {'A', INPUT_ABS_X, 422, false},
           {'A', INPUT_ABS_Y, 501, true});
    config->rotate_cw = 0;
}
#endif

int main(void) {
    struct iqs7211e_config config = {
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
    test_report_modes(&config, &dev);
    test_click_worker_failures(&dev);
#if defined(CONFIG_IQS7211E_FILTER_BYPASS)
    test_unfiltered_report_streams(&config, &dev);
#else
    test_report_streams(&config, &dev);
#endif
    d = (struct iqs7211e_data){.dev = &dev};
    assert(iqs7211e_report_abs_coordinates(&d, 1, 2) == 0);
    assert(iqs7211e_report_rel_coordinates(&d, 1, 2) == 0);
    fail_abs = 1;
    assert(iqs7211e_report_abs_coordinates(&d, 1, 2) < 0);
    puts("iqs7211e runtime fault-injection tests passed");
}
