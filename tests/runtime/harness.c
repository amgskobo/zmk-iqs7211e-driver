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
    bool report_abs;
    int rotate_cw, jitter_deadband;
    int single_tap, double_tap, triple_tap;
    uint16_t touch_verify_interval_ms;
};
struct iqs7211e_data {
    const struct device *dev;
    struct k_work_delayable touch_verify_work, click_work;
    atomic_t suspended, rdy_recheck_attempts;
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
#define IQS7211E_INIT_UPDATE_SETTINGS 4
#define IQS7211E_CLICK_EDGE_MS 20
#define K_FOREVER 0
#define K_MSEC(ms) (ms)
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_DBG(...) ((void)0)
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
static int atomic_get(atomic_t *v) { return *v; }
static void atomic_clear(atomic_t *v) { *v = 0; }
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w) {
    return CONTAINER_OF(w, struct k_work_delayable, work);
}
static void k_work_cancel_delayable(struct k_work_delayable *w) { (void)w; }
static int scheduled, release_attempts, presses, fail_key, fail_abs;
static int iqs7211e_reschedule_work(struct k_work_delayable *w, int ms) {
    (void)w; (void)ms; scheduled++; return 0;
}
static void iqs7211e_note_work_queue_stack_usage(void) {}
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
    puts("iqs7211e runtime fault-injection tests passed");
}
