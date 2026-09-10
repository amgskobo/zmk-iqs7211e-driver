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
    int scroll_slider_layer, rotate_cw, jitter_deadband, scroll_start;
    int single_tap, double_tap, triple_tap;
    uint16_t touch_verify_interval_ms;
    const uint8_t *scroll_layers;
    uint8_t scroll_layer_count;
    const uint8_t *scroll_slider_trigger_layers;
    uint8_t scroll_slider_trigger_layer_count;
};
struct iqs7211e_data {
    const struct device *dev;
    struct k_work_delayable touch_verify_work, click_work;
    atomic_t suspended, contact_tap_state, rdy_recheck_attempts;
    bool last_touched_state, touch_release_pending, is_scroll_slider_layer_active;
    bool scroll_slider_layer_activated_by_driver, suppress_delayed_scroll_tap;
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
static int atomic_set(atomic_t *v, int n) { int old = *v; *v = n; return old; }
static void atomic_clear(atomic_t *v) { *v = 0; }
static void atomic_or(atomic_t *v, int n) { *v |= n; }
static bool atomic_cas(atomic_t *v, int old, int next) {
    if (*v != old) return false;
    *v = next; return true;
}
typedef int zmk_event_t;
static struct device *callback_device;
#define DEVICE_DT_INST_GET(inst) callback_device
#define DT_INST_FOREACH_STATUS_OKAY(fn) fn(0)
#define ARG_UNUSED(x) ((void)(x))
#define ZMK_EV_EVENT_BUBBLE 0
static struct k_work_delayable *k_work_delayable_from_work(struct k_work *w) {
    return CONTAINER_OF(w, struct k_work_delayable, work);
}
static void k_work_cancel_delayable(struct k_work_delayable *w) { (void)w; }
static int scheduled, deactivated, release_attempts, presses, fail_key, fail_abs;
static int iqs7211e_reschedule_work(struct k_work_delayable *w, int ms) {
    (void)w; (void)ms; scheduled++; return 0;
}
static void iqs7211e_note_work_queue_stack_usage(void) {}
static int zmk_keymap_layer_deactivate(int layer, bool unused) {
    (void)layer; (void)unused; deactivated++; return 0;
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
static uint32_t layers;
static int fingers, gesture, queued_clicks;
typedef uint8_t zmk_keymap_layer_index_t;
typedef uint8_t zmk_keymap_layer_id_t;
static zmk_keymap_layer_index_t highest_layer_index;
static zmk_keymap_layer_id_t studio_layer_order[8];
static zmk_keymap_layer_index_t zmk_keymap_highest_layer_active(void) {
    return highest_layer_index;
}
static zmk_keymap_layer_id_t
zmk_keymap_layer_index_to_id(zmk_keymap_layer_index_t index) {
    return studio_layer_order[index];
}
static bool zmk_keymap_layer_active(int layer) { return (layers & (1u << layer)) != 0; }
static int zmk_keymap_layer_activate(int layer, bool unused) {
    (void)unused; layers |= 1u << layer; return 0;
}
static int iqs7211e_queue_value_updates(struct iqs7211e_data *d) { (void)d; return 0; }
static int iqs7211e_get_num_fingers(struct iqs7211e_data *d) { (void)d; return fingers; }
static int iqs7211e_get_touchpad_event(struct iqs7211e_data *d) { (void)d; return gesture; }
static bool iqs7211e_is_tap_gesture(int g) { return g >= 1 && g <= 3; }
static bool iqs7211e_scroll_slider_trigger_layer_allowed(const struct iqs7211e_config *c) {
    (void)c; return layers == 0;
}
static void iqs7211e_queue_clicks(struct iqs7211e_data *d, uint16_t button, uint8_t clicks) {
    (void)d; (void)button; queued_clicks += clicks;
}

/* DRIVER_FUNCTIONS */

int main(void) {
    /* ZMK Studio reorders layer indexes, but DTS continues to name layer IDs. */
    const uint8_t studio_trigger_layers[] = {4};
    struct iqs7211e_config studio_config = {
        .scroll_slider_trigger_layers = studio_trigger_layers,
        .scroll_slider_trigger_layer_count = 1,
    };
    studio_layer_order[0] = 4; /* Layer ID 4 was moved to the top/index 0. */
    highest_layer_index = 0;
    assert(iqs7211e_layer_allowed(studio_config.scroll_slider_trigger_layers,
                                  studio_config.scroll_slider_trigger_layer_count));
    studio_layer_order[0] = 1;
    assert(!iqs7211e_layer_allowed(studio_config.scroll_slider_trigger_layers,
                                   studio_config.scroll_slider_trigger_layer_count));

    const struct iqs7211e_config config = {.report_abs = true, .scroll_slider_layer = 6};
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
    d.is_scroll_slider_layer_active = d.scroll_slider_layer_activated_by_driver = true;
    d.contact_tap_state = 3;
    fail_key = 2;
    assert(iqs7211e_release_touch(&d) == -5);
    assert(!d.last_touched_state && d.touch_release_pending);
    assert(deactivated == 1 && d.suppress_delayed_scroll_tap);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(d.touch_release_pending);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(!d.touch_release_pending && deactivated == 1);
    int attempts = release_attempts;
    assert(iqs7211e_release_touch(&d) == 0);
    assert(release_attempts == attempts);

    d.last_touched_state = true;
    d.contact_tap_state = 3;
    fail_abs = 1;
    assert(iqs7211e_release_touch(&d) == -5);
    assert(d.touch_release_pending && d.suppress_delayed_scroll_tap);
    iqs7211e_click_work_handler(&d.click_work.work);
    assert(!d.touch_release_pending && d.contact_tap_state == 0);
    assert(iqs7211e_begin_runtime_reinitialization(&d) == 0);
    assert(!d.suppress_delayed_scroll_tap);
    /* Both manual layers and implicit slider layers use the same history. */
    const uint8_t manual_layers[] = {2, 3, 5, 6, 1};
    for (size_t i = 0; i < sizeof(manual_layers); i++) {
        struct iqs7211e_config c = {.report_abs = true, .scroll_slider_layer = -1,
            .scroll_layers = &manual_layers[i], .scroll_layer_count = 1};
        if (i >= 3) {
            c.scroll_slider_layer = manual_layers[i];
            c.scroll_layers = NULL;
            c.scroll_layer_count = 0;
        }
        struct device board = {.config = &c};
        d = (struct iqs7211e_data){.dev = &board, .finger_1_x = 500, .finger_1_y = 500,
                                  .finger_1_touch_strength = 100, .finger_1_area = 2};
        queued_clicks = 0;
        layers = 1u << manual_layers[i]; fingers = 1; gesture = 0;
        assert(iqs7211e_report_data(&d) == 0);
        layers = 0; fingers = 0;
        assert(iqs7211e_report_data(&d) == 0);
        assert(d.suppress_delayed_scroll_tap);
        gesture = IQS7211E_GESTURE_DOUBLE_TAP;
        assert(iqs7211e_report_data(&d) == 0 && queued_clicks == 0);
        assert(!d.suppress_delayed_scroll_tap);
        fingers = 1; gesture = 0;
        assert(iqs7211e_report_data(&d) == 0);
        fingers = 0; gesture = IQS7211E_GESTURE_SINGLE_TAP;
        assert(iqs7211e_report_data(&d) == 0 && queued_clicks == 1);
        /* Failed normal release must be retried before a new contact. */
        fingers = 1; gesture = 0;
        assert(iqs7211e_report_data(&d) == 0);
        fingers = 0; fail_key = 1;
        assert(iqs7211e_report_data(&d) == -5 && d.touch_release_pending);
        fingers = 1;
        assert(iqs7211e_report_data(&d) == 0);
        assert(!d.touch_release_pending && d.last_touched_state);
        /* Layer enabled and disabled between reports must still be remembered. */
        board.data = &d;
        callback_device = &board;
        layers = 1u << manual_layers[i];
        iqs7211e_tap_layer_listener(NULL);
        layers = 0;
        iqs7211e_tap_layer_listener(NULL);
        fingers = 0; gesture = 0;
        assert(iqs7211e_report_data(&d) == 0 && d.suppress_delayed_scroll_tap);
        gesture = IQS7211E_GESTURE_TRIPLE_TAP;
        assert(iqs7211e_report_data(&d) == 0 && queued_clicks == 1);
        /* A new normal contact cancels unused delayed suppression. */
        d.suppress_delayed_scroll_tap = true;
        fingers = 1; gesture = 0;
        assert(iqs7211e_report_data(&d) == 0 && !d.suppress_delayed_scroll_tap);
        fingers = 0; gesture = IQS7211E_GESTURE_DOUBLE_TAP;
        assert(iqs7211e_report_data(&d) == 0 && queued_clicks == 3);
        if (i >= 3) {
            /* Automatic edge activation must enter the same gate even when
             * the layer stub does not dispatch a synchronous callback. */
            c.scroll_start = 50;
            d.finger_1_x = 1020;
            d.finger_1_y = 500;
            layers = 0; fingers = 1; gesture = IQS7211E_GESTURE_SINGLE_TAP;
            assert(iqs7211e_report_data(&d) == 0);
            assert(d.scroll_slider_layer_activated_by_driver);
            assert((d.contact_tap_state & 2) && queued_clicks == 3);
            layers = 0; /* History, not current activation or ownership, gates taps. */
            fingers = 0;
            assert(iqs7211e_report_data(&d) == 0 && queued_clicks == 3);
            assert(!d.scroll_slider_layer_activated_by_driver);
        }
    }
    puts("iqs7211e runtime fault-injection tests passed");
}
