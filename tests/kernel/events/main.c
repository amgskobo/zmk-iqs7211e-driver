/* Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */
/*
 * The complete driver on the real kernel: DT instances initialized by the
 * kernel, the real I2C emulator controller, the emulated RDY GPIO with edges
 * raised from timer ISRs, the private work queue, delayable work, the input
 * queue and thread, PM device actions, and the real ZMK input listener up to
 * the HID report. Only the sensor is a model (sensor_model.h). Each expected
 * trace below is derived from the driver's contract, not captured from a run.
 */
#include <stdlib.h>
#include <string.h>
#include "../../../src/iqs7211e.c"
#include <zephyr/drivers/emul.h>
#include <zephyr/drivers/i2c_emul.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include <zephyr/pm/device.h>
#include <zmk/hid.h>
static void fail(int line, const char *text);
#include "sensor_model.h"

#define SENSOR_LIST(X)                                                                             \
    X(S_POINTER, iqs_pointer, 1)                                                                           \
    X(S_MAP, iqs_map, 2)                                                                           \
    X(S_OFF, iqs_off, 3)                                                                           \
    X(S_REL, iqs_rel, 4)                                                                           \
    X(S_ROT1, iqs_rot1, 5)                                                                         \
    X(S_ROT2, iqs_rot2, 6)                                                                         \
    X(S_ROT3, iqs_rot3, 7)                                                                         \
    X(S_NOV, iqs_nov, 8)

#define SENSOR_ENUM(id, label, pin) id,
enum { SENSOR_LIST(SENSOR_ENUM) SENSOR_COUNT };

#define MODEL_INIT(id, label, rdy_pin)                                                             \
    [id] = {.index = id,                                                                           \
            .dev = DEVICE_DT_GET(DT_NODELABEL(label)),                                             \
            .port = DEVICE_DT_GET(DT_NODELABEL(gpio0)),                                            \
            .pin = rdy_pin},
static struct model models[SENSOR_COUNT] = {SENSOR_LIST(MODEL_INIT)};

#define MODEL_EMUL(id, label, pin)                                                                 \
    EMUL_DT_DEFINE(DT_NODELABEL(label), model_emul_init, &models[id], NULL, &model_i2c_api, NULL);
SENSOR_LIST(MODEL_EMUL)

/* RDY is idle before the driver's first look at it. */
static int model_rdy_idle_init(void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        model_prepare(&models[i]);
    }
    return 0;
}
SYS_INIT(model_rdy_idle_init, POST_KERNEL, 80);

static const char *current_case = "none";

/* ----- What came out ----- */

struct seen_event {
    int64_t ticks;
    int32_t value;
    uint16_t code;
    uint8_t type;
    uint8_t sensor;
    bool sync;
};
static struct seen_event seen[512];
static atomic_t seen_count;
extern struct k_msgq input_msgq; /* Real Zephyr input queue, observed without replacing it. */
K_SEM_DEFINE(input_gate, 0, 1);
static atomic_t input_gate_waiting;
static atomic_t input_gate_closed;

static void dump_all(void);
static void fail(int line, const char *text) {
    dump_all();
    printk("IQSFAIL case=%s line=%d: %s\n", current_case, line, text);
    exit(1);
}
#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            fail(__LINE__, #cond);                                                                 \
        }                                                                                          \
    } while (0)

static void record_event(int sensor, struct input_event *event) {
    if (atomic_cas(&input_gate_closed, 1, 0)) {
        atomic_set(&input_gate_waiting, 1);
        k_sem_take(&input_gate, K_FOREVER);
        atomic_clear(&input_gate_waiting);
    }
    /* Reports reach consumers on the input thread, never on the driver's queue. */
    CHECK(k_current_get() == input);
    const atomic_val_t index = atomic_get(&seen_count);
    CHECK(index < (atomic_val_t)ARRAY_SIZE(seen));
    atomic_inc(&seen_count);
    seen[index] = (struct seen_event){.ticks = k_uptime_ticks(), .value = event->value,
                                      .code = event->code, .type = event->type,
                                      .sensor = (uint8_t)sensor, .sync = event->sync};
}

#define SENSOR_CALLBACK(id, label, pin)                                                            \
    static void record_##label(struct input_event *event, void *user_data) {                       \
        ARG_UNUSED(user_data);                                                                     \
        record_event(id, event);                                                                   \
    }                                                                                              \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(label)), record_##label, NULL);
SENSOR_LIST(SENSOR_CALLBACK)

struct hid_report {
    int64_t ticks;
    int16_t x;
    int16_t y;
    uint8_t buttons;
};
static struct hid_report hid_seen[128];
static int hid_count;

static int record_hid_report(void) {
    const struct zmk_hid_mouse_report *report = zmk_hid_get_mouse_report();
    CHECK(hid_count < (int)ARRAY_SIZE(hid_seen));
    hid_seen[hid_count++] = (struct hid_report){.ticks = k_uptime_ticks(), .x = report->body.d_x, .y = report->body.d_y,
                                                .buttons = report->body.buttons};
    return 0;
}
int __wrap_zmk_endpoints_send_mouse_report(void) { return record_hid_report(); }
int __wrap_zmk_endpoint_send_mouse_report(void) { return record_hid_report(); }

static void dump_all(void) {
    const int count = (int)atomic_get(&seen_count);
    for (int i = 0; i < count; i++) {
        const struct seen_event *e = &seen[i];
        printk("IQSTRACE i=%d s=%u type=%u code=%u value=%d sync=%d ticks=%lld ms=%lld\n", i, e->sensor, e->type,
               e->code, e->value, e->sync, e->ticks, k_ticks_to_ms_near64(e->ticks));
    }
    for (int i = 0; i < hid_count; i++) {
        printk("IQSHID i=%d buttons=%u x=%d y=%d ticks=%lld ms=%lld\n", i,
               hid_seen[i].buttons, hid_seen[i].x, hid_seen[i].y, hid_seen[i].ticks,
               k_ticks_to_ms_near64(hid_seen[i].ticks));
    }
    for (int s = 0; s < SENSOR_COUNT; s++) {
        const struct model *m = &models[s];
        for (int i = 0; i < m->bus_count; i++) {
            const struct model_bus_op *op = &m->bus[i];
            printk("IQSBUS s=%d %s reg=0x%02x len=%u b0=0x%02x b1=0x%02x nack=%d ticks=%lld ms=%lld\n", s,
                   op->write ? "write" : "read", op->reg, op->len, op->b0, op->b1, op->nack,
                   op->ticks, k_ticks_to_ms_near64(op->ticks));
        }
    }
}

struct want {
    int32_t value;
    uint16_t code;
    uint8_t type;
    bool sync;
};
#define W_TOUCH(v) {(v), INPUT_BTN_TOUCH, INPUT_EV_KEY, false}
#define W_X(v) {(v), INPUT_ABS_X, INPUT_EV_ABS, false}
#define W_Y(v) {(v), INPUT_ABS_Y, INPUT_EV_ABS, true}
#define W_RX(v) {(v), INPUT_REL_X, INPUT_EV_REL, false}
#define W_RY(v) {(v), INPUT_REL_Y, INPUT_EV_REL, true}
#define W_RX_FIRST {0, INPUT_REL_X, INPUT_EV_REL, true}
#define W_BTN(n, v) {(v), (uint16_t)(INPUT_BTN_0 + (n)), INPUT_EV_KEY, true}
#define W_CLICK(n) W_BTN(n, 1), W_BTN(n, 0)

/* Exactly these reports from this sensor, in this order, type, code, value and sync. */
static void expect_stream(int sensor, const struct want *want, int len, int line) {
    const int count = (int)atomic_get(&seen_count);
    int matched = 0;
    for (int i = 0; i < count; i++) {
        const struct seen_event *e = &seen[i];
        if (e->sensor != sensor) {
            continue;
        }
        if (matched >= len || e->type != want[matched].type || e->code != want[matched].code ||
            e->value != want[matched].value || e->sync != want[matched].sync) {
            fail(line, "report differs from the expected trace");
        }
        matched++;
    }
    if (matched != len) {
        fail(line, "wrong number of reports");
    }
}
#define EXPECT(sensor, ...)                                                                        \
    do {                                                                                           \
        static const struct want expected[] = {__VA_ARGS__};                                       \
        expect_stream(sensor, expected, (int)ARRAY_SIZE(expected), __LINE__);                      \
    } while (0)
#define EXPECT_NONE(sensor) expect_stream(sensor, NULL, 0, __LINE__)

static int stream_count(int sensor) {
    const int count = (int)atomic_get(&seen_count);
    int matched = 0;
    for (int i = 0; i < count; i++) {
        matched += seen[i].sensor == sensor;
    }
    return matched;
}

static void expect_hid(const struct hid_report *want, int len, int line) {
    if (hid_count != len) {
        fail(line, "wrong number of HID reports");
    }
    for (int i = 0; i < len; i++) {
        if (hid_seen[i].buttons != want[i].buttons || hid_seen[i].x != want[i].x ||
            hid_seen[i].y != want[i].y) {
            fail(line, "HID report differs from the expected trace");
        }
    }
}
#define EXPECT_HID(...)                                                                            \
    do {                                                                                           \
        static const struct hid_report expected[] = {__VA_ARGS__};                                 \
        expect_hid(expected, (int)ARRAY_SIZE(expected), __LINE__);                                 \
    } while (0)
#define HID_BUTTONS(mask) {.buttons = (mask)}

/* Consecutive edges of this button are at least one click-edge delay apart. */
static void check_click_spacing(int sensor, int button) {
    const int64_t minimum = (int64_t)k_ms_to_ticks_ceil64(20);
    const int count = (int)atomic_get(&seen_count);
    int64_t previous = -1;
    for (int i = 0; i < count; i++) {
        const struct seen_event *e = &seen[i];
        if (e->sensor != sensor || e->type != INPUT_EV_KEY || e->code != INPUT_BTN_0 + button) {
            continue;
        }
        if (previous >= 0) {
            CHECK(e->ticks - previous >= minimum);
        }
        previous = e->ticks;
    }
}

/* ----- Driving the model ----- */

static struct iqs7211e_data *driver_data(const struct model *m) { return m->dev->data; }

#define WAIT_UNTIL(cond, limit_ms)                                                                 \
    do {                                                                                           \
        int waited = 0;                                                                            \
        while (!(cond)) {                                                                          \
            if (waited++ >= (limit_ms)) {                                                          \
                fail(__LINE__, "timed out waiting for " #cond);                                    \
            }                                                                                      \
            k_sleep(K_MSEC(1));                                                                    \
        }                                                                                          \
    } while (0)

static void wait_ready(struct model *m) {
    WAIT_UNTIL(driver_data(m)->init_state == IQS7211E_INIT_DONE && m->event_mode, 3000);
    k_sleep(K_MSEC(2 * MODEL_CYCLE_MS));
}

static void bring_up(struct model *m) {
    model_power_on(m, MODEL_FRESH);
    wait_ready(m);
    m->bus_count = 0;
}

static void signal_frame(struct model *m) {
    const unsigned before = m->report_reads;
    k_timer_start(&m->event_timer, K_TICKS(1), K_NO_WAIT);
    WAIT_UNTIL(m->report_reads > before, 1000);
}

static void frame(struct model *m, uint8_t g0, uint8_t g1, uint8_t fingers, uint16_t x, uint16_t y) {
    model_load_frame(m, &(struct model_frame){.g0 = g0, .fingers = fingers, .x = x, .y = y});
    m->mem[MODEL_REG_GESTURES][1] = g1;
    m->frame_event |= g1 != 0 && (m->mem[MODEL_REG_CONFIG][1] & MODEL_GESTURE_EVENT);
    if (m->frame_event || !model_event_mode(m)) {
        signal_frame(m);
    } else if (fingers) {
        /* An unchanged contact is polled by the real 120-ms verification chain. */
        const unsigned before = m->report_reads;
        WAIT_UNTIL(m->report_reads > before, 500);
    }
}

static void touch(struct model *m, uint16_t x, uint16_t y) { frame(m, 0, 0, 1, x, y); }
static void lift(struct model *m) { frame(m, 0, 0, 0, 0, 0); }
static void tap(struct model *m, uint8_t gesture) { frame(m, gesture, 0, 0, 0, 0); }

#define TAP_SINGLE 0x01
#define TAP_DOUBLE 0x02
#define TAP_TRIPLE 0x04
#define GESTURE_PALM 0x10
#define CLICKS_DONE_MS 150

static void wait_reports(int sensor, int reports) { WAIT_UNTIL(stream_count(sensor) >= reports, 500); }

static void check_quiet(const struct model *m) {
    const struct iqs7211e_data *data = driver_data(m);
    CHECK(data->click_edges == 0);
    CHECK(!k_work_delayable_is_pending(&((struct iqs7211e_data *)data)->click_work));
    CHECK(!data->touch_release_pending);
    CHECK(!data->last_touched_state);
    CHECK(!data->touch_verify_pending);
    CHECK(!k_work_delayable_is_pending(&((struct iqs7211e_data *)data)->touch_verify_work));
}

/* Each button and contact ends released, whatever the scenario did. */
static void check_balanced(void) {
    const int count = (int)atomic_get(&seen_count);
    for (int s = 0; s < SENSOR_COUNT; s++) {
        for (int code = INPUT_BTN_0; code <= INPUT_BTN_TOUCH; code++) {
            int level = 0;
            for (int i = 0; i < count; i++) {
                const struct seen_event *e = &seen[i];
                if (e->sensor == s && e->type == INPUT_EV_KEY && e->code == code) {
                    CHECK(e->value == !level);
                    level = e->value;
                }
            }
            CHECK(level == 0);
        }
    }
}

static void finish(void) {
    int bus_before[SENSOR_COUNT];
    for (int s = 0; s < SENSOR_COUNT; s++) {
        bus_before[s] = models[s].bus_count;
    }
    int before = (int)atomic_get(&seen_count);
    k_sleep(K_MSEC(300));
    /* Nothing keeps coming once a scenario is over. */
    CHECK((int)atomic_get(&seen_count) == before);
    check_balanced();
    for (int s = 0; s < SENSOR_COUNT; s++) {
        if (models[s].present) {
            check_quiet(&models[s]);
            CHECK(models[s].bus_count == bus_before[s]);
        } else {
            CHECK(models[s].bus_count <= 3);
            for (int i = 0; i < models[s].bus_count; i++) {
                const struct model_bus_op *op = &models[s].bus[i];
                CHECK(op->write && op->nack && op->reg == MODEL_REG_SYS_CONTROL &&
                      op->len == 2 && op->b0 == 0 && op->b1 == MODEL_SW_RESET);
            }
        }
    }
    dump_all();
    printk("IQS kernel EVENTS PASS case=%s\n", current_case);
    exit(0);
}

static int count_ops(const struct model *m, uint8_t reg, bool write) {
    int matched = 0;
    for (int i = 0; i < m->bus_count; i++) {
        matched += m->bus[i].reg == reg && m->bus[i].write == write;
    }
    return matched;
}

/* The nth (0-based) transaction of this kind. */
static const struct model_bus_op *nth_op(const struct model *m, uint8_t reg, bool write, int n) {
    for (int i = 0; i < m->bus_count; i++) {
        if (m->bus[i].reg == reg && m->bus[i].write == write && n-- == 0) {
            return &m->bus[i];
        }
    }
    fail(__LINE__, "no such bus transaction");
    return NULL;
}

static int count_writes(const struct model *m) {
    int matched = 0;
    for (int i = 0; i < m->bus_count; i++) {
        matched += m->bus[i].write;
    }
    return matched;
}

static void pm_suspend(const struct model *m) { CHECK(pm_device_action_run(m->dev, PM_DEVICE_ACTION_SUSPEND) == 0); }
static void pm_resume(const struct model *m) { CHECK(pm_device_action_run(m->dev, PM_DEVICE_ACTION_RESUME) == 0); }

/* ----- Setup and boot recovery ----- */

/* Every settings block once and in order, then ACK, ATI and event mode. */
static void check_setup_sequence(const struct model *m, int first_op) {
    static const struct {
        uint8_t reg;
        uint8_t len;
    } order[] = {{0x1F, 4},  {0x21, 14}, {0x28, 22}, {0x33, 6},  {0x36, 4},  {0x38, 6},
                 {0x3B, 4},  {0x3D, 8},  {0x41, 18}, {0x4A, 2},  {0x4B, 22}, {0x56, 14},
                 {0x5D, 30}, {0x6C, 30}, {0x7B, 3},  {0x33, 2},  {0x33, 2},  {0x34, 2}};
    int next = 0;
    for (int i = first_op; i < m->bus_count; i++) {
        const struct model_bus_op *op = &m->bus[i];
        if (!op->write) {
            continue;
        }
        CHECK(next < (int)ARRAY_SIZE(order));
        CHECK(op->reg == order[next].reg && op->len == order[next].len);
        if (next == 15) {
            CHECK((op->b0 & MODEL_ACK_RESET) != 0);
        } else if (next == 16) {
            CHECK((op->b0 & MODEL_TP_RE_ATI) != 0);
        } else if (next == 17) {
            CHECK((op->b1 & MODEL_EVENT_MODE) != 0);
        }
        next++;
    }
    CHECK(next == (int)ARRAY_SIZE(order));
}

static void check_programmed(const struct model *m) {
    CHECK(m->event_mode);
    CHECK((m->mem[MODEL_REG_CONFIG][1] & 0x06) == 0x06);
    CHECK((m->mem[MODEL_REG_CONFIG][0] & 0x40) == 0);
    CHECK(m->ati_observed > 0);
    CHECK((m->mem[MODEL_REG_INFO][0] & MODEL_SHOW_RESET) == 0);
    /* The tap gestures this driver reports are enabled in the part itself. */
    CHECK(m->mem[0x4B][0] == 0x07);
    /* Coordinates are programmed to the resolution the driver rotates against. */
    CHECK(m->mem[0x43][0] == 0x00 && m->mem[0x43][1] == 0x04);
    CHECK(m->mem[0x44][0] == 0x00 && m->mem[0x44][1] == 0x04);
}

static void case_boot_fresh(void) {
    struct model *m = &models[S_POINTER];
    model_power_on(m, MODEL_FRESH);
    wait_ready(m);
    /* Well past the silent-part deadline: heard from, so never reset. */
    k_sleep(K_MSEC(400));
    check_setup_sequence(m, 0);
    check_programmed(m);
    CHECK(m->resets_seen == 0);
    CHECK(count_ops(m, MODEL_REG_PRODUCT, false) == 1);
    CHECK(atomic_get(&driver_data(m)->boot_irq_seen) != 0);
    CHECK(atomic_get(&driver_data(m)->irq_enabled) != 0);
    EXPECT_NONE(S_POINTER);
    finish();
}

/* A part that kept its power says nothing until the driver forces a reset. */
static void boot_kick(enum model_power power) {
    struct model *m = &models[S_POINTER];
    model_power_on(m, power);
    WAIT_UNTIL(m->resets_seen == 1, 300);
    const struct model_bus_op *kick = nth_op(m, MODEL_REG_SYS_CONTROL, true, 0);
    CHECK(k_ticks_to_ms_near64(kick->ticks) >= 200);
    CHECK(k_ticks_to_ms_near64(kick->ticks) <= 205);
    CHECK(kick->b0 == 0x00 && kick->b1 == MODEL_SW_RESET);
    /* The forced write is the first thing that happens on the bus. */
    CHECK(m->bus_count >= 1 && m->bus[0].write);
    wait_ready(m);
    check_setup_sequence(m, 1);
    check_programmed(m);
    CHECK(m->resets_seen == 1);
    EXPECT_NONE(S_POINTER);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}
static void case_boot_silent_event_mode(void) { boot_kick(MODEL_RETAINED_EVENT); }
static void case_boot_silent_suspended(void) { boot_kick(MODEL_RETAINED_SUSPENDED); }

/* No part at all: three bounded attempts, then quiet. */
static void case_boot_absent(void) {
    struct model *m = &models[S_POINTER];
    k_sleep(K_MSEC(1500));
    /* Not present in the model's log either, so read it directly. */
    CHECK(m->bus_count == 3);
    for (int i = 0; i < 3; i++) {
        CHECK(m->bus[i].write && m->bus[i].nack && m->bus[i].reg == MODEL_REG_SYS_CONTROL);
        CHECK(m->bus[i].b1 == MODEL_SW_RESET);
    }
    const int64_t retry = (int64_t)k_ms_to_ticks_ceil64(200);
    CHECK(m->bus[1].ticks - m->bus[0].ticks >= retry);
    CHECK(m->bus[2].ticks - m->bus[1].ticks >= retry);
    CHECK(driver_data(m)->init_state == IQS7211E_INIT_VERIFY_PRODUCT);
    CHECK(atomic_get(&driver_data(m)->irq_enabled) != 0);
    EXPECT_NONE(S_POINTER);
    printk("IQSBUS absent attempts=%d\n", m->bus_count);
    finish();
}

/* Another part answers: one read, then the bus is left alone. */
static void case_boot_wrong_product(void) {
    struct model *m = &models[S_POINTER];
    model_power_on(m, MODEL_FRESH);
    m->mem[MODEL_REG_PRODUCT][0] = 0x34;
    m->mem[MODEL_REG_PRODUCT][1] = 0x12;
    k_sleep(K_MSEC(500));
    CHECK(driver_data(m)->init_state == IQS7211E_INIT_NONE);
    CHECK(m->bus_count == 1 && !m->bus[0].write && m->bus[0].reg == MODEL_REG_PRODUCT);
    EXPECT_NONE(S_POINTER);
    finish();
}

/* Bus errors retry on the next RDY; a failed block restarts the settings. */
static void case_boot_bus_errors(void) {
    struct model *m = &models[S_POINTER];
    m->nack_next = 2;
    m->nack_write_reg = 0x3D;
    model_power_on(m, MODEL_FRESH);
    wait_ready(m);
    CHECK(m->bus[0].nack && m->bus[1].nack && !m->bus[2].nack);
    CHECK(m->bus[0].reg == MODEL_REG_PRODUCT && m->bus[2].reg == MODEL_REG_PRODUCT);
    CHECK(count_ops(m, 0x1F, true) == 2);
    check_programmed(m);
    EXPECT_NONE(S_POINTER);
    finish();
}

/* A reset that is never acted on is issued again rather than waited for. */
static void case_boot_reset_lost(void) {
    struct model *m = &models[S_POINTER];
    m->ignore_resets = 1;
    model_power_on(m, MODEL_STREAMING_NO_RESET);
    wait_ready(m);
    CHECK(m->resets_seen == 2);
    check_programmed(m);
    EXPECT_NONE(S_POINTER);
    finish();
}

/* ----- Contacts ----- */

static void case_contact_abs(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    touch(m, 500, 600);
    /* Deadband 8, then a three-point median: 700 reads as 500, 692, 692. */
    touch(m, 700, 600);
    touch(m, 700, 600);
    touch(m, 700, 600);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(692),
           W_Y(600), W_X(692), W_Y(600), W_TOUCH(0), W_X(692), W_Y(600));
    CHECK(!driver_data(m)->last_touched_state);
    finish();
}

static void case_contact_rel(void) {
    struct model *m = &models[S_REL];
    bring_up(m);
    touch(m, 500, 600);
    /* 600 reads as 500, 592, 592, 592: deltas 0, 92, 0, 0, averaged with the last. */
    touch(m, 600, 600);
    touch(m, 600, 600);
    touch(m, 600, 600);
    touch(m, 600, 600);
    lift(m);
    /* No contact edge in relative mode: that would be a left-button drag. */
    EXPECT(S_REL, W_RX_FIRST, W_RX(0), W_RY(0), W_RX(46), W_RY(0), W_RX(46), W_RY(0), W_RX(0), W_RY(0));
    /* One HID report per sync, including the first zero; no button for contact. */
    EXPECT_HID({.x = 0}, {.x = 0}, {.x = 46}, {.x = 46}, {.x = 0});
    finish();
}

static void case_contact_invalid(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    /* No strength, and out of range: neither starts a contact. */
    model_load_frame(m, &(struct model_frame){.fingers = 1, .x = 100, .y = 100});
    m->mem[MODEL_REG_STRENGTH][0] = 0;
    signal_frame(m);
    touch(m, 2000, 100);
    touch(m, 100, 2000);
    EXPECT_NONE(S_POINTER);
    touch(m, 500, 600);
    /* Once started, an invalid sample holds the last coordinate instead of lifting. */
    model_load_frame(m, &(struct model_frame){.fingers = 1, .x = 900, .y = 900});
    m->mem[MODEL_REG_AREA][0] = 0;
    signal_frame(m);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}

static void case_contact_second(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    lift(m);
    /* A new contact reseeds the filter: no history from the last one. */
    touch(m, 100, 200);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600), W_TOUCH(1), W_X(100),
           W_Y(200), W_TOUCH(0), W_X(100), W_Y(200));
    finish();
}

static void case_rotate_90(void) {
    struct model *m = &models[S_ROT1];
    bring_up(m);
    touch(m, 500, 600);
    lift(m);
    EXPECT(S_ROT1, W_TOUCH(1), W_X(424), W_Y(500), W_TOUCH(0), W_X(424), W_Y(500));
    finish();
}

static void case_rotate_180(void) {
    struct model *m = &models[S_ROT2];
    bring_up(m);
    touch(m, 500, 600);
    lift(m);
    EXPECT(S_ROT2, W_TOUCH(1), W_X(524), W_Y(424), W_TOUCH(0), W_X(524), W_Y(424));
    finish();
}

static void case_rotate_270(void) {
    struct model *m = &models[S_ROT3];
    bring_up(m);
    touch(m, 500, 600);
    lift(m);
    EXPECT(S_ROT3, W_TOUCH(1), W_X(600), W_Y(524), W_TOUCH(0), W_X(600), W_Y(524));
    finish();
}

/* ----- Touch verification ----- */

static void case_verify_holds_contact(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    k_sleep(K_MSEC(400));
    /* One read each 120 ms with no RDY at all: the frame, then three checks. */
    CHECK(count_ops(m, MODEL_REG_GESTURES, false) == 4);
    const int64_t interval = (int64_t)k_ms_to_ticks_ceil64(120);
    for (int i = 1; i < 4; i++) {
        CHECK(nth_op(m, MODEL_REG_GESTURES, false, i)->ticks -
                  nth_op(m, MODEL_REG_GESTURES, false, i - 1)->ticks >= interval);
    }
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600));
    lift(m);
    const unsigned reads = m->report_reads;
    k_sleep(K_MSEC(300));
    /* The chain ends with the contact. */
    CHECK(m->report_reads == reads);
    finish();
}

static void case_verify_finds_lift(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    /* The finger leaves and no RDY says so. */
    m->mem[MODEL_REG_INFO][1] = 0;
    m->mem[MODEL_REG_STRENGTH][0] = 0;
    m->mem[MODEL_REG_AREA][0] = 0;
    k_sleep(K_MSEC(100));
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600));
    k_sleep(K_MSEC(40));
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    const unsigned reads = m->report_reads;
    k_sleep(K_MSEC(300));
    CHECK(m->report_reads == reads);
    finish();
}

static void case_verify_bus_failure(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    /* The sensor cannot confirm the touch: the contact must not be stranded. */
    m->nack_next = 1;
    k_sleep(K_MSEC(200));
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    CHECK(count_ops(m, MODEL_REG_GESTURES, false) == 2);
    CHECK(nth_op(m, MODEL_REG_GESTURES, false, 1)->nack);
    /* The bus returns and the next report starts a new contact. */
    touch(m, 100, 200);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600), W_TOUCH(1), W_X(100),
           W_Y(200), W_TOUCH(0), W_X(100), W_Y(200));
    finish();
}

static void case_verify_disabled(void) {
    struct model *m = &models[S_NOV];
    bring_up(m);
    touch(m, 500, 600);
    k_sleep(K_MSEC(500));
    CHECK(count_ops(m, MODEL_REG_GESTURES, false) == 1);
    lift(m);
    EXPECT(S_NOV, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}

/* ----- Taps ----- */

static void case_tap_triple_lift(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0), W_CLICK(0), W_CLICK(0));
    check_click_spacing(S_POINTER, 0);
    CHECK(seen[0].ticks >= nth_op(m, MODEL_REG_GESTURES, false, 0)->ticks);
    CHECK(seen[0].ticks - nth_op(m, MODEL_REG_GESTURES, false, 0)->ticks <=
          (int64_t)k_ms_to_ticks_ceil64(1));
    finish();
}

static void case_tap_triple_after_contact(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    k_sleep(K_MSEC(5));
    frame(m, TAP_TRIPLE, 0, 0, 0, 0);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600), W_CLICK(0), W_CLICK(0),
           W_CLICK(0));
    check_click_spacing(S_POINTER, 0);
    finish();
}

static void case_tap_triple_during_touch(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    frame(m, TAP_TRIPLE, 0, 1, 500, 600);
    k_sleep(K_MSEC(30));
    lift(m);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    /* The press follows the report that carried it; the rest keep their pace across the lift. */
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_CLICK(0), W_TOUCH(0), W_X(500), W_Y(600), W_CLICK(0),
           W_CLICK(0));
    check_click_spacing(S_POINTER, 0);
    finish();
}

static void case_tap_relative_mode(void) {
    struct model *m = &models[S_REL];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_REL, W_CLICK(0), W_CLICK(0), W_CLICK(0));
    finish();
}

static void case_tap_button_map(void) {
    struct model *m = &models[S_MAP];
    bring_up(m);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    tap(m, TAP_DOUBLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    tap(m, TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_MAP, W_CLICK(1), W_CLICK(2), W_CLICK(2), W_CLICK(3), W_CLICK(3), W_CLICK(3));
    EXPECT_HID(HID_BUTTONS(BIT(1)), HID_BUTTONS(0), HID_BUTTONS(BIT(2)), HID_BUTTONS(0),
               HID_BUTTONS(BIT(2)), HID_BUTTONS(0), HID_BUTTONS(BIT(3)), HID_BUTTONS(0),
               HID_BUTTONS(BIT(3)), HID_BUTTONS(0), HID_BUTTONS(BIT(3)), HID_BUTTONS(0));
    finish();
}

static void case_tap_disabled(void) {
    struct model *m = &models[S_OFF];
    bring_up(m);
    tap(m, TAP_SINGLE);
    tap(m, TAP_DOUBLE);
    tap(m, TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT_NONE(S_OFF);
    /* Only the taps are off. */
    touch(m, 500, 600);
    lift(m);
    EXPECT(S_OFF, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}

static void case_tap_priority(void) {
    struct model *m = &models[S_MAP];
    bring_up(m);
    tap(m, TAP_SINGLE | TAP_DOUBLE | TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_MAP, W_CLICK(3), W_CLICK(3), W_CLICK(3));
    finish();
}

static void case_tap_other_gestures(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    frame(m, GESTURE_PALM, 0, 0, 0, 0);
    for (int bit = 0; bit < 8; bit++) {
        frame(m, 0, (uint8_t)BIT(bit), 0, 0, 0);
    }
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT_NONE(S_POINTER);
    finish();
}

/* A new tap ends what is playing: the press is released before the new button. */
static void case_tap_replaces_at_press(void) {
    struct model *m = &models[S_MAP];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_MAP, 1);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_MAP, W_CLICK(3), W_CLICK(1));
    check_click_spacing(S_MAP, 1);
    finish();
}

/* Between clicks nothing is held, so only the remaining clicks are dropped. */
static void case_tap_replaces_at_release(void) {
    struct model *m = &models[S_MAP];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_MAP, 2);
    tap(m, TAP_DOUBLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_MAP, W_CLICK(3), W_CLICK(2), W_CLICK(2));
    finish();
}

/* ----- RDY and the bus ----- */

/* RDY never rises again after a read: the level recheck finds it without an edge. */
static void case_rdy_level_recheck(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    m->hold_window_once = true;
    touch(m, 500, 600);
    k_sleep(K_MSEC(5));
    CHECK(count_ops(m, MODEL_REG_GESTURES, false) == 2);
    const int64_t gap = nth_op(m, MODEL_REG_GESTURES, false, 1)->ticks -
                        nth_op(m, MODEL_REG_GESTURES, false, 0)->ticks;
    CHECK(gap >= (int64_t)k_ms_to_ticks_ceil64(1));
    CHECK(gap <= (int64_t)k_ms_to_ticks_ceil64(4));
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}

/* A stuck RDY cannot spin the queue: four fast recoveries, then the back-off. */
static void case_rdy_stuck(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    m->rdy_stuck = true;
    k_timer_start(&m->event_timer, K_TICKS(1), K_NO_WAIT);
    k_sleep(K_MSEC(500));
    const int reads = count_ops(m, MODEL_REG_GESTURES, false);
    /* One from the edge, four fast, then one each 20 ms for the rest of the 500 ms. */
    CHECK(reads >= 20 && reads <= 32);
    const int64_t fast = (int64_t)k_ms_to_ticks_ceil64(4);
    const int64_t slow = (int64_t)k_ms_to_ticks_ceil64(20);
    for (int i = 1; i < 5; i++) {
        CHECK(nth_op(m, MODEL_REG_GESTURES, false, i)->ticks -
                  nth_op(m, MODEL_REG_GESTURES, false, i - 1)->ticks <= fast);
    }
    for (int i = 6; i < reads; i++) {
        CHECK(nth_op(m, MODEL_REG_GESTURES, false, i)->ticks -
                  nth_op(m, MODEL_REG_GESTURES, false, i - 1)->ticks >= slow);
    }
    EXPECT_NONE(S_POINTER);
    /* Released: the recovery state clears and the next edge reports as usual. */
    m->rdy_stuck = false;
    model_set_rdy(m, false);
    k_sleep(K_MSEC(60));
    CHECK(atomic_get(&driver_data(m)->rdy_recheck_attempts) == 0);
    const int settled = m->bus_count;
    k_sleep(K_MSEC(100));
    CHECK(m->bus_count == settled);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}

static void case_rdy_no_change(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    /* Fault injection: RDY despite no enabled change. */
    m->frame_event = true;
    signal_frame(m);
    m->frame_event = true;
    signal_frame(m);
    k_sleep(K_MSEC(50));
    EXPECT_NONE(S_POINTER);
    finish();
}

/* A failed report read leaves RDY up; the recheck reads it again, nothing lost. */
static void case_report_read_retry(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    m->nack_next = 1;
    touch(m, 501, 600); /* Raw movement triggers TP Event, held by deadband. */
    k_sleep(K_MSEC(5));
    CHECK(count_ops(m, MODEL_REG_GESTURES, false) == 3);
    CHECK(nth_op(m, MODEL_REG_GESTURES, false, 1)->nack);
    const int64_t retry = nth_op(m, MODEL_REG_GESTURES, false, 2)->ticks -
                          nth_op(m, MODEL_REG_GESTURES, false, 1)->ticks;
    CHECK(retry >= (int64_t)k_ms_to_ticks_ceil64(1));
    CHECK(retry <= (int64_t)k_ms_to_ticks_ceil64(4));
    CHECK(m->windows_timed_out == 0);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}

/* ----- A blocked consumer ----- */

/*
 * 1 held callback + 16 queued events = 17 accepted reports. A first contact
 * produces 3, subsequent samples 2 each: eight reads fill those 17 slots;
 * the ninth read blocks on X. Later sensor cycles expire independently.
 */
static void case_input_backpressure(void) {
    struct model *m = &models[S_NOV];
    static const struct model_frame script[] = {
        {.fingers=1, .x=500, .y=600}, {.fingers=1, .x=501, .y=600},
        {.fingers=1, .x=500, .y=600}, {.fingers=1, .x=501, .y=600},
        {.fingers=1, .x=500, .y=600}, {.fingers=1, .x=501, .y=600},
        {.fingers=1, .x=500, .y=600}, {.fingers=1, .x=501, .y=600},
        {.fingers=1, .x=500, .y=600}, {.g0=TAP_TRIPLE, .fingers=1, .x=501, .y=600},
        {.g0=TAP_TRIPLE, .fingers=1, .x=500, .y=600}, {.g0=TAP_TRIPLE, .fingers=1, .x=501, .y=600},
    };
    bring_up(m);
    const unsigned reads_before = m->report_reads;
    atomic_set(&input_gate_closed, 1);
    m->script = script;
    m->script_len = ARRAY_SIZE(script);
    /* 110 ms between frames, longer than the programmed 100-ms window. */
    k_timer_start(&m->event_timer, K_TICKS(1), K_MSEC(110));
    k_sleep(K_MSEC(1500));
    CHECK(CONFIG_INPUT_QUEUE_MAX_MSGS == 16);
    CHECK(atomic_get(&input_gate_waiting) == 1);
    CHECK(k_msgq_num_used_get(&input_msgq) == 16);
    CHECK(atomic_get(&seen_count) == 0);
    CHECK(m->report_reads - reads_before == 9);
    CHECK(m->script_next == ARRAY_SIZE(script));
    CHECK(m->windows_timed_out == 3 && m->reports_lost == 3);
    CHECK(m->last_timeout_ticks - m->last_timeout_opened >=
          (int64_t)k_ms_to_ticks_ceil64(100));
    CHECK(m->last_timeout_ticks - m->last_timeout_opened <=
          (int64_t)k_ms_to_ticks_ceil64(100) + 1);
    CHECK(m->mem[MODEL_REG_GESTURES][0] == 0);
    const unsigned stalled = m->report_reads;
    k_sleep(K_MSEC(200));
    CHECK(m->report_reads == stalled);
    CHECK(!m->rdy_low && !m->report_pending);
    k_sem_give(&input_gate);
    wait_reports(S_NOV, 19); /* Ninth read's two blocked reports also arrive. */
    k_sleep(K_MSEC(5));
    CHECK(m->report_reads == stalled); /* No replay of expired windows. */
    lift(m);
    tap(m, TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    /* 9 serviced contact packets: 1 edge + 9 XY pairs. +/-1 is inside deadband. */
    EXPECT(S_NOV, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600),
           W_CLICK(0), W_CLICK(0), W_CLICK(0));
    check_click_spacing(S_NOV, 0);
    finish();
}


/*
 * Stationary contact: verification fills 17 slots (1 held callback + 16 queued),
 * using 8 packets: 3 + 7*2 reports. An unmapped tap event supplies packet 9,
 * which blocks on X. Another unmapped tap window expires while verification becomes
 * queued on the blocked driver queue. Its forced read must still find a finger.
 */
static void case_input_backpressure_verify_contact(void) {
    struct model *m = &models[S_OFF];
    bring_up(m);
    CHECK(((const struct iqs7211e_config *)m->dev->config)->touch_verify_interval_ms == 120);
    const unsigned reads_before = m->report_reads;
    atomic_set(&input_gate_closed, 1);
    touch(m, 500, 600);
    WAIT_UNTIL(m->report_reads - reads_before == 8, 1100);
    CHECK(k_msgq_num_used_get(&input_msgq) == 16);
    CHECK(atomic_get(&input_gate_waiting) == 1);
    CHECK(atomic_get(&seen_count) == 0);
    /* The contact is stationary; unmapped tap events open windows but emit no clicks. */
    frame(m, TAP_SINGLE, 0, 1, 500, 600);
    CHECK(m->report_reads - reads_before == 9);
    model_load_frame(m, &(struct model_frame){.g0 = TAP_SINGLE,
                                             .fingers = 1, .x = 500, .y = 600});
    k_timer_start(&m->event_timer, K_TICKS(1), K_NO_WAIT);
    k_sleep(K_MSEC(200));
    CHECK(m->windows_timed_out == 1 && m->reports_lost == 1);
    CHECK(!m->rdy_low && !m->report_pending);
    CHECK(m->mem[MODEL_REG_GESTURES][0] == 0);
    CHECK(m->report_reads - reads_before == 9);
    CHECK(k_msgq_num_used_get(&input_msgq) == 16);
    CHECK(atomic_get(&input_gate_waiting) == 1);
    CHECK(atomic_get(&seen_count) == 0);
    /* The real verification timeout expired; its handler cannot run yet. */
    CHECK((k_work_busy_get(&driver_data(m)->touch_verify_work.work) & K_WORK_QUEUED) != 0);
    CHECK(k_uptime_ticks() - nth_op(m, MODEL_REG_GESTURES, false, 7)->ticks >=
          (int64_t)k_ms_to_ticks_ceil64(120));
    const int64_t resumed = k_uptime_ticks();
    k_sem_give(&input_gate);
    WAIT_UNTIL(m->report_reads - reads_before == 10, 250);
    wait_reports(S_OFF, 21);
    CHECK(nth_op(m, MODEL_REG_GESTURES, false, 9)->ticks >= resumed);
    CHECK(nth_op(m, MODEL_REG_GESTURES, false, 9)->ticks > m->last_timeout_ticks);
    /* 9 serviced packets plus the due verification: 1 edge + 10 XY pairs. */
    EXPECT(S_OFF, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600));
    CHECK(driver_data(m)->last_touched_state);
    lift(m);
    EXPECT(S_OFF, W_TOUCH(1), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_X(500), W_Y(600), W_X(500), W_Y(600),
           W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}


/* ----- A reset at runtime ----- */

/* The part resets by itself, then streams again with Show Reset set. */
static void runtime_reset(struct model *m) {
    const int settings = count_ops(m, 0x1F, true);
    model_reset_now(m);
    WAIT_UNTIL(count_ops(m, 0x1F, true) > settings, 500);
    wait_ready(m);
    check_programmed(m);
}

static void case_reset_idle(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    runtime_reset(m);
    /* The settings -> ACK -> ATI -> event mode sequence, without product or reset steps. */
    check_setup_sequence(m, 0);
    EXPECT_NONE(S_POINTER);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}

static void case_reset_at_press(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_POINTER, 1);
    runtime_reset(m);
    /* The one release owed goes out; the clicks still to come do not. */
    EXPECT(S_POINTER, W_CLICK(0));
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0), W_CLICK(0));
    finish();
}

static void case_reset_at_release(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_POINTER, 2);
    runtime_reset(m);
    /* Nothing is held between clicks: no extra release, no more presses. */
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}

static void case_reset_touching(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    runtime_reset(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    CHECK(!driver_data(m)->last_touched_state);
    const unsigned reads = m->report_reads;
    k_sleep(K_MSEC(300));
    /* The old contact's verification died with it. */
    CHECK(m->report_reads == reads);
    touch(m, 100, 200);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600), W_TOUCH(1), W_X(100),
           W_Y(200), W_TOUCH(0), W_X(100), W_Y(200));
    finish();
}

/* ----- Power management ----- */

static void case_pm_idle(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    pm_suspend(m);
    CHECK(pm_device_action_run(m->dev, PM_DEVICE_ACTION_SUSPEND) == -EALREADY);
    CHECK(count_writes(m) == 1);
    const struct model_bus_op *sleep = nth_op(m, MODEL_REG_SYS_CONTROL, true, 0);
    CHECK(sleep->b0 == 0x00 && sleep->b1 == MODEL_SUSPEND);
    CHECK(m->suspended);
    /* An edge while suspended wakes nothing. */
    model_set_rdy(m, true);
    k_sleep(K_MSEC(50));
    model_set_rdy(m, false);
    CHECK(m->bus_count == 1);
    pm_resume(m);
    CHECK(count_writes(m) == 2);
    const struct model_bus_op *wake = nth_op(m, MODEL_REG_SYS_CONTROL, true, 1);
    CHECK(wake->b0 == 0x00 && wake->b1 == 0x00);
    CHECK(!m->suspended);
    /* The memory map is retained: no setup is repeated. */
    k_sleep(K_MSEC(100));
    CHECK(count_writes(m) == 2);
    CHECK(driver_data(m)->init_state == IQS7211E_INIT_DONE);
    EXPECT_NONE(S_POINTER);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}

static void case_pm_at_press(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_POINTER, 1);
    pm_suspend(m);
    /* No yield after the PM call: delivery must already be observable. */
    EXPECT(S_POINTER, W_CLICK(0));
    k_sleep(K_MSEC(100));
    pm_resume(m);
    k_sleep(K_MSEC(100));
    /* Cancelled clicks are not replayed. */
    EXPECT(S_POINTER, W_CLICK(0));
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0), W_CLICK(0));
    finish();
}

static void case_pm_at_release(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_POINTER, 2);
    pm_suspend(m);
    k_sleep(K_MSEC(100));
    pm_resume(m);
    k_sleep(K_MSEC(100));
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}

static void case_pm_touching(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    pm_suspend(m);
    k_sleep(K_MSEC(1));
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    const unsigned reads = m->report_reads;
    /* Past several verification periods: the timer was cancelled. */
    k_sleep(K_MSEC(400));
    CHECK(m->report_reads == reads);
    pm_resume(m);
    touch(m, 100, 200);
    lift(m);
    EXPECT(S_POINTER, W_TOUCH(1), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600), W_TOUCH(1), W_X(100),
           W_Y(200), W_TOUCH(0), W_X(100), W_Y(200));
    finish();
}

/* A dead bus must not abort a power-off: the driver side still quiesces. */
static void case_pm_bus_down_at_suspend(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    wait_reports(S_POINTER, 1);
    m->bus_down = true;
    pm_suspend(m);
    k_sleep(K_MSEC(1));
    EXPECT(S_POINTER, W_CLICK(0));
    CHECK(!driver_data(m)->sensor_suspended);
    CHECK(atomic_get(&driver_data(m)->suspended) != 0);
    m->bus_down = false;
    const int attempts = m->bus_count;
    pm_resume(m);
    /* It was never put to sleep, so nothing is written to wake it. */
    k_sleep(K_MSEC(50));
    CHECK(m->bus_count == attempts);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0), W_CLICK(0));
    finish();
}

/* The wake is retried, fast and then slowly, until the bus returns. */
static void case_pm_bus_down_at_resume(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    pm_suspend(m);
    m->bus_down = true;
    m->bus_count = 0;
    pm_resume(m);
    CHECK(driver_data(m)->sensor_suspended);
    k_sleep(K_MSEC(300));
    /* The attempt in resume, one at the first recheck, then two more 20 ms apart. */
    CHECK(m->bus_count == 4);
    const int64_t fast = (int64_t)k_ms_to_ticks_ceil64(20);
    CHECK(m->bus[2].ticks - m->bus[1].ticks >= fast);
    CHECK(m->bus[3].ticks - m->bus[2].ticks >= fast);
    k_sleep(K_MSEC(800));
    const int64_t slow = (int64_t)k_ms_to_ticks_ceil64(1000);
    CHECK(m->bus_count == 5);
    CHECK(m->bus[4].ticks - m->bus[3].ticks >= slow);
    for (int i = 0; i < 5; i++) {
        CHECK(m->bus[i].write && m->bus[i].nack && m->bus[i].b1 == 0x00);
    }
    m->bus_down = false;
    WAIT_UNTIL(!driver_data(m)->sensor_suspended, 1100);
    CHECK(!m->suspended);
    CHECK(driver_data(m)->sensor_resume_attempts == 0);
    EXPECT_NONE(S_POINTER);
    tap(m, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0));
    finish();
}

/* Suspended during setup: resume starts over rather than continuing half-way. */
static void case_pm_during_setup(void) {
    struct model *m = &models[S_POINTER];
    model_power_on(m, MODEL_FRESH);
    WAIT_UNTIL(driver_data(m)->init_state == IQS7211E_INIT_ATI, 1000);
    pm_suspend(m);
    CHECK(m->suspended);
    k_sleep(K_MSEC(50));
    pm_resume(m);
    CHECK(driver_data(m)->init_state == IQS7211E_INIT_VERIFY_PRODUCT);
    CHECK(count_ops(m, MODEL_REG_PRODUCT, false) == 1);
    wait_ready(m);
    CHECK(count_ops(m, MODEL_REG_PRODUCT, false) == 2);
    /* The earlier ACK cleared Show Reset, so this pass resets the part first. */
    CHECK(m->resets_seen == 1);
    check_programmed(m);
    EXPECT_NONE(S_POINTER);
    finish();
}

/* The early device suspends every sensor before the bus goes; their own turn then skips. */
static void case_pm_early_device(void) {
    struct model *a = &models[S_POINTER];
    struct model *b = &models[S_MAP];
    /* The early device intentionally visits every DT sensor: make all present,
     * so absent-device finish checks remain limited to boot attempts. */
    for (int s = 0; s < SENSOR_COUNT; s++) {
        bring_up(&models[s]);
    }
    tap(a, TAP_TRIPLE);
    wait_reports(S_POINTER, 1);
    const struct device *early = DEVICE_GET(iqs7211e_early_pm);
    CHECK(pm_device_action_run(early, PM_DEVICE_ACTION_SUSPEND) == 0);
    k_sleep(K_MSEC(1));
    EXPECT(S_POINTER, W_CLICK(0));
    CHECK(a->suspended && b->suspended);
    CHECK(count_writes(a) == 1 && count_writes(b) == 1);
    for (int s = 0; s < SENSOR_COUNT; s++) {
        CHECK(models[s].suspended && count_writes(&models[s]) == 1);
        pm_suspend(&models[s]);
        CHECK(count_writes(&models[s]) == 1);
    }
    CHECK(pm_device_action_run(early, PM_DEVICE_ACTION_RESUME) == 0);
    for (int s = 0; s < SENSOR_COUNT; s++) {
        pm_resume(&models[s]);
        CHECK(!models[s].suspended);
    }
    CHECK(!a->suspended && !b->suspended);
    tap(a, TAP_SINGLE);
    tap(b, TAP_SINGLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0), W_CLICK(0));
    EXPECT(S_MAP, W_CLICK(1));
    finish();
}

/* ----- Two sensors on one queue ----- */

static void case_two_sensors_independent(void) {
    struct model *a = &models[S_POINTER];
    struct model *b = &models[S_MAP];
    bring_up(a);
    bring_up(b);
    tap(a, TAP_TRIPLE);
    k_sleep(K_MSEC(5));
    tap(b, TAP_TRIPLE);
    touch(b, 500, 600);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    lift(b);
    EXPECT(S_POINTER, W_CLICK(0), W_CLICK(0), W_CLICK(0));
    check_click_spacing(S_POINTER, 0);
    check_click_spacing(S_MAP, 3);
    /* Press, contact, remaining five edges at >=20 ms, verification at 120 ms,
     * then the lift at 150 ms. Constant coordinates stay 500/600 through filtering. */
    EXPECT(S_MAP, W_BTN(3, 1), W_TOUCH(1), W_X(500), W_Y(600), W_BTN(3, 0),
           W_CLICK(3), W_CLICK(3), W_X(500), W_Y(600), W_TOUCH(0), W_X(500), W_Y(600));
    finish();
}

static void case_two_sensors_suspend_one(void) {
    struct model *a = &models[S_POINTER];
    struct model *b = &models[S_MAP];
    bring_up(a);
    bring_up(b);
    tap(a, TAP_TRIPLE);
    tap(b, TAP_TRIPLE);
    wait_reports(S_POINTER, 1);
    pm_suspend(a);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT(S_POINTER, W_CLICK(0));
    /* The other sensor plays on untouched. */
    EXPECT(S_MAP, W_CLICK(3), W_CLICK(3), W_CLICK(3));
    pm_resume(a);
    finish();
}

/* ----- Through the real ZMK listener ----- */

static void case_hid_triple_tap(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    tap(m, TAP_TRIPLE);
    k_sleep(K_MSEC(CLICKS_DONE_MS));
    EXPECT_HID(HID_BUTTONS(1), HID_BUTTONS(0), HID_BUTTONS(1), HID_BUTTONS(0), HID_BUTTONS(1),
               HID_BUTTONS(0));
    finish();
}

/*
 * Without an input processor to consume it, ZMK turns the absolute contact edge
 * into the left button. A production route must drop BTN_TOUCH; this pins why.
 */
static void case_hid_abs_contact_is_left_button(void) {
    struct model *m = &models[S_POINTER];
    bring_up(m);
    touch(m, 500, 600);
    lift(m);
    EXPECT_HID(HID_BUTTONS(1), HID_BUTTONS(0));
    finish();
}

/* ----- The table ----- */

#define CASE(name) {#name, case_##name}
static const struct {
    const char *name;
    void (*run)(void);
} cases[] = {
    CASE(boot_fresh),
    CASE(boot_silent_event_mode),
    CASE(boot_silent_suspended),
    CASE(boot_absent),
    CASE(boot_wrong_product),
    CASE(boot_bus_errors),
    CASE(boot_reset_lost),
    CASE(contact_abs),
    CASE(contact_rel),
    CASE(contact_invalid),
    CASE(contact_second),
    CASE(rotate_90),
    CASE(rotate_180),
    CASE(rotate_270),
    CASE(verify_holds_contact),
    CASE(verify_finds_lift),
    CASE(verify_bus_failure),
    CASE(verify_disabled),
    CASE(tap_triple_lift),
    CASE(tap_triple_after_contact),
    CASE(tap_triple_during_touch),
    CASE(tap_relative_mode),
    CASE(tap_button_map),
    CASE(tap_disabled),
    CASE(tap_priority),
    CASE(tap_other_gestures),
    CASE(tap_replaces_at_press),
    CASE(tap_replaces_at_release),
    CASE(rdy_level_recheck),
    CASE(rdy_stuck),
    CASE(rdy_no_change),
    CASE(report_read_retry),
    CASE(input_backpressure),
    CASE(input_backpressure_verify_contact),
    CASE(reset_idle),
    CASE(reset_at_press),
    CASE(reset_at_release),
    CASE(reset_touching),
    CASE(pm_idle),
    CASE(pm_at_press),
    CASE(pm_at_release),
    CASE(pm_touching),
    CASE(pm_bus_down_at_suspend),
    CASE(pm_bus_down_at_resume),
    CASE(pm_during_setup),
    CASE(pm_early_device),
    CASE(two_sensors_independent),
    CASE(two_sensors_suspend_one),
    CASE(hid_triple_tap),
    CASE(hid_abs_contact_is_left_button),
};

static void tests(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    const char *which = getenv("IQS_KERNEL_CASE");
    CHECK(which != NULL);
    if (strcmp(which, "--list") == 0) {
        for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
            printk("IQSCASE %s\n", cases[i].name);
        }
        exit(0);
    }
    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        if (strcmp(which, cases[i].name) == 0) {
            current_case = cases[i].name;
            cases[i].run();
            /* Each case ends in finish(). */
            fail(__LINE__, "case returned");
        }
    }
    fail(__LINE__, "unknown case");
}
K_THREAD_DEFINE(iqs_events, 8192, tests, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
