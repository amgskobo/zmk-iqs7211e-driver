/* Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */
/*
 * A behavioural model of the IQS7211E behind the real Zephyr I2C emulator bus.
 * Register numbers and bit positions below are written as literals from the
 * memory map, independently of the driver's macros, so a wrong driver constant
 * is seen as a wrong transaction. RDY is driven from timer ISRs on the emulated
 * GPIO. The model is not a sensor: it has no capacitive front end or gesture
 * classifier, and its timings are chosen, not measured. Forced communication
 * completes immediately: clock stretching (§11.9.2), the wake reseed (§10.2),
 * and electrical timing are not modelled. Successful STOP ends a window (§11.7).
 */
#ifndef IQS_KERNEL_SENSOR_MODEL_H_
#define IQS_KERNEL_SENSOR_MODEL_H_

#define MODEL_REG_PRODUCT 0x00
#define MODEL_REG_GESTURES 0x0E
#define MODEL_REG_INFO 0x0F
#define MODEL_REG_X 0x10
#define MODEL_REG_Y 0x11
#define MODEL_REG_STRENGTH 0x12
#define MODEL_REG_AREA 0x13
#define MODEL_REG_TIMEOUT 0x32
#define MODEL_REG_SYS_CONTROL 0x33
#define MODEL_REG_CONFIG 0x34
#define MODEL_WORDS 0x80

/* INFO_FLAGS byte 0. */
#define MODEL_SHOW_RESET 0x80
#define MODEL_RE_ATI_OCCURRED 0x10
/* SYSTEM_CONTROL byte 0 and byte 1. */
#define MODEL_ACK_RESET 0x80
#define MODEL_TP_RE_ATI 0x20
#define MODEL_SW_RESET 0x02
#define MODEL_SUSPEND 0x08
/* CONFIG_SETTINGS byte 1. */
#define MODEL_EVENT_MODE 0x01
#define MODEL_GESTURE_EVENT 0x02
#define MODEL_TP_EVENT 0x04

#define MODEL_CYCLE_MS 10
#define MODEL_RESET_MS 170
#define MODEL_BUS_LOG 1024
#define MODEL_SCRIPT 32

enum model_power {
    /* Powered up from nothing: streaming, Show Reset set. */
    MODEL_FRESH,
    /* Streaming, but the reset flag was acknowledged before this boot. */
    MODEL_STREAMING_NO_RESET,
    /* Kept its power through the SoC's reset, in event mode: silent. */
    MODEL_RETAINED_EVENT,
    /* Kept its power, suspended by the soft-off before this boot: silent. */
    MODEL_RETAINED_SUSPENDED,
};

struct model_bus_op {
    int64_t ticks;
    uint8_t reg;
    uint8_t len;
    uint8_t b0;
    uint8_t b1;
    bool write;
    bool nack;
};

struct model_frame {
    uint8_t g0;
    uint8_t fingers;
    uint16_t x;
    uint16_t y;
};

struct model {
    int index;
    const struct device *dev;
    const struct device *port;
    gpio_pin_t pin;

    uint8_t mem[MODEL_WORDS][2];
    bool present;
    bool event_mode;
    bool suspended;
    bool resetting;
    bool rdy_low;
    /* Unread data belongs only to the current communication window (§11.6). */
    bool report_pending;
    bool frame_event;
    struct model_frame previous_frame;
    unsigned windows_timed_out;
    unsigned reports_lost;
    int64_t window_opened;
    int64_t last_timeout_opened;
    int64_t last_timeout_ticks;
    unsigned ati_observed;
    /* RDY held active whatever the bus does. */
    bool rdy_stuck;
    /* The next closed window stays open once: an edge the driver never sees. */
    bool hold_window_once;
    /* Every transaction fails. */
    bool bus_down;
    /* The next nack_next transactions fail, after nack_after that succeed. */
    int nack_after;
    int nack_next;
    /* One write to this register fails; 0 for none. */
    uint8_t nack_write_reg;
    /* This many reset commands are lost by the part. */
    int ignore_resets;
    int ati_cycles;
    unsigned resets_seen;
    unsigned report_reads;

    const struct model_frame *script;
    int script_len;
    int script_next;

    struct k_timer cycle_timer;
    struct k_timer event_timer;
    struct k_timer reset_timer;
    struct k_timer window_timer;

    struct model_bus_op bus[MODEL_BUS_LOG];
    int bus_count;
};

static void model_set_rdy(struct model *m, bool low) {
    m->rdy_low = low;
    const int ret = gpio_emul_input_set(m->port, m->pin, low ? 0 : 1);
    if (ret != 0) {
        printk("IQSFAIL gpio_emul_input_set: %d\n", ret);
        exit(1);
    }
}

static bool model_event_mode(const struct model *m) {
    return m->event_mode && !(m->mem[MODEL_REG_INFO][0] & MODEL_SHOW_RESET);
}

static void model_discard_report(struct model *m) {
    m->mem[MODEL_REG_GESTURES][0] = 0;
    m->mem[MODEL_REG_GESTURES][1] = 0;
    m->report_pending = false;
}

static void model_window_timeout(struct k_timer *timer) {
    struct model *m = CONTAINER_OF(timer, struct model, window_timer);
    if (!m->rdy_low || m->rdy_stuck) {
        return;
    }
    m->last_timeout_opened = m->window_opened;
    m->last_timeout_ticks = k_uptime_ticks();
    m->windows_timed_out++;
    m->reports_lost += m->report_pending;
    model_discard_report(m);
    /* §11.6 loses this window's events, not the current sensing sample.
     * A later forced/verification read still sees a finger that remains down. */
    model_set_rdy(m, false);
}

static void model_open_window(struct model *m) {
    if (m->rdy_low) {
        return;
    }
    unsigned timeout = m->mem[MODEL_REG_TIMEOUT][0] |
                       ((unsigned)m->mem[MODEL_REG_TIMEOUT][1] << 8);
    m->window_opened = k_uptime_ticks();
    k_timer_start(&m->window_timer, K_MSEC(timeout), K_NO_WAIT);
    model_set_rdy(m, true);
}

/* STOP ends the session; suspend becomes effective after this session (§10.2). */
static void model_end_window(struct model *m) {
    if (m->rdy_stuck) {
        return;
    }
    if (m->hold_window_once) {
        m->hold_window_once = false;
        return;
    }
    k_timer_stop(&m->window_timer);
    model_discard_report(m);
    model_set_rdy(m, false);
    m->suspended = (m->mem[MODEL_REG_SYS_CONTROL][1] & MODEL_SUSPEND) != 0;
}

static void model_reset_now(struct model *m) {
    m->resetting = false;
    m->report_pending = false;
    m->event_mode = false;
    m->suspended = false;
    m->ati_cycles = 0;
    m->frame_event = false;
    m->previous_frame = (struct model_frame){0};
    k_timer_stop(&m->window_timer);
    memset(m->mem, 0, sizeof(m->mem));
    m->mem[MODEL_REG_PRODUCT][0] = 0x58;
    m->mem[MODEL_REG_PRODUCT][1] = 0x04;
    m->mem[MODEL_REG_INFO][0] = MODEL_SHOW_RESET;
    /* Table A (0x32) and §11.6 give no default: assume 100 ms until programmed. */
    m->mem[MODEL_REG_TIMEOUT][0] = 100;
}

static void model_cycle_handler(struct k_timer *timer) {
    struct model *m = CONTAINER_OF(timer, struct model, cycle_timer);

    if (!m->present || m->suspended || m->resetting || m->rdy_stuck) {
        return;
    }
    if (m->rdy_low) {
        return; /* Processing resumes when STOP or the I2C timeout ends the session. */
    }
    /* §11.9.1: the completion indication lasts one processing/comms cycle. */
    m->mem[MODEL_REG_INFO][0] &= ~MODEL_RE_ATI_OCCURRED;
    if (m->ati_cycles > 0 && --m->ati_cycles == 0) {
        m->mem[MODEL_REG_INFO][0] |= MODEL_RE_ATI_OCCURRED;
    }
    if (!model_event_mode(m)) {
        model_open_window(m);
    }
}

static void model_load_frame(struct model *m, const struct model_frame *f);

static void model_event_handler(struct k_timer *timer) {
    struct model *m = CONTAINER_OF(timer, struct model, event_timer);
    /* Scripts advance on the sensor clock, never on a host read. */
    if (m->script != NULL && m->script_next < m->script_len) {
        model_load_frame(m, &m->script[m->script_next++]);
    }
    if (m->present && !m->suspended && !m->resetting &&
        (!model_event_mode(m) || m->frame_event || m->rdy_stuck)) {
        m->report_pending = true;
        model_open_window(m);
    }
    m->frame_event = false;
    if (m->script != NULL && m->script_next == m->script_len) {
        k_timer_stop(&m->event_timer);
    }
}

static void model_reset_handler(struct k_timer *timer) {
    model_reset_now(CONTAINER_OF(timer, struct model, reset_timer));
}

static void model_apply_sys_control(struct model *m) {
    const uint8_t b0 = m->mem[MODEL_REG_SYS_CONTROL][0];
    const uint8_t b1 = m->mem[MODEL_REG_SYS_CONTROL][1];

    /* Applied at STOP, after the command session. */
    if ((b0 & MODEL_ACK_RESET) != 0) {
        m->mem[MODEL_REG_INFO][0] &= ~MODEL_SHOW_RESET;
    }
    if ((b0 & MODEL_TP_RE_ATI) != 0) {
        m->mem[MODEL_REG_INFO][0] &= ~MODEL_RE_ATI_OCCURRED;
        m->ati_cycles = 2;
    }
    if ((b1 & MODEL_SW_RESET) != 0) {
        m->resets_seen++;
        if (m->ignore_resets > 0) {
            m->ignore_resets--;
        } else {
            m->resetting = true;
            k_timer_start(&m->reset_timer, K_MSEC(MODEL_RESET_MS), K_NO_WAIT);
        }
    }
    /* One-shot commands read back as zero. */
    m->mem[MODEL_REG_SYS_CONTROL][0] &= ~(MODEL_ACK_RESET | MODEL_TP_RE_ATI);
    m->mem[MODEL_REG_SYS_CONTROL][1] &= ~MODEL_SW_RESET;
}

static void model_load_frame(struct model *m, const struct model_frame *f) {
    const struct model_frame *old = &m->previous_frame;
    const uint8_t enabled = m->mem[MODEL_REG_CONFIG][1];
    m->frame_event = ((enabled & MODEL_GESTURE_EVENT) && f->g0 != 0) ||
        ((enabled & MODEL_TP_EVENT) &&
         (f->fingers != old->fingers || f->x != old->x || f->y != old->y));
    m->previous_frame = *f;
    m->mem[MODEL_REG_GESTURES][0] = f->g0;
    m->mem[MODEL_REG_INFO][1] = f->fingers;
    m->mem[MODEL_REG_X][0] = (uint8_t)f->x;
    m->mem[MODEL_REG_X][1] = (uint8_t)(f->x >> 8);
    m->mem[MODEL_REG_Y][0] = (uint8_t)f->y;
    m->mem[MODEL_REG_Y][1] = (uint8_t)(f->y >> 8);
    m->mem[MODEL_REG_STRENGTH][0] = f->fingers ? 100 : 0;
    m->mem[MODEL_REG_AREA][0] = f->fingers ? 2 : 0;
}

static int model_i2c_transfer(const struct emul *target, struct i2c_msg *msgs, int num_msgs, int addr) {
    struct model *m = target->data;

    ARG_UNUSED(addr);
    if (num_msgs != 2 || msgs[0].len != 1 || (msgs[0].flags & I2C_MSG_READ) != 0) {
        printk("IQSFAIL unexpected I2C shape: msgs=%d\n", num_msgs);
        exit(1);
    }

    const uint8_t reg = msgs[0].buf[0];
    const bool write = (msgs[1].flags & I2C_MSG_READ) == 0;
    const uint8_t len = (uint8_t)msgs[1].len;
    uint8_t *buf = msgs[1].buf;
    bool nack = m->bus_down || !m->present;

    if (!nack && m->nack_after > 0) {
        m->nack_after--;
    } else if (!nack && m->nack_next > 0) {
        m->nack_next--;
        nack = true;
    }
    if (!nack && write && m->nack_write_reg != 0 && m->nack_write_reg == reg) {
        m->nack_write_reg = 0;
        nack = true;
    }

    if (m->bus_count == MODEL_BUS_LOG) {
        fail(__LINE__, "bus log overflow");
    }
    {
        m->bus[m->bus_count++] = (struct model_bus_op){
            .ticks = k_uptime_ticks(), .reg = reg, .len = len, .write = write, .nack = nack,
            .b0 = write && len > 0 ? buf[0] : 0, .b1 = write && len > 1 ? buf[1] : 0};
    }
    if (nack) {
        return -EIO;
    }

    for (uint8_t i = 0; i < len; i++) {
        const unsigned word = (unsigned)reg + i / 2U;
        if (word >= MODEL_WORDS) {
            printk("IQSFAIL register out of range: 0x%02x+%u\n", reg, i);
            exit(1);
        }
        if (write) {
            m->mem[word][i % 2U] = buf[i];
        } else {
            buf[i] = m->mem[word][i % 2U];
        }
    }

    if (!write) {
        m->bus[m->bus_count - 1].b0 = len > 0 ? buf[0] : 0;
        m->bus[m->bus_count - 1].b1 = len > 1 ? buf[1] : 0;
    }
    if (!write && reg == MODEL_REG_INFO && (buf[0] & MODEL_RE_ATI_OCCURRED)) {
        m->ati_observed++;
    }
    if (write) {
        const unsigned last = (unsigned)reg + (len - 1U) / 2U;
        if (reg <= MODEL_REG_SYS_CONTROL && last >= MODEL_REG_SYS_CONTROL) {
            model_apply_sys_control(m);
        }
        if (reg <= MODEL_REG_CONFIG && last >= MODEL_REG_CONFIG) {
            m->event_mode = (m->mem[MODEL_REG_CONFIG][1] & MODEL_EVENT_MODE) != 0;
        }
    } else if (reg == MODEL_REG_GESTURES && len == 12) {
        /* Gesture flags are events: they clear once read. */
        m->mem[MODEL_REG_GESTURES][0] = 0;
        m->mem[MODEL_REG_GESTURES][1] = 0;
        m->report_pending = false;
        m->report_reads++;
    }
    model_end_window(m);
    return 0;
}

static int model_emul_init(const struct emul *target, const struct device *parent) {
    ARG_UNUSED(target);
    ARG_UNUSED(parent);
    return 0;
}

static const struct i2c_emul_api model_i2c_api = {.transfer = model_i2c_transfer};

static void model_prepare(struct model *m) {
    k_timer_init(&m->cycle_timer, model_cycle_handler, NULL);
    k_timer_init(&m->event_timer, model_event_handler, NULL);
    k_timer_init(&m->reset_timer, model_reset_handler, NULL);
    k_timer_init(&m->window_timer, model_window_timeout, NULL);
    (void)gpio_pin_configure(m->port, m->pin, GPIO_INPUT | GPIO_ACTIVE_LOW);
    model_set_rdy(m, false);
}

static void model_power_on(struct model *m, enum model_power power) {
    model_reset_now(m);
    switch (power) {
    case MODEL_FRESH:
        break;
    case MODEL_STREAMING_NO_RESET:
        m->mem[MODEL_REG_INFO][0] = 0;
        break;
    case MODEL_RETAINED_EVENT:
        m->mem[MODEL_REG_INFO][0] = 0;
        m->mem[MODEL_REG_CONFIG][1] = MODEL_EVENT_MODE;
        m->event_mode = true;
        break;
    case MODEL_RETAINED_SUSPENDED:
        m->mem[MODEL_REG_INFO][0] = 0;
        m->mem[MODEL_REG_SYS_CONTROL][1] = MODEL_SUSPEND;
        m->mem[MODEL_REG_CONFIG][1] = MODEL_EVENT_MODE;
        m->event_mode = true;
        m->suspended = true;
        break;
    }
    m->present = true;
    k_timer_start(&m->cycle_timer, K_MSEC(MODEL_CYCLE_MS), K_MSEC(MODEL_CYCLE_MS));
}

#endif /* IQS_KERNEL_SENSOR_MODEL_H_ */
