/*
 * Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 *
 * Stand-ins for the Zephyr APIs the driver uses, so its own header - and with
 * it the real struct iqs7211e_data - compiles on the host. The zephyr/ headers
 * beside this file include it. The I2C bus is a register model with fault
 * injection; GPIO, work queues and input record what the driver asks of them.
 */

#pragma once

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Not every harness uses every stand-in. */
#define STUB __attribute__((unused))
#define BIT(n) (1UL << (n))
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define ARG_UNUSED(x) ((void)(x))
#define CONTAINER_OF(ptr, type, field) ((type *)(void *)((char *)(ptr) - offsetof(type, field)))
/* Unevaluated printf() keeps the format strings checked and the arguments used. */
#define LOG_DBG(fmt, ...) ((void)sizeof(printf(fmt, ##__VA_ARGS__)))
#define LOG_INF(fmt, ...) LOG_DBG(fmt, ##__VA_ARGS__)
#define LOG_WRN(fmt, ...) ((void)log_warnings++, LOG_DBG(fmt, ##__VA_ARGS__))
#define LOG_ERR(fmt, ...) ((void)log_errors++, LOG_DBG(fmt, ##__VA_ARGS__))
#define K_FOREVER ((k_timeout_t){-1})
#define K_NO_WAIT ((k_timeout_t){0})
#define K_MSEC(ms) ((k_timeout_t){(ms)})
#define K_TICKS(value) ((k_timeout_t){(value)})
#define CONFIG_SYS_CLOCK_TICKS_PER_SEC 1000
#define K_THREAD_STACK_SIZEOF(stack) sizeof(stack)
#define GPIO_INPUT 0x10
#define GPIO_INT_DISABLE 0
#define GPIO_INT_EDGE_TO_ACTIVE 1
#define INPUT_ABS_X 0x00
#define INPUT_ABS_Y 0x01
#define INPUT_REL_X 0x00
#define INPUT_REL_Y 0x01
#define INPUT_BTN_0 0x100
#define INPUT_BTN_1 0x101
#define INPUT_BTN_2 0x102
#define INPUT_BTN_TOUCH 0x14a

typedef long atomic_t;
typedef long atomic_val_t;
typedef uint32_t gpio_port_pins_t;
typedef struct { int64_t ms; } k_timeout_t;
struct k_thread { int unused; };
typedef struct k_thread *k_tid_t;
struct k_work { void (*handler)(struct k_work *work); bool pending; };
struct k_work_delayable { struct k_work work; bool scheduled; int64_t delay_ms; };
struct k_work_sync { int unused; };
struct k_work_q { struct k_thread thread; bool started; int priority; };
struct k_work_queue_config { const char *name; bool no_yield; bool essential; };
struct device { const char *name; const void *config; void *data; bool ready; };
struct i2c_dt_spec { const struct device *bus; uint16_t addr; };
struct gpio_dt_spec { const struct device *port; uint8_t pin; uint16_t dt_flags; };
struct gpio_callback;
typedef void (*gpio_callback_handler_t)(const struct device *port, struct gpio_callback *cb,
                                        gpio_port_pins_t pins);
struct gpio_callback { gpio_callback_handler_t handler; gpio_port_pins_t pin_mask; };

static STUB int log_warnings;
static STUB int log_errors;

/* ----- Work queues ----- */
static STUB int submit_result;
static STUB int reschedule_result;
static STUB int submits;
static STUB struct k_thread *current_thread;
static STUB struct k_thread other_thread;
static STUB int flushes;

static STUB atomic_val_t atomic_get(const atomic_t *target) { return *target; }
static STUB atomic_val_t atomic_set(atomic_t *target, atomic_val_t value) {
    const atomic_val_t old = *target;
    *target = value;
    return old;
}
static STUB atomic_val_t atomic_inc(atomic_t *target) { return (*target)++; }
static STUB atomic_val_t atomic_clear(atomic_t *target) { return atomic_set(target, 0); }
static STUB void k_work_queue_init(struct k_work_q *queue) { memset(queue, 0, sizeof(*queue)); }
static STUB void k_work_queue_start(struct k_work_q *queue, void *stack, size_t size, int prio,
                                    const struct k_work_queue_config *cfg) {
    assert(stack != NULL && size > 0 && cfg != NULL && !cfg->essential && !cfg->no_yield);
    queue->started = true;
    queue->priority = prio;
}
static STUB k_tid_t k_work_queue_thread_get(struct k_work_q *queue) { return &queue->thread; }
static STUB k_tid_t k_current_get(void) { return current_thread != NULL ? current_thread : &other_thread; }
static STUB void k_work_init(struct k_work *work, void (*handler)(struct k_work *work)) {
    *work = (struct k_work){.handler = handler};
}
static STUB void k_work_init_delayable(struct k_work_delayable *work,
                                       void (*handler)(struct k_work *work)) {
    *work = (struct k_work_delayable){.work = {.handler = handler}};
}
static STUB struct k_work_delayable *k_work_delayable_from_work(struct k_work *work) {
    return CONTAINER_OF(work, struct k_work_delayable, work);
}
static STUB int k_work_submit_to_queue(struct k_work_q *queue, struct k_work *work) {
    assert(queue->started);
    if (submit_result < 0) {
        return submit_result;
    }
    const int queued_now = !work->pending;
    work->pending = true;
    submits++;
    return queued_now;
}
static STUB int k_work_reschedule_for_queue(struct k_work_q *queue, struct k_work_delayable *work,
                                            k_timeout_t delay) {
    assert(queue->started);
    if (reschedule_result < 0) {
        return reschedule_result;
    }
    /* K_NO_WAIT on an item already queued leaves it where it is. */
    const int queued_now = delay.ms != 0 || !work->scheduled;
    work->scheduled = true;
    work->delay_ms = delay.ms;
    return queued_now;
}
static STUB int k_work_cancel_delayable(struct k_work_delayable *work) {
    work->scheduled = false;
    return 0;
}
static STUB bool k_work_cancel_delayable_sync(struct k_work_delayable *work,
                                              struct k_work_sync *sync) {
    ARG_UNUSED(sync);
    work->scheduled = false;
    return false;
}
static STUB bool k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync) {
    ARG_UNUSED(sync);
    work->pending = false;
    return false;
}
/* The queue runs what was submitted before the flush returns. */
static STUB bool k_work_flush(struct k_work *work, struct k_work_sync *sync) {
    ARG_UNUSED(sync);
    flushes++;
    if (work->pending) {
        work->pending = false;
        work->handler(work);
    }
    return false;
}

/* ----- I2C: the sensor's memory map ----- */
/* Word addressed, as the part is: each address holds two bytes. */
static STUB uint8_t regs[512];
#define REG(address) (&regs[(size_t)(address) * 2])
static STUB int i2c_reads;
static STUB int i2c_writes;
static STUB int read_fail_after = -1;  /* reads that succeed before one fails */
static STUB int write_fail_after = -1; /* likewise for writes */
static STUB int last_write_reg = -1;
static STUB size_t last_write_len;
/* Runs during the next read, as something landing mid-transaction would. */
static STUB void (*on_read)(void);

static STUB bool countdown_fails(int *after) {
    if (*after == 0) {
        *after = -1;
        return true;
    }
    if (*after > 0) {
        (*after)--;
    }
    return false;
}
static STUB int i2c_burst_read_dt(const struct i2c_dt_spec *spec, uint8_t reg, uint8_t *buf,
                                  size_t len) {
    assert(spec->bus != NULL && (size_t)reg * 2 + len <= sizeof(regs));
    if (on_read != NULL) {
        void (*hook)(void) = on_read;
        on_read = NULL;
        hook();
    }
    if (countdown_fails(&read_fail_after)) {
        return -EIO;
    }
    memcpy(buf, REG(reg), len);
    i2c_reads++;
    return 0;
}
static STUB int i2c_burst_write_dt(const struct i2c_dt_spec *spec, uint8_t reg,
                                   const uint8_t *buf, size_t len) {
    assert(spec->bus != NULL && (size_t)reg * 2 + len <= sizeof(regs));
    if (countdown_fails(&write_fail_after)) {
        return -EIO;
    }
    memcpy(REG(reg), buf, len);
    i2c_writes++;
    last_write_reg = reg;
    last_write_len = len;
    return 0;
}

/* ----- GPIO: the RDY line ----- */
static STUB int irq_mode = -1;
static STUB int irq_configures;
static STUB int irq_arms;
static STUB int irq_configure_result;
static STUB int irq_fail_after = -1;
static STUB int rdy_level;
static STUB int rdy_raw_result;
static STUB int rdy_result;
static STUB int pin_configure_result;
static STUB int add_callback_result;
static STUB struct gpio_callback *added_callback;
/* Runs as the interrupt is armed, as an already-active level would. */
static STUB void (*on_arm)(void);
/* Runs as the interrupt is masked, as a suspend landing then would. */
static STUB void (*on_mask)(void);
/* An edge arriving while a failed mask is in progress. */
static STUB void (*on_irq_failure)(void);

/* Every device whose readiness was asked, in order. */
static STUB const struct device *ready_checked[8];
static STUB int ready_checks;
static STUB bool device_is_ready(const struct device *dev) {
    ready_checked[ready_checks++ % 8] = dev;
    return dev->ready;
}
static STUB int gpio_pin_configure_dt(const struct gpio_dt_spec *spec, int flags) {
    assert(flags == GPIO_INPUT && spec->port != NULL);
    return pin_configure_result;
}
static STUB void gpio_init_callback(struct gpio_callback *cb, gpio_callback_handler_t handler,
                                    gpio_port_pins_t mask) {
    cb->handler = handler;
    cb->pin_mask = mask;
}
static STUB int gpio_add_callback(const struct device *port, struct gpio_callback *cb) {
    ARG_UNUSED(port);
    if (add_callback_result == 0) {
        added_callback = cb;
    }
    return add_callback_result;
}
static STUB int gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *spec, int mode) {
    ARG_UNUSED(spec);
    if (countdown_fails(&irq_fail_after) || irq_configure_result < 0) {
        if (on_irq_failure != NULL) {
            void (*hook)(void) = on_irq_failure;
            on_irq_failure = NULL;
            hook();
        }
        return irq_configure_result < 0 ? irq_configure_result : -EIO;
    }
    irq_mode = mode;
    irq_configures++;
    if (mode == GPIO_INT_EDGE_TO_ACTIVE) {
        irq_arms++;
    }
    void (**hook)(void) = mode == GPIO_INT_EDGE_TO_ACTIVE ? &on_arm : &on_mask;
    if (*hook != NULL) {
        void (*run_hook)(void) = *hook;
        *hook = NULL;
        run_hook();
    }
    return 0;
}
static STUB int gpio_pin_get_raw(const struct device *port, uint8_t pin) {
    ARG_UNUSED(port);
    ARG_UNUSED(pin);
    return rdy_raw_result < 0 ? rdy_raw_result : !rdy_level;
}
static STUB int gpio_pin_get_dt(const struct gpio_dt_spec *spec) {
    ARG_UNUSED(spec);
    return rdy_result < 0 ? rdy_result : rdy_level;
}

/* ----- Input ----- */
struct input_record { uint16_t type; uint16_t code; int32_t value; bool sync; };
#define INPUT_KEY 1
#define INPUT_ABS 3
#define INPUT_REL 2
static STUB struct input_record inputs[256];
static STUB size_t input_count;
static STUB int input_fail_after = -1;
static STUB bool input_fail_all;
static STUB int input_failures;

static STUB int record_input(uint16_t type, uint16_t code, int32_t value, bool sync,
                             k_timeout_t timeout) {
    assert(timeout.ms == -1); /* always K_FOREVER on the private queue */
    if (input_fail_all) {
        /* The first failure differs from the rest, so they can be told apart. */
        return input_failures++ == 0 ? -EAGAIN : -EIO;
    }
    if (countdown_fails(&input_fail_after)) {
        return -EAGAIN;
    }
    assert(input_count < ARRAY_SIZE(inputs));
    inputs[input_count++] = (struct input_record){type, code, value, sync};
    return 0;
}
static STUB int input_report_key(const struct device *dev, uint16_t code, int32_t value,
                                 bool sync, k_timeout_t timeout) {
    ARG_UNUSED(dev);
    return record_input(INPUT_KEY, code, value, sync, timeout);
}
static STUB int input_report_abs(const struct device *dev, uint16_t code, int32_t value,
                                 bool sync, k_timeout_t timeout) {
    ARG_UNUSED(dev);
    return record_input(INPUT_ABS, code, value, sync, timeout);
}
static STUB int input_report_rel(const struct device *dev, uint16_t code, int32_t value,
                                 bool sync, k_timeout_t timeout) {
    ARG_UNUSED(dev);
    return record_input(INPUT_REL, code, value, sync, timeout);
}
