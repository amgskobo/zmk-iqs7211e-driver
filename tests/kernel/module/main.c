/* Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */
/* Compile the complete owner source once. No sensor instance or bus model:
 * this fixture checks real queue cancellation and PM call context only. */
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/gpio/gpio_emul.h>
#include "../../../src/iqs7211e.c"

static struct iqs7211e_data state;
static const struct iqs7211e_config config = {
    .irq_gpio = {.port = DEVICE_DT_GET(DT_NODELABEL(gpio0)), .pin = 0,
                 .dt_flags = GPIO_ACTIVE_LOW},
};
static const struct device sensor = {.name = "native-pm", .config = &config, .data = &state};
K_SEM_DEFINE(completed, 0, 1);
static int pm_result;
K_SEM_DEFINE(released, 0, 1);
static unsigned release_count;
static void request_suspend(struct k_work *work);
K_SEM_DEFINE(cancel_entered, 0, 1);
K_SEM_DEFINE(cancel_continue, 0, 1);
K_SEM_DEFINE(suspend_requested, 0, 1);
static unsigned cancel_exited;

static void hold_for_cancel(struct k_work *work) {
    ARG_UNUSED(work);
    k_sem_give(&cancel_entered);
    __ASSERT_NO_MSG(k_sem_take(&cancel_continue, K_FOREVER) == 0);
    cancel_exited++;
}

static void suspend_external(void *a, void *b, void *c) {
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    for (;;) {
        __ASSERT_NO_MSG(k_sem_take(&suspend_requested, K_FOREVER) == 0);
        pm_result = iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND);
        k_sem_give(&completed);
    }
}
K_THREAD_DEFINE(iqs_suspender, 2048, suspend_external, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);

/* Every synchronous cancellation must finish before the next wait starts.
 * Use real running handlers, not idle work or mocked cancellation results. */
static void verify_running_cancellations(void) {
    struct k_work_delayable *const delayed[] = {
        &state.touch_verify_work, &state.rdy_recheck_work,
        &state.boot_kick_work, NULL, &state.click_work,
    };
    const k_work_handler_t handlers[] = {
        iqs7211e_touch_verify_work_handler, iqs7211e_rdy_recheck_work_handler,
        iqs7211e_boot_kick_work_handler, NULL, request_suspend,
    };
    static struct k_work_sync fixture_sync;
    __ASSERT_NO_MSG(gpio_emul_input_set(config.irq_gpio.port, config.irq_gpio.pin, 1) == 0);
    for (unsigned i = 0; i < ARRAY_SIZE(delayed); ++i) {
        /* Resume's inactive-level recheck must finish before reinitialization. */
        k_work_cancel_delayable_sync(&state.rdy_recheck_work, &fixture_sync);
        struct k_work *target;
        if (delayed[i] != NULL) {
            k_work_init_delayable(delayed[i], hold_for_cancel);
            target = &delayed[i]->work;
            __ASSERT_NO_MSG(iqs7211e_reschedule_work(delayed[i], K_NO_WAIT) >= 0);
        } else {
            k_work_init(&state.work, hold_for_cancel);
            target = &state.work;
            __ASSERT_NO_MSG(iqs7211e_submit_work(target) >= 0);
        }
        __ASSERT_NO_MSG(k_sem_take(&cancel_entered, K_SECONDS(1)) == 0);
        state.init_state = IQS7211E_INIT_NONE; /* No physical bus in this fixture. */
        state.sensor_resume_attempts = 4;
        state.click_edges = 1;
        state.click_button = INPUT_BTN_0;
        k_sem_give(&suspend_requested);
        const int64_t deadline = k_uptime_get() + 1000;
        while (!(k_work_busy_get(target) & K_WORK_CANCELING) &&
               k_uptime_get() < deadline) {
            k_sleep(K_MSEC(1));
        }
        __ASSERT(k_work_busy_get(target) & K_WORK_CANCELING,
                 "External PM must wait for the running cancellation");
        __ASSERT(k_sem_count_get(&completed) == 0 && cancel_exited == i,
                 "Suspend must not finish before its running handler returns");
        if (delayed[i] == &state.rdy_recheck_work) {
            __ASSERT(state.sensor_resume_attempts == 4,
                     "Cancellation must retain worker-owned state until the handler returns");
        }
        k_sem_give(&cancel_continue);
        __ASSERT_NO_MSG(k_sem_take(&completed, K_SECONDS(1)) == 0);
        __ASSERT_NO_MSG(pm_result == 0 && cancel_exited == i + 1);
        __ASSERT_NO_MSG(k_work_busy_get(target) == 0);
        __ASSERT_NO_MSG(k_sem_take(&released, K_SECONDS(1)) == 0);
        __ASSERT(state.click_edges == 0 && release_count == i + 1,
                 "Each external suspend must finish exactly one owed release");
        if (delayed[i] != NULL) {
            k_work_init_delayable(delayed[i], handlers[i]);
        } else {
            k_work_init(&state.work, request_suspend);
        }
        /* Keep resume from starting sensor setup on the nonexistent bus. */
        state.init_state = IQS7211E_INIT_DONE;
        __ASSERT_NO_MSG(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    }
}

static void input_pm_request(struct input_event *event, void *user_data) {
    ARG_UNUSED(user_data);
    if (event->type == INPUT_EV_KEY && event->code == INPUT_BTN_0 && !event->value) {
        release_count++;
        k_sem_give(&released);
        return;
    }
    if (event->type != INPUT_EV_MSC || event->value != 1) {
        return;
    }
    /* The input consumer cannot make room while waiting for PM's private-queue
     * release. Fill the real queue so that cycle is deterministic. */
    for (unsigned i = 0; i < CONFIG_INPUT_QUEUE_MAX_MSGS; i++) {
        __ASSERT_NO_MSG(input_report(&sensor, INPUT_EV_MSC, INPUT_MSC_SCAN,
                                     2, true, K_NO_WAIT) == 0);
    }
    pm_result = iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND);
    k_sem_give(&completed);
}
INPUT_CALLBACK_DEFINE(&sensor, input_pm_request, NULL);

static void request_suspend(struct k_work *work) {
    ARG_UNUSED(work);
    pm_result = iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND);
    k_sem_give(&completed);
}

static void verify_timeout_conversion(void) {
    static const uint32_t rates[] = {1, 100, 1000, 32768, 65536, 65537, UINT32_MAX};
    for (unsigned h = 0; h < ARRAY_SIZE(rates); ++h) {
        const uint32_t hz = rates[h];
        for (uint32_t ms = 0; ms <= UINT16_MAX; ++ms) {
            const k_ticks_t expected = (k_ticks_t)(IS_ENABLED(CONFIG_TIMEOUT_64BIT) ?
                z_tmcvt_64(ms, 1000U, hz, true, true, false) :
                z_tmcvt_32(ms, 1000U, hz, true, true, false));
            __ASSERT_NO_MSG(iqs7211e_timeout_ticks((uint16_t)ms, hz) == expected);
        }
    }
}

static void tests(void *a, void *b, void *c) {
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    (void)iqs7211e_init;
    verify_timeout_conversion();
    const char *which = getenv("IQS_KERNEL_CASE");
    __ASSERT_NO_MSG(which != NULL);
    state.dev = &sensor;
    state.init_state = IQS7211E_INIT_NONE;
    iqs7211e_start_work_queue();
    k_work_init(&state.work, request_suspend);
    k_work_init(&state.pm_release_work, iqs7211e_pm_release_work_handler);
    k_work_init_delayable(&state.click_work, request_suspend);
    k_work_init_delayable(&state.touch_verify_work, iqs7211e_touch_verify_work_handler);
    k_work_init_delayable(&state.rdy_recheck_work, iqs7211e_rdy_recheck_work_handler);
    k_work_init_delayable(&state.boot_kick_work, iqs7211e_boot_kick_work_handler);
    __ASSERT_NO_MSG(gpio_pin_configure_dt(&config.irq_gpio, GPIO_INPUT) == 0);
    if (strcmp(which, "cancel") == 0) {
        verify_running_cancellations();
        printk("IQS kernel PM PASS case=cancel time=%lld\n", k_uptime_get());
        exit(0);
    }
    if (strcmp(which, "work") == 0) {
        __ASSERT_NO_MSG(iqs7211e_submit_work(&state.work) >= 0);
    } else if (strcmp(which, "click") == 0) {
        __ASSERT_NO_MSG(strcmp(which, "click") == 0);
        __ASSERT_NO_MSG(iqs7211e_reschedule_work(&state.click_work, K_NO_WAIT) >= 0);
    } else {
        __ASSERT_NO_MSG(strcmp(which, "input") == 0);
        state.click_edges = 1;
        state.click_button = INPUT_BTN_0;
        __ASSERT_NO_MSG(input_report(&sensor, INPUT_EV_MSC, INPUT_MSC_SCAN,
                                     1, true, K_FOREVER) == 0);
    }
    __ASSERT(k_sem_take(&completed, K_SECONDS(1)) == 0,
             "PM queue-context call failed to return");
    __ASSERT(pm_result == -EWOULDBLOCK, "PM producer/consumer queue must reject synchronous waits");
    __ASSERT_NO_MSG(atomic_get(&state.suspended) == 0);
    if (strcmp(which, "input") == 0) {
        __ASSERT(state.click_edges == 1 && release_count == 0,
                 "Rejected input-thread PM must preserve its owed release");
    }
    /* A rejected self-call must leave the queue usable for normal PM. */
    __ASSERT_NO_MSG(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_SUSPEND) == 0);
    __ASSERT_NO_MSG(atomic_get(&state.suspended) == 1);
    if (strcmp(which, "input") == 0) {
        __ASSERT(k_sem_take(&released, K_SECONDS(1)) == 0,
                 "External PM must deliver the preserved release");
        __ASSERT(state.click_edges == 0 && release_count == 1,
                 "External PM must complete the release exactly once");
    }
    __ASSERT_NO_MSG(iqs7211e_pm_action(&sensor, PM_DEVICE_ACTION_RESUME) == 0);
    __ASSERT_NO_MSG(atomic_get(&state.suspended) == 0);
    printk("IQS kernel PM PASS case=%s time=%lld\n", which, k_uptime_get());
    exit(0);
}
K_THREAD_DEFINE(iqs_tests, 4096, tests, NULL, NULL, NULL,
                K_LOWEST_APPLICATION_THREAD_PRIO, 0, 100);
