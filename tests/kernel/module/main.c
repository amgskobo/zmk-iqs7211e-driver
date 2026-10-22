/* Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */
/* Compile the complete owner source once. No sensor instance or bus model:
 * this fixture checks real queue cancellation and PM call context only. */
#include <stdlib.h>
#include <string.h>
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

static void tests(void *a, void *b, void *c) {
    ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
    (void)iqs7211e_init;
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
