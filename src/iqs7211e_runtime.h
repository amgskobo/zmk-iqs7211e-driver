/*
 * Copyright (c) 2026 @amgskobo
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_INPUT_IQS7211E_RUNTIME_H_
#define ZEPHYR_DRIVERS_INPUT_IQS7211E_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

/* INFO_FLAGS byte 0, bit 7: the sensor completed a power-on reset. */
#define IQS7211E_RUNTIME_SHOW_RESET_MASK UINT8_C(0x80)

static inline bool iqs7211e_runtime_show_reset(uint8_t info_flags_0)
{
    return (info_flags_0 & IQS7211E_RUNTIME_SHOW_RESET_MASK) != 0U;
}

/*
 * ZMK maps INPUT_BTN_TOUCH to mouse button 0. Absolute-coordinate processors
 * need the contact edge and can explicitly consume it, but direct relative
 * reporting must not turn every movement into a left-button drag.
 */
static inline bool iqs7211e_runtime_reports_touch_state(bool report_abs)
{
    return report_abs;
}

#endif /* ZEPHYR_DRIVERS_INPUT_IQS7211E_RUNTIME_H_ */
