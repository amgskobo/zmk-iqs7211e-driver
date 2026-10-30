/* SPDX-License-Identifier: MIT */
#pragma once

#include <stdint.h>
#include <zephyr/sys_clock.h>

/* Verification intervals are uint16 milliseconds. Ordinary clock rates fit
 * the whole ceiling numerator in uint32. Faster clocks split the fraction
 * before multiplication; the final result retains the kernel tick ABI. */
static inline k_ticks_t iqs7211e_timeout_ticks(uint16_t ms, uint32_t hz) {
    if (hz <= (UINT32_MAX - 999U) / UINT16_MAX) {
        return ((uint32_t)ms * hz + 999U) / 1000U;
    }
    const uint32_t seconds = ms / 1000U;
    const uint32_t remainder = ms % 1000U;
    const uint32_t fraction = remainder * (hz / 1000U) +
                              (remainder * (hz % 1000U) + 999U) / 1000U;
    return (k_ticks_t)seconds * hz + fraction;
}
