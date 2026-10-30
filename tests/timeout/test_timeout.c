/* SPDX-License-Identifier: MIT */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "iqs7211e_timeout.h"

#ifdef IQS_TIMEOUT_WIDTH_PROBE
k_ticks_t probe_timeout(uint16_t ms, uint32_t hz);
k_ticks_t probe_timeout(uint16_t ms, uint32_t hz) {
    return iqs7211e_timeout_ticks(ms, hz);
}
#else
int main(void) {
    static const uint32_t rates[] = {
        0, 1, 10, 100, 128, 1000, 1024, 32768, 65535, 65536, 65537,
        1000000, INT32_MAX, UINT32_MAX,
    };
    for (unsigned h = 0; h < sizeof(rates) / sizeof(rates[0]); ++h) {
        const uint32_t hz = rates[h];
        for (uint32_t ms = 0; ms <= UINT16_MAX; ++ms) {
            const k_ticks_t expected = (k_ticks_t)(((uint64_t)ms * hz + 999U) / 1000U);
            assert(iqs7211e_timeout_ticks((uint16_t)ms, hz) == expected);
        }
    }
    puts("IQS timeout all65536 waits at14 clock rates, exact ceil and kernel width: PASS");
    return 0;
}
#endif
