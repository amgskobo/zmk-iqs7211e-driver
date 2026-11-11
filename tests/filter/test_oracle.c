/* Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */

/* Independent specification: project the anchor onto the sample's deadband
 * interval, then sort an explicit three-sample window. No driver state fields
 * or implementation median helper participate in the expected result. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "iqs7211e_filter.h"

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "FILTER ORACLE FAIL line=%d: %s\n", __LINE__, #condition); \
    exit(1); \
} } while (0)

struct oracle {
    int32_t anchor;
    uint16_t window[3];
};

static int compare(const void *a, const void *b) {
    return (*(const uint16_t *)a > *(const uint16_t *)b) -
           (*(const uint16_t *)a < *(const uint16_t *)b);
}

static void reset(struct oracle *expected, struct iqs7211e_axis_filter *actual,
                  uint16_t seed) {
    expected->anchor = seed;
    for (unsigned i = 0; i < 3; ++i) expected->window[i] = seed;
    iqs7211e_axis_filter_reset(actual, seed);
}

static void step(struct oracle *expected, struct iqs7211e_axis_filter *actual,
                 uint16_t sample, uint16_t deadband) {
#if defined(CONFIG_IQS7211E_FILTER_BYPASS)
    expected->anchor = sample;
    for (unsigned i = 0; i < 3; ++i) expected->window[i] = sample;
#else
    const int32_t lower = (int32_t)sample - deadband;
    const int32_t upper = (int32_t)sample + deadband;
    if (expected->anchor < lower) expected->anchor = lower;
    if (expected->anchor > upper) expected->anchor = upper;
    expected->window[0] = expected->window[1];
    expected->window[1] = expected->window[2];
    expected->window[2] = expected->anchor;
#endif
    uint16_t window[3];
    for (unsigned i = 0; i < 3; ++i) window[i] = expected->window[i];
    qsort(window, 3, sizeof(window[0]), compare);
    const struct iqs7211e_axis_filter_result result =
        iqs7211e_axis_filter_apply(actual, sample, deadband);
    CHECK(result.gated == expected->anchor);
    CHECK(result.filtered == window[1]);
}

int main(void) {
    const uint16_t deadbands[] = {0, 1, 4, 8, 1024, UINT16_MAX};
    const uint16_t seeds[] = {0, 1, 4, 8, 511, 512, 1023, 1024, UINT16_MAX};
    struct oracle expected[2];
    struct iqs7211e_axis_filter actual[2];
    for (unsigned d = 0; d < sizeof(deadbands)/sizeof(deadbands[0]); ++d) {
        for (unsigned s = 0; s < sizeof(seeds)/sizeof(seeds[0]); ++s) {
            reset(&expected[0], &actual[0], seeds[s]);
            reset(&expected[1], &actual[1], seeds[(s + 1) % 9]);
            for (uint16_t sample = 0; sample <= 1024; ++sample) {
                step(&expected[0], &actual[0], sample, deadbands[d]);
                step(&expected[1], &actual[1], 1024 - sample, deadbands[d]);
                step(&expected[0], &actual[0], 1024 - sample, deadbands[d]);
            }
        }
    }
    uint32_t sequence = UINT32_C(0x7211e);
    for (unsigned i = 0; i < 65536; ++i) {
        sequence = sequence * UINT32_C(1664525) + UINT32_C(1013904223);
        const unsigned side = i % 2;
        if (i % 257 == 0) reset(&expected[side], &actual[side], sequence >> 16);
        step(&expected[side], &actual[side], sequence >> 16, deadbands[i % 6]);
    }
    puts("iqs7211e independent clamp/sorted-window oracle passed");
    return 0;
}
