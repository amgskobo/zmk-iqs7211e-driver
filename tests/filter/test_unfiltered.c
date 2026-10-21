/* Copyright (c) 2026 amgskobo
 * SPDX-License-Identifier: MIT
 */

/* Share validity/reset-policy assertions; retain every filtered trace in its
 * original binary. The bypass binary has independent immediate-output traces. */
#define main filtered_test_main
#include "test_filter.c"
#undef main

int main(void)
{
    struct iqs7211e_axis_filter x, y;
    iqs7211e_axis_filter_reset(&x, 500);
    iqs7211e_axis_filter_reset(&y, 900);
    const uint16_t samples[] = {501, 500, 600, 0, 1024, 100, 99, 100};
    for (size_t i = 0; i < sizeof(samples) / sizeof(samples[0]); i++)
    {
        struct iqs7211e_axis_filter_result r =
            iqs7211e_axis_filter_apply(&x, samples[i], 8);
        CHECK(r.gated == samples[i] && r.filtered == samples[i]);
        CHECK(y.gated == 900 && y.median_prev_1 == 900 && y.median_prev_2 == 900);
    }
    iqs7211e_axis_filter_reset(&x, 700);
    CHECK(iqs7211e_axis_filter_apply(&x, 701, 1024).filtered == 701);
    CHECK(iqs7211e_axis_filter_apply(&y, 899, 0).filtered == 899);
    const int16_t moves[][3] = {{501, 500, 0}, {500, 501, 1}, {0, 1024, 5},
                                {1024, 0, -5}, {500, 500, 20}, {500, 500, -20}};
    const int16_t expected[] = {1, -1, -1024, 1024, 0, 0};
    for (size_t i = 0; i < sizeof(moves) / sizeof(moves[0]); i++)
    {
        struct iqs7211e_relative_axis_result r =
            iqs7211e_relative_axis_apply(moves[i][0], moves[i][1], moves[i][2]);
        CHECK(r.delta == expected[i] && r.smoothed == expected[i]);
    }
    test_coordinate_validity();
    test_touch_state_reporting_policy();
    test_runtime_show_reset_detection();
    puts("iqs7211e unfiltered boundaries, immediate/reversed/stopped output and independent state passed");
    return 0;
}
