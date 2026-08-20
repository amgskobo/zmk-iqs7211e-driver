/* SPDX-License-Identifier: MIT */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "iqs7211e_filter.h"

#define TEST_DEADBAND 8

#define CHECK(condition)                                                                    \
    do                                                                                      \
    {                                                                                       \
        if (!(condition))                                                                   \
        {                                                                                   \
            fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            exit(1);                                                                        \
        }                                                                                   \
    } while (0)

static void test_reset_and_deadband(void)
{
    struct iqs7211e_axis_filter state;
    iqs7211e_axis_filter_reset(&state, 500);

    CHECK(state.gated == 500);
    CHECK(state.median_prev_1 == 500);
    CHECK(state.median_prev_2 == 500);

    for (uint16_t sample = 501; sample <= 508; sample++)
    {
        struct iqs7211e_axis_filter_result result =
            iqs7211e_axis_filter_apply(&state, sample, TEST_DEADBAND);
        CHECK(result.gated == 500);
        CHECK(result.filtered == 500);
    }

    struct iqs7211e_axis_filter_result result =
        iqs7211e_axis_filter_apply(&state, 509, TEST_DEADBAND);
    CHECK(result.gated == 501);
    CHECK(result.filtered == 500);

    result = iqs7211e_axis_filter_apply(&state, 510, TEST_DEADBAND);
    CHECK(result.gated == 502);
    CHECK(result.filtered == 501);
}

static void test_negative_direction_is_symmetric(void)
{
    struct iqs7211e_axis_filter state;
    iqs7211e_axis_filter_reset(&state, 500);

    struct iqs7211e_axis_filter_result result =
        iqs7211e_axis_filter_apply(&state, 491, TEST_DEADBAND);
    CHECK(result.gated == 499);
    CHECK(result.filtered == 500);

    result = iqs7211e_axis_filter_apply(&state, 490, TEST_DEADBAND);
    CHECK(result.gated == 498);
    CHECK(result.filtered == 499);
}

static void test_slow_motion_accumulates(void)
{
    struct iqs7211e_axis_filter state;
    iqs7211e_axis_filter_reset(&state, 100);
    struct iqs7211e_axis_filter_result result = {.gated = 100, .filtered = 100};

    for (uint16_t sample = 101; sample <= 140; sample++)
    {
        result = iqs7211e_axis_filter_apply(&state, sample, TEST_DEADBAND);
    }

    CHECK(result.gated == 132);
    CHECK(result.filtered == 131);
}

static void test_single_sample_spike_is_bounded(void)
{
    struct iqs7211e_axis_filter state;
    iqs7211e_axis_filter_reset(&state, 500);

    struct iqs7211e_axis_filter_result result =
        iqs7211e_axis_filter_apply(&state, 600, TEST_DEADBAND);
    CHECK(result.filtered == 500);

    result = iqs7211e_axis_filter_apply(&state, 500, TEST_DEADBAND);
    CHECK(result.filtered == 508);
    CHECK(result.filtered - 500 <= TEST_DEADBAND);
}

static void test_zero_deadband_keeps_median_filter(void)
{
    struct iqs7211e_axis_filter state;
    iqs7211e_axis_filter_reset(&state, 500);

    struct iqs7211e_axis_filter_result result =
        iqs7211e_axis_filter_apply(&state, 600, 0);
    CHECK(result.gated == 600);
    CHECK(result.filtered == 500);

    result = iqs7211e_axis_filter_apply(&state, 600, 0);
    CHECK(result.filtered == 600);
}

struct parity_axis_state
{
    struct iqs7211e_axis_filter filter;
    uint8_t touch_count;
    int16_t previous_coordinate;
};

struct parity_axis_result
{
    int16_t coordinate;
    bool filter_updated;
};

static struct parity_axis_result
apply_shared_axis_pipeline(struct parity_axis_state *state, uint16_t sample,
                           bool sample_valid)
{
    bool filter_updated = sample_valid;
    int16_t coordinate;

    if (state->touch_count == 0)
    {
        CHECK(sample_valid);
        iqs7211e_axis_filter_reset(&state->filter, sample);
        coordinate = sample;
    }
    else if (!filter_updated)
    {
        coordinate = state->previous_coordinate;
    }
    else
    {
        coordinate = iqs7211e_axis_filter_apply(&state->filter, sample, TEST_DEADBAND).filtered;
    }

    state->previous_coordinate = coordinate;
    state->touch_count++;

    return (struct parity_axis_result){
        .coordinate = coordinate,
        .filter_updated = filter_updated,
    };
}

static void test_absolute_relative_parity(void)
{
    static const struct
    {
        uint16_t sample;
        bool valid;
        bool expected_filter_update;
    } frames[] = {
        {500, true, true},  /* First coordinate always passes. */
        {503, true, true},
        {510, true, true},
        {518, true, true},
        {531, false, false}, /* Invalid frame: hold, no filter drift. */
        {540, true, true},
        {520, true, true},
        {480, true, true},
        {470, true, true},
    };
    struct parity_axis_state absolute_state = {0};
    struct parity_axis_state relative_state = {0};

    int16_t previous_absolute = frames[0].sample;
    int16_t previous_relative = frames[0].sample;
    int16_t previous_absolute_delta = 0;
    int16_t previous_relative_delta = 0;

    for (size_t i = 0; i < sizeof(frames) / sizeof(frames[0]); i++)
    {
        struct parity_axis_result absolute = apply_shared_axis_pipeline(
            &absolute_state, frames[i].sample, frames[i].valid);
        struct parity_axis_result relative = apply_shared_axis_pipeline(
            &relative_state, frames[i].sample, frames[i].valid);

        CHECK(absolute.coordinate == relative.coordinate);
        CHECK(absolute.filter_updated == frames[i].expected_filter_update);
        CHECK(relative.filter_updated == frames[i].expected_filter_update);
        CHECK(absolute_state.filter.gated == relative_state.filter.gated);
        CHECK(absolute_state.filter.median_prev_1 == relative_state.filter.median_prev_1);
        CHECK(absolute_state.filter.median_prev_2 == relative_state.filter.median_prev_2);

        if (i == 0)
        {
            previous_absolute = absolute.coordinate;
            previous_relative = relative.coordinate;
            continue;
        }

        if (!frames[i].expected_filter_update)
        {
            CHECK(absolute.coordinate == previous_absolute);
        }

        int16_t expected_delta =
            (int16_t)((int32_t)absolute.coordinate - previous_absolute);
        int16_t expected_smoothed =
            (int16_t)(((int32_t)expected_delta + previous_absolute_delta) / 2);
        struct iqs7211e_relative_axis_result reported =
            iqs7211e_relative_axis_apply(relative.coordinate, previous_relative,
                                         previous_relative_delta);

        CHECK(reported.delta == expected_delta);
        CHECK(reported.smoothed == expected_smoothed);

        previous_absolute = absolute.coordinate;
        previous_relative = relative.coordinate;
        previous_absolute_delta = expected_delta;
        previous_relative_delta = reported.delta;
    }

    iqs7211e_axis_filter_reset(&relative_state.filter, 900);
    CHECK(relative_state.filter.gated == 900);
    CHECK(relative_state.filter.median_prev_1 == 900);
    CHECK(relative_state.filter.median_prev_2 == 900);
}

static void test_stationary_samples_decay_relative_velocity(void)
{
    /* A stationary report must drain the direct REL path's delta history. */
    struct iqs7211e_relative_axis_result first =
        iqs7211e_relative_axis_apply(500, 500, 20);
    CHECK(first.delta == 0);
    CHECK(first.smoothed == 10);

    struct iqs7211e_relative_axis_result second =
        iqs7211e_relative_axis_apply(500, 500, first.delta);
    CHECK(second.delta == 0);
    CHECK(second.smoothed == 0);
}

static void test_coordinate_validity(void)
{
    CHECK(iqs7211e_coordinate_sample_valid(1, 500, 500, 300, 2));
    CHECK(!iqs7211e_coordinate_sample_valid(0, 500, 500, 300, 2));
    CHECK(!iqs7211e_coordinate_sample_valid(1, UINT16_MAX, 500, 300, 2));
    CHECK(!iqs7211e_coordinate_sample_valid(1, 500, UINT16_MAX, 300, 2));
    CHECK(!iqs7211e_coordinate_sample_valid(1, 500, 500, 0, 2));
    CHECK(!iqs7211e_coordinate_sample_valid(1, 500, 500, 300, 0));
}

int main(void)
{
    test_reset_and_deadband();
    test_negative_direction_is_symmetric();
    test_slow_motion_accumulates();
    test_single_sample_spike_is_bounded();
    test_zero_deadband_keeps_median_filter();
    test_absolute_relative_parity();
    test_stationary_samples_decay_relative_velocity();
    test_coordinate_validity();
    puts("iqs7211e filter tests passed");
    return 0;
}
