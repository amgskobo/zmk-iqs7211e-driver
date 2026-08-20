/*
 * Copyright (c) 2026 @amgskobo
 *
 * SPDX-License-Identifier: MIT
 */

#include "iqs7211e_filter.h"

bool iqs7211e_coordinate_sample_valid(uint8_t fingers, uint16_t x, uint16_t y,
                                      uint16_t strength, uint16_t area)
{
    return fingers > 0U && x != UINT16_MAX && y != UINT16_MAX && strength > 0U && area > 0U;
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b)
    {
        uint16_t tmp = a;
        a = b;
        b = tmp;
    }
    if (b > c)
    {
        b = c;
    }

    return (a > b) ? a : b;
}

void iqs7211e_axis_filter_reset(struct iqs7211e_axis_filter *state, uint16_t sample)
{
    state->gated = sample;
    state->median_prev_1 = sample;
    state->median_prev_2 = sample;
}

struct iqs7211e_axis_filter_result
iqs7211e_axis_filter_apply(struct iqs7211e_axis_filter *state, uint16_t sample,
                          uint16_t deadband)
{
    int32_t delta = (int32_t)sample - state->gated;

    if (delta > deadband)
    {
        state->gated = sample - deadband;
    }
    else if (delta < -(int32_t)deadband)
    {
        state->gated = sample + deadband;
    }

    struct iqs7211e_axis_filter_result result = {
        .gated = state->gated,
        .filtered = median3(state->gated, state->median_prev_1, state->median_prev_2),
    };

    state->median_prev_2 = state->median_prev_1;
    state->median_prev_1 = state->gated;

    return result;
}

struct iqs7211e_relative_axis_result
iqs7211e_relative_axis_apply(int16_t current, int16_t previous,
                             int16_t previous_delta)
{
    int16_t delta = (int16_t)((int32_t)current - previous);

    return (struct iqs7211e_relative_axis_result){
        .delta = delta,
        .smoothed = (int16_t)(((int32_t)delta + previous_delta) / 2),
    };
}
