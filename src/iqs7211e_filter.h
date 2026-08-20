/*
 * Copyright (c) 2026 @amgskobo
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

struct iqs7211e_axis_filter
{
    uint16_t gated;
    uint16_t median_prev_1;
    uint16_t median_prev_2;
};

struct iqs7211e_axis_filter_result
{
    uint16_t gated;
    uint16_t filtered;
};

struct iqs7211e_relative_axis_result
{
    int16_t delta;
    int16_t smoothed;
};

bool iqs7211e_coordinate_sample_valid(uint8_t fingers, uint16_t x, uint16_t y,
                                      uint16_t strength, uint16_t area);

void iqs7211e_axis_filter_reset(struct iqs7211e_axis_filter *state, uint16_t sample);
struct iqs7211e_axis_filter_result
iqs7211e_axis_filter_apply(struct iqs7211e_axis_filter *state, uint16_t sample,
                          uint16_t deadband);

/* Convert one canonical filtered coordinate into the relative report values. */
struct iqs7211e_relative_axis_result
iqs7211e_relative_axis_apply(int16_t current, int16_t previous,
                             int16_t previous_delta);
