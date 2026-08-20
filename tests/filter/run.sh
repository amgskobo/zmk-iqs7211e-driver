#!/bin/sh
#
# Copyright (c) 2026 @amgskobo
#
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
binary="${TMPDIR:-/tmp}/iqs7211e-filter-test"

cc -std=c11 -Wall -Wextra -Werror \
    -I"$repo_dir/src" \
    "$repo_dir/tests/filter/test_filter.c" \
    "$repo_dir/src/iqs7211e_filter.c" \
    -o "$binary"
"$binary"
