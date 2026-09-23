#!/bin/sh
#
# Copyright (c) 2026 amgskobo
#
# SPDX-License-Identifier: MIT

set -eu

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/iqs7211e-filter-test.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

for variant in optimized sanitized coverage; do
    if [ "$variant" = sanitized ]; then
        set -- -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=all
    elif [ "$variant" = coverage ]; then
        set -- -O0 --coverage
    else
        set -- -O2
    fi
    cc -std=c11 -Wall -Wextra -Werror "$@" \
        -I"$repo_dir/src" \
        "$repo_dir/tests/filter/test_filter.c" \
        "$repo_dir/src/iqs7211e_filter.c" \
        -o "$build_dir/$variant"
    ASAN_OPTIONS=detect_leaks=0 "$build_dir/$variant"
done

coverage=$(cd "$build_dir" && gcov -b -c \
    -o "$build_dir/coverage-iqs7211e_filter.gcno" "$repo_dir/src/iqs7211e_filter.c")
printf '%s\n' "$coverage"
printf '%s\n' "$coverage" | grep -Fq 'Lines executed:100.00%'
printf '%s\n' "$coverage" | grep -Fq 'Taken at least once:100.00%'
