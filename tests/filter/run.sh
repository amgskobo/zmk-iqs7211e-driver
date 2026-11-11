#!/bin/sh
#
# Copyright (c) 2026 amgskobo
#
# SPDX-License-Identifier: MIT

set -eu

mode="${ZMK_TEST_MODE:-quick}"
case "$mode" in
    quick|full) ;;
    *) echo "ZMK_TEST_MODE must be quick or full" >&2; exit 2 ;;
esac
export ZMK_TEST_MODE="$mode"

repo_dir=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
build_dir=$(mktemp -d "${TMPDIR:-/tmp}/iqs7211e-filter-test.XXXXXX")
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM

variants="sanitized"
[ "$mode" != full ] || variants="optimized sanitized coverage"
for variant in $variants; do
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
    cc -std=c11 -Wall -Wextra -Werror "$@" -I"$repo_dir/src" \
        "$repo_dir/tests/filter/test_oracle.c" "$repo_dir/src/iqs7211e_filter.c" \
        -o "$build_dir/oracle-$variant"
    ASAN_OPTIONS=detect_leaks=0 "$build_dir/oracle-$variant"
done

# The opt-in bypass has its own expected trace; removing the compiler define
# makes its first one-unit movement fail against the normal deadband/median.
for variant in $variants; do
    if [ "$variant" = sanitized ]; then
        set -- -O1 -g -fno-omit-frame-pointer -fsanitize=address,undefined -fno-sanitize-recover=all
    elif [ "$variant" = coverage ]; then
        set -- -O0 --coverage
    else
        set -- -O2
    fi
    cc -std=c11 -Wall -Wextra -Werror "$@" -DCONFIG_IQS7211E_FILTER_BYPASS=1 \
        -I"$repo_dir/src" "$repo_dir/tests/filter/test_unfiltered.c" \
        "$repo_dir/src/iqs7211e_filter.c" -o "$build_dir/unfiltered-$variant"
    ASAN_OPTIONS=detect_leaks=0 "$build_dir/unfiltered-$variant"
    cc -std=c11 -Wall -Wextra -Werror "$@" -DCONFIG_IQS7211E_FILTER_BYPASS=1 \
        -I"$repo_dir/src" "$repo_dir/tests/filter/test_oracle.c" \
        "$repo_dir/src/iqs7211e_filter.c" -o "$build_dir/oracle-unfiltered-$variant"
    ASAN_OPTIONS=detect_leaks=0 "$build_dir/oracle-unfiltered-$variant"
done

# Independent oracle control: lose the oldest median input in an isolated copy.
python3 - "$repo_dir/src/iqs7211e_filter.c" "$build_dir/lost-history.c" <<'PY'
import pathlib, sys
source = pathlib.Path(sys.argv[1]).read_text()
before = '.filtered = median3(state->gated, previous_gated, state->median_prev_2),'
assert source.count(before) == 1
pathlib.Path(sys.argv[2]).write_text(source.replace(
    before, '.filtered = median3(state->gated, previous_gated, previous_gated),'))
PY
cc -std=c11 -Wall -Wextra -Werror -O2 -I"$repo_dir/src" \
    "$repo_dir/tests/filter/test_oracle.c" "$build_dir/lost-history.c" \
    -o "$build_dir/oracle-negative"
control_exit=0
"$build_dir/oracle-negative" >"$build_dir/oracle-negative.log" 2>&1 || control_exit=$?
[ "$control_exit" -eq 1 ]
grep -Eq '^FILTER ORACLE FAIL line=[0-9]+: result.filtered == window\[1\]$' \
    "$build_dir/oracle-negative.log"
cat "$build_dir/oracle-negative.log"
echo "PASS: lost-median-history compiled and rejected by independent sorted-window oracle (exit 1)"

# Negative control: the bypass trace must reject the unchanged filtered path.
cc -std=c11 -Wall -Wextra -Werror -O2 -I"$repo_dir/src" \
    "$repo_dir/tests/filter/test_unfiltered.c" "$repo_dir/src/iqs7211e_filter.c" \
    -o "$build_dir/unfiltered-negative"
if "$build_dir/unfiltered-negative" >"$build_dir/negative.log" 2>&1; then
    echo "Unfiltered negative control unexpectedly passed" >&2
    exit 1
fi
grep -Fq 'r.gated == samples[i] && r.filtered == samples[i]' "$build_dir/negative.log"
cat "$build_dir/negative.log"
echo "PASS: normal filtered implementation rejected by immediate-output trace"

if [ "$mode" = full ]; then
coverage=$(cd "$build_dir" && gcov -b -c \
    -o "$build_dir/coverage-iqs7211e_filter.gcno" "$repo_dir/src/iqs7211e_filter.c")
printf '%s\n' "$coverage"
printf '%s\n' "$coverage" | grep -Fq 'Lines executed:100.00%'
printf '%s\n' "$coverage" | grep -Fq 'Taken at least once:100.00%'
coverage=$(cd "$build_dir" && gcov -b -c \
    -o "$build_dir/unfiltered-coverage-iqs7211e_filter.gcno" "$repo_dir/src/iqs7211e_filter.c")
printf '%s\n' "$coverage"
printf '%s\n' "$coverage" | grep -Fq 'Lines executed:100.00%'
printf '%s\n' "$coverage" | grep -Fq 'Taken at least once:100.00%'
fi
