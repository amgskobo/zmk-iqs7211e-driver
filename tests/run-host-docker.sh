#!/usr/bin/env bash
# Copyright (c) 2026 amgskobo
# SPDX-License-Identifier: MIT
set -euo pipefail

export MSYS_NO_PATHCONV=1
repo_root="$(CDPATH='' cd -- "$(dirname -- "$0")/.." && pwd)"
image="${ZMK_TEST_IMAGE:-zmkfirmware/zmk-build-arm:stable}"

docker run --rm --env PYTHONDONTWRITEBYTECODE=1 --env ZMK_TEST_MODE="${ZMK_TEST_MODE:-quick}" --volume "$repo_root:/src:ro" "$image" \
    /bin/bash -lc 'cd /src && python3 -m unittest discover -s tests -p test_retirement.py -v && python3 -m unittest discover -s tests/integration -p "test_*.py" -v && bash tests/filter/run.sh && python3 tests/runtime/run.py'
