# Copyright (c) 2025-2026 amgskobo
# SPDX-License-Identifier: MIT

if (CONFIG_IQS7211E)
    zephyr_library()
    zephyr_library_sources(src/iqs7211e.c)
    zephyr_library_sources(src/iqs7211e_filter.c)
    zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()
