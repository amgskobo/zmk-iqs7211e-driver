# SPDX-License-Identifier: MIT
target_sources(app PRIVATE main.c ../../../src/iqs7211e_filter.c)
# The HID report the real listener would hand to a transport. The core's own
# function name differs between the supported cores.
zephyr_ld_options(-Wl,--wrap=zmk_endpoints_send_mouse_report)
zephyr_ld_options(-Wl,--wrap=zmk_endpoint_send_mouse_report)
