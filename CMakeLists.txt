  zephyr_library()

  zephyr_library_sources_ifdef(CONFIG_ZMK_IQS7211E src/iqs7211e.c)
