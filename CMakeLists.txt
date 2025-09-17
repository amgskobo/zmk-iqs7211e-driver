zephyr_library()

zephyr_library_sources_ifdef(CONFIG_IQS7211E src/iqs7211e.c)
zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)