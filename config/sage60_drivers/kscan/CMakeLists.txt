# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_library_named(zmk__sage60_drivers__kscan_ec)
zephyr_library_include_directories(${CMAKE_SOURCE_DIR}/include)

zephyr_library_sources(kscan_ec_matrix.c)
