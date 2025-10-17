# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rab7_sensorfusion)

target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/sgp41.c)
target_sources(app PRIVATE src/dps368.c)
target_sources(app PRIVATE src/bmi323.c)
target_sources(app PRIVATE src/sht41.c)

target_sources(app PRIVATE src/bmp585/bmp_driver.c)
target_sources(app PRIVATE src/bmp585/bmp5.c)
target_sources(app PRIVATE src/bmp585/bmp585_app.c)

target_sources(app PRIVATE src/bme690/bme_driver.c)
target_sources(app PRIVATE src/bme690/bme69x.c)
target_sources(app PRIVATE src/bme690/bme690_app.c)

target_sources(app PRIVATE src/sensirion_gas_index_algorithm.c)