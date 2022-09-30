# SPDX-License-Identifier: Apache-2.0

zephyr_library()

string(TOUPPER ${CONFIG_SOC} TI_CPU)

zephyr_compile_definitions(
  -D${TI_CPU}
  )

zephyr_library_sources(
  soc.c
  )

zephyr_library_include_directories(
  ${ZEPHYR_BASE}/kernel/include
  ${ZEPHYR_BASE}/arch/arm/include
  )
