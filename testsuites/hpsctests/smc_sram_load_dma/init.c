/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (C) 2019 On-Line Applications Research Corporation (OAR)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define CONFIGURE_INIT
#include "system.h"
#include "tmacros.h"
#include <stdio.h>
#include <bsp/sramfs.h>
#include <bsp/dma.h>

const char rtems_test_name[] = "SMC SRAM Load w/ DMA";

extern char __rtps_ddr_low_0_start__[];
rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  TEST_BEGIN();

  /* Initialize DMA controller */
  DMA_Config_t dma_config;
  status = dma_init(&dma_config, BSP_DMA_BASE);
  directive_failed(status, "DMA Initialization");

  /* Initialize SMC controller */
  SRAMFS_Config_t sramfs_config;
  status = sramfs_init(&sramfs_config, SMC_LSIO_SRAM_BASE, &dma_config);

  /* Load syscfg */
  uint32_t *load_addr = (uint32_t *)__rtps_ddr_low_0_start__;
  status = sramfs_load_addr(&sramfs_config, "syscfg", load_addr);
  directive_failed(status, "SMC SRAM \"syscfg\" Load");

  /* Check first two bytes of syscfg */
  fatal_int_service_status(*load_addr & 0xffff, 0x94c, "SMC SRAM Load Address");

  /* Uninitialize SMC controller */
  status = sramfs_uninit(&sramfs_config);

  /* Uninitialize DMA controller */
  status = dma_uninit(&dma_config);
  directive_failed(status, "DMA Uninitialization");
  TEST_END();
  rtems_test_exit( 0 );
}
