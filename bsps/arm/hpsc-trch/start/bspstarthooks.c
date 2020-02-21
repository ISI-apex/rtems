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

#include <bsp.h>
#include <bsp/start.h>
#include <bsp/fatal.h>
#include <bsp/linker-symbols.h>
#include "../../shared/cache/cacheimpl.h"

#include <bsp/hwinfo.h>
#include <bsp/irq.h>
#include <bsp/hpsc-arch.h>
#include <bsp/fdt.h>
#include <libfdt.h>

static int gcounter = 2;
static void cusleep(int i)
{
  gcounter = 2;
  while (i > 0) {
    i--;
    gcounter*=gcounter;
  }
}

static void cmsleep(int i)
{
  while (i > 0) {
    cusleep(1000000);
    i--;
  }
}

static void csleep(int i)
{
  while (i > 0) {
    cmsleep(250);
    i--;
  }
}

#include <libchip/ns16550.h>
extern ns16550_context hpsc_trch_uart_context_0;
void set_uart_params(void)
{
  const void *fdt = hpsc_arch_bin;
  int status;
  uint32_t size;
  int32_t size_variance = 4;
  int32_t size_diff;
  int root;
  int nic4;
  int lsio_uart_0;
  fdt32_t *uart_0_reg;
  fdt32_t *uart_0_interrupt_map;
  int len;
  uint32_t uart_0_address;

  status = fdt_check_header(fdt);
  if (status != 0) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }

  size = fdt_totalsize(fdt);
  /* size is the self-reported size of the FDT blob and hpsc_arch_bin_size
   * is the size of the blob plus any necessary padding */
  size_diff = hpsc_arch_bin_size - size;
  /* the size of the blob plus padding must always be >= the size of the
   * FDT self reported size */
  if (size_diff > size_variance || size_diff < 0) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }

  root = fdt_path_offset(fdt, "/");
  if (root < 0) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }

  nic4 = fdt_subnode_offset(fdt, root, "nic4");
  if (nic4 < 0) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }

  /* Searching for labels doesn't work because they don't get mapped in the device tree blob
   * Searching for serial@<address> requires prior knowledge of the address
   * Assume the 1st serial entry in nic4 is what is desired
   * There is no way to search for the nth entry
   * Find the first serial entry
   */
  lsio_uart_0 = fdt_subnode_offset(fdt, nic4, "serial");
  if (lsio_uart_0 < 0) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }

  /* Ensure that the node is also a serial node */
  if (strncmp("serial", fdt_get_name(fdt, lsio_uart_0, NULL), strlen("serial")) != 0) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }

  uart_0_reg = (fdt32_t *) fdt_getprop(fdt, lsio_uart_0, "reg", &len);
  if (!uart_0_reg || len != 16) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }
  uart_0_address = fdt32_to_cpu(uart_0_reg[0]);
  hpsc_trch_uart_context_0.port = uart_0_address;
  hpsc_trch_uart_context_0.port = 0x30000000;

  uart_0_interrupt_map = (fdt32_t *) fdt_getprop(fdt, lsio_uart_0, "interrupt-map", &len);
  if (!uart_0_interrupt_map || len != 20) {
    bsp_fatal(HPSC_TRCH_FATAL_UART_ADDRESS);
  }
#define GIC_EDGE_RISE 1
#define GIC_EDGE_FALL 2
#define GIC_EDGE_BOTH 3
#define GIC_LVL_HI    4
#define GIC_LVL_LO    8
  /*uart_int_trigger = fdt32_to_cpu(uart_0_interrupt_map[6]);*/
  hpsc_trch_uart_context_0.irq = fdt32_to_cpu(uart_0_interrupt_map[4]);
}

BSP_START_TEXT_SECTION void bsp_start_hook_0( void )
{
  csleep(5);
  set_uart_params();
}

BSP_START_TEXT_SECTION void bsp_start_hook_1( void )
{
  bsp_start_copy_sections();
  bsp_start_clear_bss();
}
