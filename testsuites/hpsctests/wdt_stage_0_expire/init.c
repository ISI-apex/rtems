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
#include <bsp/hpsc-wdt.h>
#include <bsp/hwinfo.h>
#include <bsp/hpsc-irqs.dtsh>
#include <bsp/irq.h>

const char rtems_test_name[] = "HPSC WDT Stage 0 Expire";

/* this has to match choice in TRCH */
#define WDT_FREQ_HZ WDT_MIN_FREQ_HZ

#define MS_TO_WDT_CYCLES(ms) ((ms) * (WDT_FREQ_HZ / 1000))
#define WDT_CYCLES_TO_US(c) (1000000 * (c) / WDT_FREQ_HZ)
#define WDT_IRQ   ( TRCH_IRQ__WDT_TRCH_ST1 )
#define NUM_STAGES 2

static struct HPSC_WDT_Config wdt;
uint64_t timeouts[NUM_STAGES] = { 1000, 1000 }; /* timeouts ms */
volatile bool expired = false;

static void WDT_isr(void *arg)
{
  /* Clear the timeout condition in the watchdog timer */
  wdt_timeout_clear(&wdt, 0);
  printk("wdt test: expired\n");
  expired = true;
}

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  TEST_BEGIN();

  /* only the monitor can configure the timer */
  wdt_init_monitor(&wdt, "TRCHST1", WDT_TRCH_BASE, WDT_IRQ, WDT_CLK_FREQ_HZ, WDT_MAX_DIVIDER);


  uint64_t timeouts_cycles[NUM_STAGES];
  for (int i = 0; i < NUM_STAGES; ++i) {
    timeouts_cycles[i] = MS_TO_WDT_CYCLES(timeouts[i]);
  }
  status = wdt_configure(&wdt, WDT_FREQ_HZ, NUM_STAGES, timeouts_cycles);
  directive_failed(status, "WDT 0 Configure");

  status = wdt_handler_install(&wdt, WDT_isr, NULL);
  directive_failed(status, "WDT 0 Install ISR");

  wdt_enable(&wdt);

  volatile uint64_t interval_us = WDT_CYCLES_TO_US(timeouts_cycles[0]);
  printf("wdt test: interval %llu us\r\n", interval_us);

  /* Wait for stage to expire and trigger the interrupt */
  usleep(interval_us + interval_us / 4);
  fatal_int_service_status(expired, true, "WDT Stage 0 Expected to Expire");

  /* Kick the watchdog so it won't fire the interrupt again */
  wdt_kick(&wdt);

  status = wdt_handler_remove(&wdt, WDT_isr, NULL);
  directive_failed(status, "WDT 0 Remove ISR");
  wdt_disable(&wdt);
  wdt_uninit(&wdt);

  TEST_END();
  rtems_test_exit( 0 );
}
