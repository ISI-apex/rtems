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
#include <stdio.h>
#include <fcntl.h>
#include <tmacros.h>
#include <bsp/synopsys/alt_generalpurpose_io.h>
#include <bspopts.h>

const char rtems_test_name[] = "TRCH GPIO Config Test";

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status = RTEMS_SUCCESSFUL;
  int bsp_configured_gpio_instances = NUM_GPIO_BLOCKS;
  ALT_GPIO_PARAM_ENABLE_STATE_t state;

  TEST_BEGIN();

  /* Check GPIO instances */
  printf("BSP is configured for %d GPIO instances\n",
         bsp_configured_gpio_instances);

  /* Check if synthesized GPIO instances support encoded parameters */
  for (ALT_GPIO_INSTANCE_t gpio_iid=ALT_GPIO_INSTANCE0;
       gpio_iid<bsp_configured_gpio_instances; gpio_iid++) {
    if ( gpio_iid == ALT_GPIO_INSTANCE_UNKNOWN) {
      status = RTEMS_NOT_CONFIGURED;
      directive_failed(status, "GPIO Configuration");
      break;
    }

    state = alt_gpio_add_encoded_params_state_get(gpio_iid);
    printf("GPIO instance %d encoded parameters are %s\n", gpio_iid,
           state == ALT_GPIO_PARAM_ENABLED ? "enabled" : "disabled");
    if (state == ALT_GPIO_PARAM_ENABLED) {
      ;
    }
  }

  status = alt_gpio_init();
  directive_failed(status, "GPIO Initialization");


  TEST_END();
  rtems_test_exit( 0 );
}
