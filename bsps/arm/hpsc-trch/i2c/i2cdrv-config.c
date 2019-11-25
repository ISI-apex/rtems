/**
 * @file
 *
 * @ingroup RTEMSBSPsARMCycVI2C
 */

/*
 * Copyright (c) 2014 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <info@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp.h>
#include "i2cdrv-config.h"

const i2cdrv_configuration i2cdrv_config[TRCH_NO_I2C] = {
  {
    .controller = ALT_I2C_I2C0,
    .device_name = "/dev/i2c0",
    .speed = TRCH_I2C0_SPEED
  },
  {
    .controller = ALT_I2C_I2C1,
    .device_name = "/dev/i2c1",
    .speed = TRCH_I2C1_SPEED
  }
};
