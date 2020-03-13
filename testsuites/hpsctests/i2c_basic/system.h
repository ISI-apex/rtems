/*  system.h
 *
 *  This include file contains information that is included in every
 *  function in the test set.
 *
 *  COPYRIGHT (c) 1989-1999.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */

#include <rtems.h>
#include <rtems/test.h>
#include <bsp/i2cdrv.h>

/* functions */

rtems_task Init(
  rtems_task_argument argument
);

/* configuration information */

#include <bsp.h> /* for device driver prototypes */

#define CONFIGURE_APPLICATION_NEEDS_SIMPLE_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_DOES_NOT_NEED_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS            2
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_BSP_PREREQUISITE_DRIVERS I2C_DRIVER_TABLE_ENTRY
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS    4

#define CONFIGURE_MAXIMUM_DRIVERS    12
#define CONFIGURE_MAXIMUM_SEMAPHORES 4
#if 0
#define CONFIGURE_MINIMUM_TASK_STACK_SIZE 512
#endif

#include <rtems/confdefs.h>

/* end of include file */