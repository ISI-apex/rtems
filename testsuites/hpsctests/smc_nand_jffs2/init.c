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
#include <bsp/xnandps_blkdev.h>
#include <bsp/xnandps_bbm.h>
#include <bsp/xnandps_jffs2.h>
#include <bsp/xnandps.h>
#include <rtems/blkdev.h>

#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <unistd.h>

//#include "fstest.h"
//#include "fs_config.h"

const char rtems_test_name[] = "HPSC SMC-353 NAND JFFS2 Write";
XNandPs NandInstance; /* XNand Instance. */
XNandPs_Flash_Control flash_control;
rtems_jffs2_mount_data mount_data;
static rtems_resource_snapshot before_mount;
static rtems_resource_snapshot after_unmount;
#define BASE_FOR_TEST "/jffs2_mnt"
u8 block_data[64*2048];
void *volatile prevent_compiler_optimizations;

/* Erase the blocks in the flash */
static void erase_blocks(XNandPs *NandInstPtr)
{
  u32 BlockIndex;
  u32 EndBlock = NandInstPtr->Geometry.NumBlocks;
  rtems_status_code status;
  memset(block_data, 0xff, sizeof(block_data));

  for (BlockIndex = 0; BlockIndex < EndBlock; BlockIndex++) {
    /* Don't erase bad blocks. */
    if (XNandPs_IsBlockBad(NandInstPtr, BlockIndex) == XST_SUCCESS) {
      continue;
    }
    /* Perform erase operation. */
    status = XNandPs_EraseBlock(NandInstPtr, BlockIndex);
    directive_failed(status, "Block Data Erase");
    /* Write FF to the entire block */
    u64 ofs = BlockIndex;
    ofs *= NandInstPtr->Geometry.BlockSize;
    status = XNandPs_Write(NandInstPtr, ofs, sizeof(block_data), block_data, NULL);
    directive_failed(status, "Block Data Write 0xFF");
  }
}

#define FILE_NAME "aaa"

#define DIR_NAME "bbb"

static void test1(void)
{
  int rv;
  char buf[10];

  rtems_test_assert(MAXNAMLEN == NAME_MAX);

  rv = mknod(FILE_NAME, S_IFREG | S_IRWXU | S_IRWXG | S_IRWXO, 0);
  rtems_test_assert(rv == 0);

  rv = mkdir(DIR_NAME, S_IRWXU | S_IRWXG | S_IRWXO );
  rtems_test_assert(rv == 0);

  struct stat st;
  rv = stat(FILE_NAME, &st);
  rtems_test_assert(rv == 0);

  rv = chdir(DIR_NAME);
  rtems_test_assert(rv == 0);

  rtems_test_assert(!strcmp(getcwd(buf, sizeof(buf)), "/" DIR_NAME));

  rv = chdir("..");
  rtems_test_assert(rv == 0);

  rtems_test_assert(!strcmp(getcwd(buf, sizeof(buf)), "/"));
}

static void test2(void)
{
  int rv;
  char buf[10];

  struct stat st;
  rv = stat(FILE_NAME, &st);
  rtems_test_assert(rv == 0);

  rv = chdir(DIR_NAME);
  rtems_test_assert(rv == 0);

  rtems_test_assert(!strcmp(getcwd(buf, sizeof(buf)), "/" DIR_NAME));

  rv = chdir("..");
  rtems_test_assert(rv == 0);

  rtems_test_assert(!strcmp(getcwd(buf, sizeof(buf)), "/"));
}

static void unchroot(void)
{
  int rv;
  /* break out of chroot */
  rtems_libio_use_global_env();

  /* Perform deferred global location releases */
  struct stat st;
  rv = stat(".", &st);
  rtems_test_assert(rv == 0);

  sleep(5);

  /* Perform deferred memory frees */
  prevent_compiler_optimizations = malloc(1);
  free(prevent_compiler_optimizations);
}

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;
  int rv;

  XNandPs_Config nand_config =
  {
    0,       /**< Device ID of device */
    (uint32_t)SMC_BASE,  /**< SMC Base address */
    (uint32_t)TRCH_NAND_BASE,        /**< NAND flash Base address */
    8      /**< Flash data width */
  };

  TEST_BEGIN();

  /* Initialize the flash driver. */
  status = XNandPs_CfgInitialize(&NandInstance, &nand_config,
      nand_config.SmcBase,nand_config.FlashBase);
  directive_failed(status, "NAND Initialization");

  /* erase flash */
  erase_blocks(&NandInstance);

  XNandPs_flash_control_init(&flash_control, &NandInstance);
  XNandPs_mount_data_init(&mount_data, &flash_control);

  rv = mkdir(BASE_FOR_TEST, S_IRWXU | S_IRWXG | S_IRWXO);
  rtems_test_assert(rv == 0);

  /* Extend heap since NAND writebuffer is fairly large */
  status = rtems_heap_extend((void *)0x40000000, 0x30000000);
  directive_failed(status, "Heap Extend for NAND Writebuffer");

  rtems_resource_snapshot_take(&before_mount);

  rv = mount(
    NULL,
    BASE_FOR_TEST,
    RTEMS_FILESYSTEM_TYPE_JFFS2,
    RTEMS_FILESYSTEM_READ_WRITE,
    &mount_data
  );
  rtems_test_assert(rv == 0);

  chroot(BASE_FOR_TEST);
  test1();
  unchroot();

  rv = unmount(BASE_FOR_TEST);
  rtems_test_assert(rv == 0);

  /* Remount FS and check to see that the structure is still correct */
  rv = mount(
    NULL,
    BASE_FOR_TEST,
    RTEMS_FILESYSTEM_TYPE_JFFS2,
    RTEMS_FILESYSTEM_READ_WRITE,
    &mount_data
  );
  rtems_test_assert(rv == 0);

  chroot(BASE_FOR_TEST);
  test2();
  unchroot();

  rv = unmount(BASE_FOR_TEST);
  rtems_test_assert(rv == 0);

  rtems_resource_snapshot_take(&after_unmount);
  rtems_test_assert(rtems_resource_snapshot_equal(&before_mount, &after_unmount));

  TEST_END();
  rtems_test_exit( 0 );
}
