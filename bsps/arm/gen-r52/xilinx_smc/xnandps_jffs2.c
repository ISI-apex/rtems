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

//#include "pmacros.h"

#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <assert.h>

#include <rtems/libio.h>
#include <rtems/libcsupport.h>
#include <bsp/xnandps.h>
#include <bsp/xnandps_jffs2.h>
#include <bsp/xnandps_bbm.h>

//#include "fstest.h"
//#include "fstest_support.h"

static XNandPs_Flash_Control *get_flash_control(rtems_jffs2_flash_control *super)
{
  return (XNandPs_Flash_Control *) super;
}

static int flash_read(
  rtems_jffs2_flash_control *super,
  uint32_t offset,
  unsigned char *buffer,
  size_t size_of_buffer
)
{
  XNandPs_Flash_Control *self = get_flash_control(super);
  uint32_t offset_index = offset / self->flash_driver->Geometry.BlockSize;
  uint32_t block_offset = offset % self->flash_driver->Geometry.BlockSize;

  while (size_of_buffer) {
    /* calculate new read offset for the current block */
    uint32_t read_offset = offset_index * self->flash_driver->Geometry.BlockSize + block_offset;
    uint32_t read_size = self->flash_driver->Geometry.BlockSize - block_offset;

    /* clamp read size to remaining bytes to read */
    if (read_size > size_of_buffer) {
      read_size = size_of_buffer;
    }

    /* only the first read can have a non-block-boundary offset */
    block_offset = 0;
    /* decrease remaining read by the size being read */
    size_of_buffer -= read_size;

    /* Perform the read operation for this block */
    rtems_status_code sc = XNandPs_Read(self->flash_driver, read_offset, read_size, buffer, NULL);
    if (sc) {
      return -EIO;
    }

    /* move buffer along by the size read */
    buffer += read_size;
    /* move to next block */
    offset_index++;
  }
  return 0;
}

static int flash_write(
  rtems_jffs2_flash_control *super,
  uint32_t offset,
  const unsigned char *buffer,
  size_t size_of_buffer
)
{
  XNandPs_Flash_Control *self = get_flash_control(super);
  uint32_t offset_index = offset / self->flash_driver->Geometry.BlockSize;
  uint32_t block_offset = offset % self->flash_driver->Geometry.BlockSize;

  while (size_of_buffer) {
    /* calculate new write offset for the current block */
    uint32_t write_offset = offset_index * self->flash_driver->Geometry.BlockSize + block_offset;
    uint32_t write_size = self->flash_driver->Geometry.BlockSize - block_offset;

    /* clamp write size to remaining bytes to write */
    if (write_size > size_of_buffer) {
      write_size = size_of_buffer;
    }

    /* only the first write can have a non-block-boundary offset */
    block_offset = 0;
    /* decrease remaining write by the size being written */
    size_of_buffer -= write_size;

    /* Perform the write operation for this block */
    rtems_status_code sc = XNandPs_Write(self->flash_driver, write_offset, write_size, (void *)buffer, NULL);
    if (sc) {
      return -EIO;
    }

    /* move buffer along by the size written */
    buffer += write_size;
    /* move to next block */
    offset_index++;
  }
  return 0;
}

static int flash_erase(
  rtems_jffs2_flash_control *super,
  uint32_t offset
)
{
  XNandPs_Flash_Control *self = get_flash_control(super);

  /* Get block index */
  uint32_t BlockIndex = offset / self->flash_driver->Geometry.BlockSize;

  /* Perform erase operation. */
  rtems_status_code sc = XNandPs_EraseBlock(self->flash_driver, BlockIndex);
  if (sc ) {
    return -EIO;
  }

  return 0;
}

static int flash_block_is_bad(
  rtems_jffs2_flash_control *super,
  uint32_t offset,
  bool *bad
)
{
  assert(bad);
  XNandPs_Flash_Control *self = get_flash_control(super);

  if (offset > self->flash_driver->Geometry.DeviceSize) {
    return -EIO;
  }

  /* Get block index */
  uint32_t BlockIndex = offset / self->flash_driver->Geometry.BlockSize;

  *bad = (XNandPs_IsBlockBad(self->flash_driver, BlockIndex) == XST_SUCCESS);
  return 0;
}

static int flash_block_mark_bad(
  rtems_jffs2_flash_control *super,
  uint32_t offset
)
{
  XNandPs_Flash_Control *self = get_flash_control(super);

  if (offset > self->flash_driver->Geometry.DeviceSize) {
    return -EIO;
  }

  /* Get block index */
  uint32_t BlockIndex = offset / self->flash_driver->Geometry.BlockSize;

  return XNandPs_MarkBlockBad(self->flash_driver, BlockIndex) == XST_SUCCESS ? RTEMS_SUCCESSFUL : -EIO;
}

static int flash_read_oob(
  rtems_jffs2_flash_control *super,
  uint32_t offset,
  uint8_t *oobbuf,
  uint32_t ooblen
)
{
  uint8_t spare_bytes[64];
  XNandPs_Flash_Control *self = get_flash_control(super);

  if (offset > self->flash_driver->Geometry.DeviceSize) {
    return -EIO;
  }

  /* Can't request more spare bytes than exist */
  if (ooblen > self->flash_driver->Geometry.SpareBytesPerPage * self->flash_driver->Geometry.PagesPerBlock) {
    return -EIO;
  }

  /* Get page index */
  uint32_t PageIndex = offset / self->flash_driver->Geometry.BytesPerPage;

  while (ooblen) {
    int rv = XNandPs_ReadSpareBytes(self->flash_driver, PageIndex, spare_bytes);
    /* oobbuf isn't guaranteed to be able to hold the entirety of spare bytes,
     * so read and then copy */
    uint32_t readlen = self->flash_driver->Geometry.SpareBytesPerPage;
    if (ooblen < readlen) {
	    readlen = ooblen;
    }

    if (rv) {
      return -EIO;
    }

    memcpy(oobbuf, spare_bytes, readlen);

    PageIndex++;
    ooblen -= readlen;
    oobbuf += readlen;
  }
  return RTEMS_SUCCESSFUL;
}

static int flash_write_oob(
  rtems_jffs2_flash_control *super,
  uint32_t offset,
  uint8_t *oobbuf,
  uint32_t ooblen
)
{
  uint8_t spare_bytes[64];
  uint8_t *buffer = oobbuf;
  XNandPs_Flash_Control *self = get_flash_control(super);

  if (offset > self->flash_driver->Geometry.DeviceSize) {
    return -EIO;
  }

  /* Writing a page spare area to large will result in ignored data.  */
  if (ooblen > self->flash_driver->Geometry.SpareBytesPerPage) {
    return -EIO;
  }

  /* Writing a page spare area to small will result in invalid accesses */
  if (ooblen < self->flash_driver->Geometry.SpareBytesPerPage) {
    int rv = flash_read_oob(super, offset, spare_bytes, self->flash_driver->Geometry.SpareBytesPerPage);
    if (rv) {
      return rv;
    }
    buffer = spare_bytes;
    memcpy(buffer, oobbuf, ooblen);
  }

  /* Get page index */
  uint32_t PageIndex = offset / self->flash_driver->Geometry.BytesPerPage;

  return XNandPs_WriteSpareBytes(self->flash_driver, PageIndex, buffer) == XST_SUCCESS ? RTEMS_SUCCESSFUL : -EIO;
}

static uint32_t flash_get_oob_size(
  rtems_jffs2_flash_control *super
)
{
  XNandPs_Flash_Control *self = get_flash_control(super);

  return self->flash_driver->Geometry.SpareBytesPerPage;
}

static XNandPs_Flash_Control flash_instance = {
  .super = {
    .read = flash_read,
    .write = flash_write,
    .erase = flash_erase,
    .block_is_bad = flash_block_is_bad,
    .block_mark_bad = flash_block_mark_bad,
    .oob_read = flash_read_oob,
    .oob_write = flash_write_oob,
    .get_oob_size = flash_get_oob_size,
  }
};

void XNandPs_flash_control_init(XNandPs_Flash_Control *fc, XNandPs *NandInstPtr)
{
  *fc = flash_instance;
  fc->super.block_size = NandInstPtr->Geometry.BlockSize;
  fc->super.flash_size = NandInstPtr->Geometry.DeviceSize;
  fc->super.write_size = NandInstPtr->Geometry.BytesPerPage;

  fc->flash_driver = NandInstPtr;
}

static rtems_jffs2_compressor_control compressor_instance = {
  .compress = rtems_jffs2_compressor_rtime_compress,
  .decompress = rtems_jffs2_compressor_rtime_decompress
};

void XNandPs_mount_data_init(rtems_jffs2_mount_data *mount_data, XNandPs_Flash_Control *fc)
{
  mount_data->compressor_control = &compressor_instance;
  mount_data->flash_control = &fc->super;
}

//#define CONFIGURE_INIT_TASK_ATTRIBUTES RTEMS_FLOATING_POINT
