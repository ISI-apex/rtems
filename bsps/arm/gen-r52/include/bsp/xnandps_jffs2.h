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

#ifndef __XNANDPS_JFFS2_H__
#define __XNANDPS_JFFS2_H__

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#include <bsp/xnandps.h>
#include <rtems/jffs2.h>

typedef struct {
  rtems_jffs2_flash_control super;
  XNandPs *flash_driver;
} XNandPs_Flash_Control;

/*!
 * Initialize the SMC-353.
 *
 * \param       fc
 *              The allocated flash control structure to initialize.
 *
 * \param       NandInstPtr
 *              A pointer to an initialized NAND Instance structure for JFFS2 backend.
 */
void XNandPs_flash_control_init(XNandPs_Flash_Control *fc, XNandPs *NandInstPtr);
void XNandPs_mount_data_init(rtems_jffs2_mount_data *mount_data, XNandPs_Flash_Control *fc);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* __XNANDPS_JFFS2_H__ */
