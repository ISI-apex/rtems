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
#include <bsp/mmu_500.h>

const char rtems_test_name[] = "RTPS/TRCH-to-HPPS MMU Map";

/* Page table */
#define RTPS_HPPS_PT_ADDR                0xc3e00000
#define RTPS_HPPS_PT_SIZE                  0x1ff000

/* HPPS DDR LOW */
#define HPPS_DDR_LOW_ADDR__HPPS_SMP   (uint32_t*)0xc0000000
#define HPPS_DDR_LOW_SIZE__HPPS_SMP   0x1fc00000

#define RT_MMU_TEST_DATA_HI_0_WIN_ADDR   (uint32_t*)0xc0000000
#define RT_MMU_TEST_DATA_HI_1_WIN_ADDR   (uint32_t*)0xc1000000

/* Two regions of same size */
#define RT_MMU_TEST_DATA_HI_0_ADDR     (uint64_t*)0x100000000
#define RT_MMU_TEST_DATA_HI_1_ADDR     (uint64_t*)0x100010000
#define RT_MMU_TEST_DATA_HI_SIZE       0x10000

#define MMU_TEST_DATA_HI_0_ADDR        (uint64_t*)0x0500000000
#define MMU_TEST_DATA_HI_1_ADDR        (uint64_t*)0x0500001000
#define MMU_TEST_DATA_HI_SIZE          0x001000

#define MMU_TEST_DATA_LO_0_ADDR        (uint32_t*)0xc2ffe000
#define MMU_TEST_DATA_LO_1_ADDR        (uint32_t*)0xc2fff000
#define MMU_TEST_DATA_LO_SIZE          0x001000

rtems_status_code test_32_mmu_access_physical_mwr(
       MMU_Config_t *mmu_config,
       MMU_Context_t *trch_ctx,
       uint32_t *virt_write_addr,
       uint32_t *phy_read_addr,
       unsigned mapping_sz);
rtems_status_code test_32_mmu_access_physical_wmr(
       MMU_Config_t *mmu_config,
       MMU_Context_t *trch_ctx,
       uint32_t *virt_read_addr,
       uint32_t *phy_write_addr,
       unsigned mapping_sz);
rtems_status_code test_mmu_mapping_swap(
       MMU_Config_t *mmu_config,
       MMU_Context_t *trch_ctx,
       uint32_t *virt_write_addr,
       uint32_t *virt_read_addr,
       uint64_t *phy_addr,
       unsigned mapping_sz);
rtems_status_code test_rt_mmu( MMU_Context_t *trch_ctx );

/* map-write-read test
 *  1. TRCH creates a mapping
 *  2. TRCH enables MMU
 *  3. TRCH writes to an address in the mapping created
 *  4. TRCH disables MMU
 *  5. TRCH reads the physical address where the data was written to and checks
 *     if the data is correct
 *
 * Test is successful if TRCH reads the correct data
 */
rtems_status_code test_32_mmu_access_physical_mwr(MMU_Config_t *mmu_config,
       MMU_Context_t *trch_ctx,
       uint32_t *virt_write_addr,
       uint32_t *phy_read_addr,
       unsigned mapping_sz){

    _Bool success = 0;
    
    uint32_t val = 0xbeeff00d;
    uint32_t old_val;

    mmu_disable(mmu_config); /* might be already enabled if the core reboots */

    if (mmu_map(trch_ctx, (uint32_t*)virt_write_addr, (uint32_t*)phy_read_addr,
                          mapping_sz)) {
        printf("test mmu 32-bit access physical map-write-read: "
               " mapping create failed\n");
        goto cleanup_map;
    }

    mmu_enable(mmu_config);

    old_val = *(virt_write_addr);
    *(virt_write_addr) = val;

    mmu_disable(mmu_config);

    if (*(phy_read_addr) == val) {
        success = 1;
    }
    else {
        printf( "test mmu 32-bit access physical map-write-read: "
            "read wrong value\n");
        success = 0;
    }

    mmu_enable(mmu_config);

    *(virt_write_addr) = old_val;

    mmu_disable(mmu_config);

    mmu_unmap(trch_ctx, virt_write_addr, mapping_sz);

cleanup_map:

    if (success) {
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_INVALID_ADDRESS;
}

/* write-map-read test
 *  1. TRCH creates a mapping
 *  2. TRCH writes to a low physical address
 *  3. TRCH enables MMU
 *  4. TRCH reads the virtual address corresponding to the physical address
 *  written to and checks if data is correct
 *
 *  Test is successful if TRCH reads the correct data
 */
rtems_status_code test_32_mmu_access_physical_wmr(MMU_Config_t *mmu_config,
       MMU_Context_t *trch_ctx,
       uint32_t *virt_read_addr,
       uint32_t *phy_write_addr,
       unsigned mapping_sz){

    _Bool success = 0;

    uint32_t val = 0xbeeff00d;
    uint32_t old_val;

    /* might be already enabled if the core reboots */
    mmu_disable(mmu_config); 

    old_val = *(phy_write_addr);
    *(phy_write_addr) = val;

    if (mmu_map(trch_ctx, virt_read_addr, phy_write_addr, mapping_sz)) {
        printf("test mmu 32-bit access physical write-map-read: "
            "mapping create failed\n");
        goto cleanup_map;
    }

    mmu_enable(mmu_config);

    if (*(virt_read_addr) == val) {
        success = 1;
    }
    else {
        printf("test mmu 32-bit access physical write-map-read: "
               "read wrong value\n");
        success = 0;
    }

    *(virt_read_addr) = old_val;

    mmu_disable(mmu_config);

    mmu_unmap(trch_ctx, virt_read_addr, mapping_sz);

cleanup_map:

    if (success) {
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_INVALID_ADDRESS;
}

/* In this test,
 *   1. TRCH creates a mapping from virt_write_addr to phy_addr
 *   2. TRCH enables MMU
 *   3. TRCH writes data to virt_write_addr (effectively writing to phy_addr)
 *   4. TRCH unmaps the mapping created in step 1
 *   5. TRCH maps virt_read_addr to phy_addr
 *   6. TRCH reads virt_read_addr, checking if data is same as in step 3
 *
 *   Test is successful if TRCH reads the correct data
 */
rtems_status_code test_mmu_mapping_swap(
       MMU_Config_t *mmu_config,
       MMU_Context_t *trch_ctx,
       uint32_t *virt_write_addr,
       uint32_t *virt_read_addr,
       uint64_t *phy_addr,
       unsigned mapping_sz){

    _Bool success = 0;

    uint32_t val = 0xbeeff00d;
    uint32_t old_val;

    /* might be already enabled if the core reboots */
    mmu_disable(mmu_config);

    if (mmu_map(trch_ctx, virt_write_addr, phy_addr, mapping_sz)) {
        printf("test mmu mapping swap: "
               "addr_from_1->addr_to mapping create failed\n");
        goto cleanup_map;
    }

    mmu_enable(mmu_config);

    old_val = *(virt_write_addr);
    *(virt_write_addr) = val;

    mmu_disable(mmu_config);

    mmu_unmap(trch_ctx, virt_write_addr, mapping_sz);

    if (mmu_map(trch_ctx, virt_read_addr, phy_addr, mapping_sz)) {
        printf("test mmu mapping swap: "
               "addr_from_1->addr_to mapping create failed\n");
        goto cleanup_map;
    }

    mmu_enable(mmu_config);

    if (*(virt_read_addr) == val) {
        success = 1;
    }
    else {
        printf("test mmu mapping swap: "
               "read wrong value %x \n", *(virt_read_addr));
        success = 0;
    }

    *(virt_read_addr) = old_val;

    mmu_disable(mmu_config);

    mmu_unmap(trch_ctx, virt_read_addr, mapping_sz);

cleanup_map:

    if (success) {
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_INVALID_ADDRESS;
}

/* In this test, TRCH accesses same location as RTPS but via a different
 * virtual addr.
 * RTPS should read 0xc0000000 and find 0xbeeff00d, not 0xf00dbeef.
 *
 * Mappings are setup in trch/mmus.c since no easy way to keep this test
 * truly standalone, since the mappings for this test and for the MMU
 * usecase need to be active at the same time (since RTPS needs to
 *excercise these mappings when it boots).
*/
rtems_status_code test_rt_mmu( MMU_Context_t *trch_ctx )
{
  _Bool success = 0;

  if ( mmu_map(trch_ctx, HPPS_DDR_LOW_ADDR__HPPS_SMP,
                         HPPS_DDR_LOW_ADDR__HPPS_SMP,
                         HPPS_DDR_LOW_SIZE__HPPS_SMP)) {
    printf("test rt mmu: "
           "HPPS_DDR_LOW_ADDR_HPPS_SMP mapping create failed\n");
    goto cleanup_map;
  }

  if ( mmu_map(trch_ctx, RT_MMU_TEST_DATA_HI_0_WIN_ADDR,
                         RT_MMU_TEST_DATA_HI_1_ADDR,
                         RT_MMU_TEST_DATA_HI_SIZE)) {
    printf("test rt mmu: "
           "RT_MMU_TEST_DATA_HI_0_WIN_ADDR mapping create failed\n");
    goto cleanup_map;
  }

  if ( mmu_map(trch_ctx, RT_MMU_TEST_DATA_HI_1_WIN_ADDR,
                         RT_MMU_TEST_DATA_HI_0_ADDR,
                         RT_MMU_TEST_DATA_HI_SIZE)) {
    printf("test rt mmu: "
           "RT_MMU_TEST_DATA_HI_1_WIN_ADDR mapping create failed\n");
    goto cleanup_map;
  }

  uint32_t *addr = RT_MMU_TEST_DATA_HI_1_WIN_ADDR;
  uint32_t val = 0xbeeff00d;
  printf("%p <- %08x\r\n", addr, val);
  *addr = val;

  addr = RT_MMU_TEST_DATA_HI_0_WIN_ADDR;
  val = 0xf00dbeef;
  printf("%p <- %08x\r\n", addr, val);
  *addr = val;

  success = 1;

cleanup_map:

    if (success) {
        return RTEMS_SUCCESSFUL;
    }
    return RTEMS_INVALID_ADDRESS;
}

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  TEST_BEGIN();

  /* Initialize MMU controller */
  MMU_Config_t mmu_config;
  status = mmu_init(&mmu_config, RTPS_TRCH_TO_HPPS_SMMU_BASE);
  directive_failed(status, "MMU Initialization");

  /* Disable MMU mappings */
  status = mmu_disable(&mmu_config);
  directive_failed(status, "MMU Disable");

  /* Define a memory region for allocations */
  rtems_id region_id;
  status = rtems_region_create(rtems_build_name('M', 'M', 'U', ' '),
             (void*)RTPS_HPPS_PT_ADDR,
             RTPS_HPPS_PT_SIZE,
             0x1000 /* 4KB page size */,
             RTEMS_PRIORITY,
             &region_id);

  /* Create MMU context */
  MMU_Context_t *mmu_context;
  status = mmu_context_create(&mmu_context,
              &mmu_config,
              region_id,
              MMU_PAGESIZE_4KB);
  directive_failed(status, "MMU Context Creation");

  /* Create MMU stream */
  MMU_Stream_t *mmu_stream;
  status = mmu_stream_create(&mmu_stream, MASTER_ID_TRCH_CPU, mmu_context);
  directive_failed(status, "MMU Stream Creation");

  status = test_32_mmu_access_physical_mwr(
      &mmu_config,
      mmu_context,
      MMU_TEST_DATA_LO_0_ADDR,
      MMU_TEST_DATA_LO_1_ADDR,
      MMU_TEST_DATA_LO_SIZE);
  directive_failed(status, "MMU 32-bit access test map-write-read");

  status = test_32_mmu_access_physical_wmr(
      &mmu_config,
      mmu_context,
      MMU_TEST_DATA_LO_0_ADDR,
      MMU_TEST_DATA_LO_1_ADDR,
      MMU_TEST_DATA_LO_SIZE);
  directive_failed(status, "MMU 32-bit access test write-map-read");

  /*map argument 1 to argument 3 then argument 2 to argument 3 */
  status = test_mmu_mapping_swap(
      &mmu_config,
      mmu_context,
      MMU_TEST_DATA_LO_0_ADDR,
      MMU_TEST_DATA_LO_1_ADDR,
      MMU_TEST_DATA_HI_0_ADDR,
      MMU_TEST_DATA_LO_SIZE); 
  directive_failed(status, "MMU mapping swap");
 
  status = test_rt_mmu(mmu_context);
  directive_failed(status, "Test RT MMU");

  /* disable MMU mappings */
  status = mmu_disable(&mmu_config);
  directive_failed(status, "MMU Disable");
  
  /* Destroy the MMU stream */
  status = mmu_stream_destroy(mmu_stream);
  directive_failed(status, "MMU Stream Destruction");

  /* Destroy the MMU context */
  status = mmu_context_destroy(mmu_context);
  directive_failed(status, "MMU Context Destruction");

  /* Uninitialize MMU controller */
  status = mmu_uninit(&mmu_config);
  directive_failed(status, "MMU Uninitialization");

  TEST_END();
  rtems_test_exit( 0 );
}
