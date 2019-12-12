##
#
# @file
#
# @brief Makefile of LibBSP for the Cortex-R52 platform.
#

ACLOCAL_AMFLAGS = -I ../../../../aclocal

include $(top_srcdir)/../../../../automake/compile.am
include $(top_srcdir)/../../bsp.am

dist_project_lib_DATA = ../../../../../../bsps/arm/hpsc-trch/start/bsp_specs

start.$(OBJEXT): ../../../../../../bsps/arm/shared/start/start.S
	$(CPPASCOMPILE) -o $@ -c $<
project_lib_DATA = start.$(OBJEXT)

project_lib_DATA += linkcmds
dist_project_lib_DATA += ../../../../../../bsps/arm/hpsc-trch/start/linkcmds


hpsc-arch.c: ../../../../../../bsps/arm/hpsc-trch/start/hpsc-arch.dts
	arm-rtems5-gcc -E -nostdinc -I$(<D)/../include/bsp -x assembler-with-cpp -o - $< |dtc ${DTC_ALIGN} -O asm > hpsc-arch.s
	arm-rtems5-as hpsc-arch.s -o hpsc-arch.o
	arm-rtems5-objcopy -O binary hpsc-arch.o hpsc-arch.bin
	rtems-bin2c hpsc-arch.bin hpsc-arch.c
project_lib_DATA += hpsc-arch.c

project_lib_LIBRARIES = librtemsbsp.a

librtemsbsp_a_SOURCES =
librtemsbsp_a_CPPFLAGS = $(AM_CPPFLAGS)

CFLAGS += -Wno-missing-prototypes

# Shared
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/getentropy/getentropy-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/btimer/btimer-cpucounter.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/sbrk.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/stackalloc.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspfatal-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bspgetworkarea-default.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/start/bsp-fdt.c

# Startup
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/start/bsp-start-memcpy.S
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/cpucounter/cpucounter-armv7m.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/start/bspclean.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/start/bspreset.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/start/bspstart.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/start/bspstarthooks.c
librtemsbsp_a_SOURCES += hpsc-arch.c

# IRQ
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/irq/irq-default-handler.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/irq/irq-armv7m.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/irq/irq-dispatch-armv7m.c

# Console
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/console-termios-init.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/shared/dev/serial/console-termios.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/console/console-config.c

# Clock
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/clock/clock-armv7m.c

# GPIO
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/gpio/alt_generalpurpose_io.c

# I2C
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/i2c/i2cdrv.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/hpsc-trch/i2c/i2cdrv-config.c
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/i2c/alt_i2c.c

# SPI
librtemsbsp_a_SOURCES += ../../../../../../bsps/arm/shared/spi/alt_qspi.c

###############################################################################
#                  Special Rules                                              #
###############################################################################


include $(srcdir)/../../../../../../bsps/shared/irq-sources.am
include $(srcdir)/../../../../../../bsps/shared/shared-sources.am
include $(srcdir)/../../../../../../bsps/arm/hpsc-trch/headers.am
