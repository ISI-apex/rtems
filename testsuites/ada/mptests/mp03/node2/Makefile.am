include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am

include $(top_srcdir)/ada.am

AM_ADAFLAGS += -I$(srcdir)/..

noinst_PROGRAMS = mp03_ada_mp03_node2
mp03_ada_mp03_node2_SOURCES = mp03_node2.adb ../mptest.adb config.h
mp03_ada_mp03_node2_SOURCES += ../mptest.ads
mp03_ada_mp03_node2_SOURCES += ../../../support/init.c

mp03_ada_mp03_node2$(EXEEXT): mp03_node2.adb ../mptest.adb init.$(OBJEXT)
	$(GNATCOMPILE) -margs -a $< -o $@

scndir = $(rtems_ada_testsdir)
dist_scn_DATA = ada_mp03-node2.scn

