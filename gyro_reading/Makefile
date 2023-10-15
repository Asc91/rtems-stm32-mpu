#
#  RTEMS_MAKEFILE_PATH is typically set in an environment variable
#

PGM=${ARCH}/mpu.exe

PROJECT_ROOT = ~/dev_rtems/rtems/5
RTEMS_MAKEFILE_PATH = ~/dev_rtems/rtems/5/arm-rtems5/stm32f4

# C source names
CSRCS = init.c
CSRCS += ../mpu6050.c
COBJS = $(CSRCS:%.c=${ARCH}/%.o)

include $(RTEMS_MAKEFILE_PATH)/Makefile.inc
include $(RTEMS_CUSTOM)
include $(PROJECT_ROOT)/make/leaf.cfg

OBJS= $(COBJS) $(CXXOBJS) $(ASOBJS)

all:    ${ARCH} $(PGM)

$(PGM): $(OBJS)
	$(make-exe)
