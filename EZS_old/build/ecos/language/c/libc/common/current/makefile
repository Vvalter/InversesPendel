# eCos makefile

# This is a generated file - do not edit

export PREFIX := /home/vvalter/InversesPendel/EZS/build/ecos/install
export COMMAND_PREFIX := arm-none-eabi-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

export REPOSITORY := /proj/i4ezs/stm32//ecos/packages
PACKAGE := language/c/libc/common/current
OBJECT_PREFIX := language_c_libc_common
CFLAGS := -Wall -Wpointer-arith -Wstrict-prototypes -Wundef -Woverloaded-virtual -Wno-write-strings -mcpu=cortex-m4 -mthumb -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions
LDFLAGS := -mcpu=cortex-m4 -mthumb -Wl,--gc-sections -Wl,-static -Wl,-n -g -nostdlib
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := 

build: headers

headers: mlt_headers

include $(REPOSITORY)/pkgconf/rules.mak

