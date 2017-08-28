# eCos makefile

# This is a generated file - do not edit

export PREFIX := /home/vvalter/InversesPendel/EZS/build/ecos/install
export COMMAND_PREFIX := arm-none-eabi-
export CC := $(COMMAND_PREFIX)gcc
export OBJCOPY := $(COMMAND_PREFIX)objcopy
export HOST := UNIX
export AR := $(COMMAND_PREFIX)ar

export REPOSITORY := /proj/i4ezs/stm32//ecos/packages
PACKAGE := io/serial/current
OBJECT_PREFIX := io_serial
CFLAGS := -Wall -Wpointer-arith -Wstrict-prototypes -Wundef -Woverloaded-virtual -Wno-write-strings -mcpu=cortex-m4 -mthumb -g -O2 -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions
LDFLAGS := -mcpu=cortex-m4 -mthumb -Wl,--gc-sections -Wl,-static -Wl,-n -g -nostdlib
VPATH := $(REPOSITORY)/$(PACKAGE)
INCLUDE_PATH := $(INCLUDE_PATH) -I$(PREFIX)/include $(foreach dir,$(VPATH),-I$(dir) -I$(dir)/src -I$(dir)/tests) -I.
MLT := $(wildcard $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.ldi $(REPOSITORY)/$(PACKAGE)/include/pkgconf/mlt*.h)
TESTS := tests/serial1 tests/serial2 tests/serial3 tests/serial4 tests/serial5 tests/tty1 tests/tty2 tests/flow1 tests/flow2

build: headers libextras.a.stamp

LIBRARY := libextras.a
COMPILE := src/common/serial.c src/common/tty.c src/common/haldiag.c
OBJECTS := $(COMPILE:.cxx=.o.d)
OBJECTS := $(OBJECTS:.cpp=.o.d)
OBJECTS := $(OBJECTS:.c=.o.d)
OBJECTS := $(OBJECTS:.S=.o.d)

$(LIBRARY).stamp: $(OBJECTS)
	$(AR) rcs $(PREFIX)/lib/$(@:.stamp=) $(foreach obj,$?,$(if $(obj:%.o=),$(dir $(obj))$(OBJECT_PREFIX)_$(notdir $(obj:.o.d=.o)),$(obj)))
	@cat $(foreach obj,$^,$(obj:.o=.o.d)) > $(@:.stamp=.deps)
	@touch $@

headers: mlt_headers $(PREFIX)/include/cyg/io/serial.h $(PREFIX)/include/cyg/io/ttyio.h $(PREFIX)/include/cyg/io/serialio.h

$(PREFIX)/include/cyg/io/serial.h: $(REPOSITORY)/$(PACKAGE)/include/serial.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/io/ttyio.h: $(REPOSITORY)/$(PACKAGE)/include/ttyio.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

$(PREFIX)/include/cyg/io/serialio.h: $(REPOSITORY)/$(PACKAGE)/include/serialio.h
	@mkdir -p $(dir $@)
	@cp $< $@
	@chmod u+w $@

include $(REPOSITORY)/pkgconf/rules.mak

