# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vvalter/InversesPendel/EZS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vvalter/InversesPendel/EZS/build

# Include any dependencies generated for this target.
include CMakeFiles/app.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/app.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/app.dir/flags.make

ecos/install/lib/target.ld: ecos/makefile
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ecos/install/lib/target.ld"
	sh -c "make -C /home/vvalter/InversesPendel/EZS/build/ecos || exit -1; if [ -e /home/vvalter/InversesPendel/EZS/build/ecos/install/lib/target.ld ] ; then touch /home/vvalter/InversesPendel/EZS/build/ecos/install/lib/target.ld; fi"

ecos/makefile: ../ecos/ecos.ecc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating ecos/makefile"
	sh -c " cd /home/vvalter/InversesPendel/EZS/build/ecos; /proj/i4ezs/stm32/tools/ecosconfig --config=/home/vvalter/InversesPendel/EZS/ecos/ecos.ecc tree || exit -1;"

CMakeFiles/app.dir/vortrag.cpp.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/vortrag.cpp.o: ../vortrag.cpp
CMakeFiles/app.dir/vortrag.cpp.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/app.dir/vortrag.cpp.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_FLAGS) $(CXX_INCLUDES) -o CMakeFiles/app.dir/vortrag.cpp.o -c /home/vvalter/InversesPendel/EZS/vortrag.cpp

CMakeFiles/app.dir/vortrag.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app.dir/vortrag.cpp.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vvalter/InversesPendel/EZS/vortrag.cpp > CMakeFiles/app.dir/vortrag.cpp.i

CMakeFiles/app.dir/vortrag.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app.dir/vortrag.cpp.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vvalter/InversesPendel/EZS/vortrag.cpp -o CMakeFiles/app.dir/vortrag.cpp.s

CMakeFiles/app.dir/vortrag.cpp.o.requires:

.PHONY : CMakeFiles/app.dir/vortrag.cpp.o.requires

CMakeFiles/app.dir/vortrag.cpp.o.provides: CMakeFiles/app.dir/vortrag.cpp.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/vortrag.cpp.o.provides.build
.PHONY : CMakeFiles/app.dir/vortrag.cpp.o.provides

CMakeFiles/app.dir/vortrag.cpp.o.provides.build: CMakeFiles/app.dir/vortrag.cpp.o


CMakeFiles/app.dir/src/lib.c.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/src/lib.c.o: ../src/lib.c
CMakeFiles/app.dir/src/lib.c.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/app.dir/src/lib.c.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc    $(C_FLAGS) $(C_INCLUDES) -o CMakeFiles/app.dir/src/lib.c.o -c /home/vvalter/InversesPendel/EZS/src/lib.c

CMakeFiles/app.dir/src/lib.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/app.dir/src/lib.c.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vvalter/InversesPendel/EZS/src/lib.c > CMakeFiles/app.dir/src/lib.c.i

CMakeFiles/app.dir/src/lib.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/app.dir/src/lib.c.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vvalter/InversesPendel/EZS/src/lib.c -o CMakeFiles/app.dir/src/lib.c.s

CMakeFiles/app.dir/src/lib.c.o.requires:

.PHONY : CMakeFiles/app.dir/src/lib.c.o.requires

CMakeFiles/app.dir/src/lib.c.o.provides: CMakeFiles/app.dir/src/lib.c.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/src/lib.c.o.provides.build
.PHONY : CMakeFiles/app.dir/src/lib.c.o.provides

CMakeFiles/app.dir/src/lib.c.o.provides.build: CMakeFiles/app.dir/src/lib.c.o


CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o: ../libEZS/drivers/stm32f4/ezs_gpio.c
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc    $(C_FLAGS) $(C_INCLUDES) -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o -c /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_gpio.c

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_gpio.c > CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.i

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_gpio.c -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.s

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.requires:

.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.requires

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.provides: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.provides.build
.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.provides

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.provides.build: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o


CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o: ../libEZS/drivers/stm32f4/ezs_dac.cpp
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_FLAGS) $(CXX_INCLUDES) -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o -c /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_dac.cpp

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_dac.cpp > CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.i

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_dac.cpp -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.s

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.requires:

.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.requires

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.provides: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.provides.build
.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.provides

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.provides.build: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o


CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o: ../libEZS/drivers/stm32f4/ezs_serial.cpp
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_FLAGS) $(CXX_INCLUDES) -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o -c /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_serial.cpp

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_serial.cpp > CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.i

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_serial.cpp -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.s

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.requires:

.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.requires

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.provides: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.provides.build
.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.provides

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.provides.build: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o


CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o: ../libEZS/drivers/stm32f4/ezs_counter.cpp
CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_FLAGS) $(CXX_INCLUDES) -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o -c /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_counter.cpp

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_counter.cpp > CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.i

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vvalter/InversesPendel/EZS/libEZS/drivers/stm32f4/ezs_counter.cpp -o CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.s

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.requires:

.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.requires

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.provides: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.provides.build
.PHONY : CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.provides

CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.provides.build: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o


CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o: CMakeFiles/app.dir/flags.make
CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o: ../libEZS/src/ezs_delay.c
CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o: ecos/install/lib/target.ld
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc    $(C_FLAGS) $(C_INCLUDES) -o CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o -c /home/vvalter/InversesPendel/EZS/libEZS/src/ezs_delay.c

CMakeFiles/app.dir/libEZS/src/ezs_delay.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/app.dir/libEZS/src/ezs_delay.c.i"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/vvalter/InversesPendel/EZS/libEZS/src/ezs_delay.c > CMakeFiles/app.dir/libEZS/src/ezs_delay.c.i

CMakeFiles/app.dir/libEZS/src/ezs_delay.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/app.dir/libEZS/src/ezs_delay.c.s"
	/proj/i4ezs/stm32//gcc-arm-none-eabi/bin/arm-none-eabi-gcc  $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/vvalter/InversesPendel/EZS/libEZS/src/ezs_delay.c -o CMakeFiles/app.dir/libEZS/src/ezs_delay.c.s

CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.requires:

.PHONY : CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.requires

CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.provides: CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.requires
	$(MAKE) -f CMakeFiles/app.dir/build.make CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.provides.build
.PHONY : CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.provides

CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.provides.build: CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o


# Object files for target app
app_OBJECTS = \
"CMakeFiles/app.dir/vortrag.cpp.o" \
"CMakeFiles/app.dir/src/lib.c.o" \
"CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o" \
"CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o" \
"CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o" \
"CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o" \
"CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o"

# External object files for target app
app_EXTERNAL_OBJECTS =

app.elf: CMakeFiles/app.dir/vortrag.cpp.o
app.elf: CMakeFiles/app.dir/src/lib.c.o
app.elf: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o
app.elf: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o
app.elf: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o
app.elf: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o
app.elf: CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o
app.elf: CMakeFiles/app.dir/build.make
app.elf: CMakeFiles/app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable app.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/app.dir/link.txt --verbose=$(VERBOSE)
	arm-none-eabi-objcopy -O binary /home/vvalter/InversesPendel/EZS/build/app.elf /home/vvalter/InversesPendel/EZS/build/app.bin
	arm-none-eabi-objcopy -O srec /home/vvalter/InversesPendel/EZS/build/app.elf /home/vvalter/InversesPendel/EZS/build/app.srec

# Rule to build all files generated by this target.
CMakeFiles/app.dir/build: app.elf

.PHONY : CMakeFiles/app.dir/build

CMakeFiles/app.dir/requires: CMakeFiles/app.dir/vortrag.cpp.o.requires
CMakeFiles/app.dir/requires: CMakeFiles/app.dir/src/lib.c.o.requires
CMakeFiles/app.dir/requires: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_gpio.c.o.requires
CMakeFiles/app.dir/requires: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_dac.cpp.o.requires
CMakeFiles/app.dir/requires: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_serial.cpp.o.requires
CMakeFiles/app.dir/requires: CMakeFiles/app.dir/libEZS/drivers/stm32f4/ezs_counter.cpp.o.requires
CMakeFiles/app.dir/requires: CMakeFiles/app.dir/libEZS/src/ezs_delay.c.o.requires

.PHONY : CMakeFiles/app.dir/requires

CMakeFiles/app.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/app.dir/cmake_clean.cmake
.PHONY : CMakeFiles/app.dir/clean

CMakeFiles/app.dir/depend: ecos/install/lib/target.ld
CMakeFiles/app.dir/depend: ecos/makefile
	cd /home/vvalter/InversesPendel/EZS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vvalter/InversesPendel/EZS /home/vvalter/InversesPendel/EZS /home/vvalter/InversesPendel/EZS/build /home/vvalter/InversesPendel/EZS/build /home/vvalter/InversesPendel/EZS/build/CMakeFiles/app.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/app.dir/depend

