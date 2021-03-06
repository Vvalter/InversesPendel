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

# Utility rule file for gdb.

# Include the progress variables for this target.
include CMakeFiles/gdb.dir/progress.make

CMakeFiles/gdb: app.elf
CMakeFiles/gdb: tools
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Invoking gdb for debugging: starts at cyg_user_start()"
	bash /proj/i4ezs/stm32/tools/make_stm_symlinks.sh /home/vvalter/InversesPendel/EZS/build
	/proj/i4ezs/stm32/gcc-arm-none-eabi/bin/arm-none-eabi-gdb -nh --command=/proj/i4ezs/stm32//tools/ezs_debug_plain.gdb app.elf

tools:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/vvalter/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating tools"
	/usr/bin/cmake -E create_symlink /proj/i4ezs/stm32//tools /home/vvalter/InversesPendel/EZS/build/tools

gdb: CMakeFiles/gdb
gdb: tools
gdb: CMakeFiles/gdb.dir/build.make

.PHONY : gdb

# Rule to build all files generated by this target.
CMakeFiles/gdb.dir/build: gdb

.PHONY : CMakeFiles/gdb.dir/build

CMakeFiles/gdb.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gdb.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gdb.dir/clean

CMakeFiles/gdb.dir/depend:
	cd /home/vvalter/InversesPendel/EZS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vvalter/InversesPendel/EZS /home/vvalter/InversesPendel/EZS /home/vvalter/InversesPendel/EZS/build /home/vvalter/InversesPendel/EZS/build /home/vvalter/InversesPendel/EZS/build/CMakeFiles/gdb.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gdb.dir/depend

