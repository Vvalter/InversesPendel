# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/cip/2012/vy37bypi/InversesPendel/EZS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cip/2012/vy37bypi/InversesPendel/EZS/build

# Utility rule file for aiT.

# Include the progress variables for this target.
include CMakeFiles/aiT.dir/progress.make

CMakeFiles/aiT:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cip/2012/vy37bypi/InversesPendel/EZS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "run the aiT WCET analyzer"
	/proj/i4ezs/tools/a3_arm/bin/a3arm

aiT: CMakeFiles/aiT
aiT: CMakeFiles/aiT.dir/build.make

.PHONY : aiT

# Rule to build all files generated by this target.
CMakeFiles/aiT.dir/build: aiT

.PHONY : CMakeFiles/aiT.dir/build

CMakeFiles/aiT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/aiT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/aiT.dir/clean

CMakeFiles/aiT.dir/depend:
	cd /home/cip/2012/vy37bypi/InversesPendel/EZS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cip/2012/vy37bypi/InversesPendel/EZS /home/cip/2012/vy37bypi/InversesPendel/EZS /home/cip/2012/vy37bypi/InversesPendel/EZS/build /home/cip/2012/vy37bypi/InversesPendel/EZS/build /home/cip/2012/vy37bypi/InversesPendel/EZS/build/CMakeFiles/aiT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/aiT.dir/depend

