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
CMAKE_BINARY_DIR = /home/vvalter/InversesPendel/build

# Utility rule file for listing.

# Include the progress variables for this target.
include CMakeFiles/listing.dir/progress.make

CMakeFiles/listing:
	echo -e "\n--- Symbols sorted by address ---\n" > /home/vvalter/InversesPendel/build/app.lst
	arm-none-eabi-nm -S -C -n /home/vvalter/InversesPendel/build/app.elf >> /home/vvalter/InversesPendel/build/app.lst
	echo -e "\n--- Symbols sorted by size ---\n" >> /home/vvalter/InversesPendel/build/app.lst
	arm-none-eabi-nm -S -C -r --size-sort /home/vvalter/InversesPendel/build/app.elf >> /home/vvalter/InversesPendel/build/app.lst
	echo -e "\n--- Full assembly listing ---\n" >> /home/vvalter/InversesPendel/build/app.lst
	arm-none-eabi-objdump -S -x -d -C /home/vvalter/InversesPendel/build/app.elf >> /home/vvalter/InversesPendel/build/app.lst

listing: CMakeFiles/listing
listing: CMakeFiles/listing.dir/build.make

.PHONY : listing

# Rule to build all files generated by this target.
CMakeFiles/listing.dir/build: listing

.PHONY : CMakeFiles/listing.dir/build

CMakeFiles/listing.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/listing.dir/cmake_clean.cmake
.PHONY : CMakeFiles/listing.dir/clean

CMakeFiles/listing.dir/depend:
	cd /home/vvalter/InversesPendel/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vvalter/InversesPendel/EZS /home/vvalter/InversesPendel/EZS /home/vvalter/InversesPendel/build /home/vvalter/InversesPendel/build /home/vvalter/InversesPendel/build/CMakeFiles/listing.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/listing.dir/depend

