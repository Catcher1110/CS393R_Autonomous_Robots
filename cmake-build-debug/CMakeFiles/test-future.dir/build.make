# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

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
CMAKE_COMMAND = /snap/clion/124/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/124/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jianc/projects/cs393r_starter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jianc/projects/cs393r_starter/cmake-build-debug

# Utility rule file for test-future.

# Include the progress variables for this target.
include CMakeFiles/test-future.dir/progress.make

test-future: CMakeFiles/test-future.dir/build.make

.PHONY : test-future

# Rule to build all files generated by this target.
CMakeFiles/test-future.dir/build: test-future

.PHONY : CMakeFiles/test-future.dir/build

CMakeFiles/test-future.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test-future.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test-future.dir/clean

CMakeFiles/test-future.dir/depend:
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jianc/projects/cs393r_starter /home/jianc/projects/cs393r_starter /home/jianc/projects/cs393r_starter/cmake-build-debug /home/jianc/projects/cs393r_starter/cmake-build-debug /home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles/test-future.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test-future.dir/depend

