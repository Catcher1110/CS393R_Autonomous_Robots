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

# Include any dependencies generated for this target.
include src/shared/CMakeFiles/amrl-shared-lib.dir/depend.make

# Include the progress variables for this target.
include src/shared/CMakeFiles/amrl-shared-lib.dir/progress.make

# Include the compile flags for this target's objects.
include src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make

src/shared/CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.o: src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make
src/shared/CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.o: ../src/shared/util/helpers.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/shared/CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.o"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.o -c /home/jianc/projects/cs393r_starter/src/shared/util/helpers.cc

src/shared/CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.i"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianc/projects/cs393r_starter/src/shared/util/helpers.cc > CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.i

src/shared/CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.s"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianc/projects/cs393r_starter/src/shared/util/helpers.cc -o CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.s

src/shared/CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.o: src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make
src/shared/CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.o: ../src/shared/util/pthread_utils.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/shared/CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.o"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.o -c /home/jianc/projects/cs393r_starter/src/shared/util/pthread_utils.cc

src/shared/CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.i"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianc/projects/cs393r_starter/src/shared/util/pthread_utils.cc > CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.i

src/shared/CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.s"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianc/projects/cs393r_starter/src/shared/util/pthread_utils.cc -o CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.s

src/shared/CMakeFiles/amrl-shared-lib.dir/util/timer.cc.o: src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make
src/shared/CMakeFiles/amrl-shared-lib.dir/util/timer.cc.o: ../src/shared/util/timer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/shared/CMakeFiles/amrl-shared-lib.dir/util/timer.cc.o"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amrl-shared-lib.dir/util/timer.cc.o -c /home/jianc/projects/cs393r_starter/src/shared/util/timer.cc

src/shared/CMakeFiles/amrl-shared-lib.dir/util/timer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amrl-shared-lib.dir/util/timer.cc.i"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianc/projects/cs393r_starter/src/shared/util/timer.cc > CMakeFiles/amrl-shared-lib.dir/util/timer.cc.i

src/shared/CMakeFiles/amrl-shared-lib.dir/util/timer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amrl-shared-lib.dir/util/timer.cc.s"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianc/projects/cs393r_starter/src/shared/util/timer.cc -o CMakeFiles/amrl-shared-lib.dir/util/timer.cc.s

src/shared/CMakeFiles/amrl-shared-lib.dir/util/random.cc.o: src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make
src/shared/CMakeFiles/amrl-shared-lib.dir/util/random.cc.o: ../src/shared/util/random.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/shared/CMakeFiles/amrl-shared-lib.dir/util/random.cc.o"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amrl-shared-lib.dir/util/random.cc.o -c /home/jianc/projects/cs393r_starter/src/shared/util/random.cc

src/shared/CMakeFiles/amrl-shared-lib.dir/util/random.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amrl-shared-lib.dir/util/random.cc.i"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianc/projects/cs393r_starter/src/shared/util/random.cc > CMakeFiles/amrl-shared-lib.dir/util/random.cc.i

src/shared/CMakeFiles/amrl-shared-lib.dir/util/random.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amrl-shared-lib.dir/util/random.cc.s"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianc/projects/cs393r_starter/src/shared/util/random.cc -o CMakeFiles/amrl-shared-lib.dir/util/random.cc.s

src/shared/CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.o: src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make
src/shared/CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.o: ../src/shared/util/serialization.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/shared/CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.o"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.o -c /home/jianc/projects/cs393r_starter/src/shared/util/serialization.cc

src/shared/CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.i"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianc/projects/cs393r_starter/src/shared/util/serialization.cc > CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.i

src/shared/CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.s"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianc/projects/cs393r_starter/src/shared/util/serialization.cc -o CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.s

src/shared/CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.o: src/shared/CMakeFiles/amrl-shared-lib.dir/flags.make
src/shared/CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.o: ../src/shared/util/terminal_colors.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/shared/CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.o"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.o -c /home/jianc/projects/cs393r_starter/src/shared/util/terminal_colors.cc

src/shared/CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.i"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianc/projects/cs393r_starter/src/shared/util/terminal_colors.cc > CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.i

src/shared/CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.s"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianc/projects/cs393r_starter/src/shared/util/terminal_colors.cc -o CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.s

# Object files for target amrl-shared-lib
amrl__shared__lib_OBJECTS = \
"CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.o" \
"CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.o" \
"CMakeFiles/amrl-shared-lib.dir/util/timer.cc.o" \
"CMakeFiles/amrl-shared-lib.dir/util/random.cc.o" \
"CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.o" \
"CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.o"

# External object files for target amrl-shared-lib
amrl__shared__lib_EXTERNAL_OBJECTS =

../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/util/helpers.cc.o
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/util/pthread_utils.cc.o
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/util/timer.cc.o
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/util/random.cc.o
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/util/serialization.cc.o
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/util/terminal_colors.cc.o
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/build.make
../lib/libamrl-shared-lib.so: src/shared/CMakeFiles/amrl-shared-lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jianc/projects/cs393r_starter/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library ../../../lib/libamrl-shared-lib.so"
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/amrl-shared-lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/shared/CMakeFiles/amrl-shared-lib.dir/build: ../lib/libamrl-shared-lib.so

.PHONY : src/shared/CMakeFiles/amrl-shared-lib.dir/build

src/shared/CMakeFiles/amrl-shared-lib.dir/clean:
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared && $(CMAKE_COMMAND) -P CMakeFiles/amrl-shared-lib.dir/cmake_clean.cmake
.PHONY : src/shared/CMakeFiles/amrl-shared-lib.dir/clean

src/shared/CMakeFiles/amrl-shared-lib.dir/depend:
	cd /home/jianc/projects/cs393r_starter/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jianc/projects/cs393r_starter /home/jianc/projects/cs393r_starter/src/shared /home/jianc/projects/cs393r_starter/cmake-build-debug /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared /home/jianc/projects/cs393r_starter/cmake-build-debug/src/shared/CMakeFiles/amrl-shared-lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/shared/CMakeFiles/amrl-shared-lib.dir/depend

