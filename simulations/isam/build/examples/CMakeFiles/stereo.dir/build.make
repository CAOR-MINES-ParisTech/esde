# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.11

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
CMAKE_COMMAND = /usr/local/lib/python3.5/dist-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /usr/local/lib/python3.5/dist-packages/cmake/data/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/stereo.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/stereo.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/stereo.dir/flags.make

examples/CMakeFiles/stereo.dir/stereo.cpp.o: examples/CMakeFiles/stereo.dir/flags.make
examples/CMakeFiles/stereo.dir/stereo.cpp.o: ../examples/stereo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/stereo.dir/stereo.cpp.o"
	cd /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo.dir/stereo.cpp.o -c /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/examples/stereo.cpp

examples/CMakeFiles/stereo.dir/stereo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo.dir/stereo.cpp.i"
	cd /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/examples/stereo.cpp > CMakeFiles/stereo.dir/stereo.cpp.i

examples/CMakeFiles/stereo.dir/stereo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo.dir/stereo.cpp.s"
	cd /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples && /usr/lib/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/examples/stereo.cpp -o CMakeFiles/stereo.dir/stereo.cpp.s

# Object files for target stereo
stereo_OBJECTS = \
"CMakeFiles/stereo.dir/stereo.cpp.o"

# External object files for target stereo
stereo_EXTERNAL_OBJECTS =

../bin/stereo: examples/CMakeFiles/stereo.dir/stereo.cpp.o
../bin/stereo: examples/CMakeFiles/stereo.dir/build.make
../bin/stereo: ../lib/libisam.a
../bin/stereo: /usr/lib/x86_64-linux-gnu/libcholmod.so
../bin/stereo: /usr/lib/x86_64-linux-gnu/libamd.so
../bin/stereo: /usr/lib/x86_64-linux-gnu/libcolamd.so
../bin/stereo: /usr/lib/x86_64-linux-gnu/libcamd.so
../bin/stereo: /usr/lib/x86_64-linux-gnu/libccolamd.so
../bin/stereo: examples/CMakeFiles/stereo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/stereo"
	cd /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/stereo.dir/build: ../bin/stereo

.PHONY : examples/CMakeFiles/stereo.dir/build

examples/CMakeFiles/stereo.dir/clean:
	cd /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/stereo.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/stereo.dir/clean

examples/CMakeFiles/stereo.dir/depend:
	cd /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/examples /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples /home/mines/ownCloud/MINES/Papiers/ICRA2019/github/simulations/isam/build/examples/CMakeFiles/stereo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/stereo.dir/depend

