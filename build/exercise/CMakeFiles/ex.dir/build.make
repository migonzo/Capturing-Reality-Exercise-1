# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/cmake/1088/bin/cmake

# The command to remove a file.
RM = /snap/cmake/1088/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build"

# Include any dependencies generated for this target.
include exercise/CMakeFiles/ex.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include exercise/CMakeFiles/ex.dir/compiler_depend.make

# Include the progress variables for this target.
include exercise/CMakeFiles/ex.dir/progress.make

# Include the compile flags for this target's objects.
include exercise/CMakeFiles/ex.dir/flags.make

exercise/CMakeFiles/ex.dir/main.cc.o: exercise/CMakeFiles/ex.dir/flags.make
exercise/CMakeFiles/ex.dir/main.cc.o: ../exercise/main.cc
exercise/CMakeFiles/ex.dir/main.cc.o: exercise/CMakeFiles/ex.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object exercise/CMakeFiles/ex.dir/main.cc.o"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exercise/CMakeFiles/ex.dir/main.cc.o -MF CMakeFiles/ex.dir/main.cc.o.d -o CMakeFiles/ex.dir/main.cc.o -c "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise/main.cc"

exercise/CMakeFiles/ex.dir/main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ex.dir/main.cc.i"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise/main.cc" > CMakeFiles/ex.dir/main.cc.i

exercise/CMakeFiles/ex.dir/main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ex.dir/main.cc.s"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise/main.cc" -o CMakeFiles/ex.dir/main.cc.s

exercise/CMakeFiles/ex.dir/ply.cc.o: exercise/CMakeFiles/ex.dir/flags.make
exercise/CMakeFiles/ex.dir/ply.cc.o: ../exercise/ply.cc
exercise/CMakeFiles/ex.dir/ply.cc.o: exercise/CMakeFiles/ex.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object exercise/CMakeFiles/ex.dir/ply.cc.o"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT exercise/CMakeFiles/ex.dir/ply.cc.o -MF CMakeFiles/ex.dir/ply.cc.o.d -o CMakeFiles/ex.dir/ply.cc.o -c "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise/ply.cc"

exercise/CMakeFiles/ex.dir/ply.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ex.dir/ply.cc.i"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise/ply.cc" > CMakeFiles/ex.dir/ply.cc.i

exercise/CMakeFiles/ex.dir/ply.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ex.dir/ply.cc.s"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise/ply.cc" -o CMakeFiles/ex.dir/ply.cc.s

# Object files for target ex
ex_OBJECTS = \
"CMakeFiles/ex.dir/main.cc.o" \
"CMakeFiles/ex.dir/ply.cc.o"

# External object files for target ex
ex_EXTERNAL_OBJECTS =

exercise/ex: exercise/CMakeFiles/ex.dir/main.cc.o
exercise/ex: exercise/CMakeFiles/ex.dir/ply.cc.o
exercise/ex: exercise/CMakeFiles/ex.dir/build.make
exercise/ex: image/libimage.a
exercise/ex: /usr/local/lib/libceres.a
exercise/ex: /usr/lib/x86_64-linux-gnu/libpng.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libz.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libjpeg.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libHalf.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libIex.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libImath.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libIlmImf.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libIlmThread.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libglog.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1
exercise/ex: /usr/lib/x86_64-linux-gnu/libspqr.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libcholmod.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libmetis.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libamd.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libcamd.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libccolamd.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libcolamd.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libcxsparse.so
exercise/ex: /usr/lib/x86_64-linux-gnu/liblapack.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libblas.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libf77blas.so
exercise/ex: /usr/lib/x86_64-linux-gnu/libatlas.so
exercise/ex: exercise/CMakeFiles/ex.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ex"
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ex.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
exercise/CMakeFiles/ex.dir/build: exercise/ex
.PHONY : exercise/CMakeFiles/ex.dir/build

exercise/CMakeFiles/ex.dir/clean:
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" && $(CMAKE_COMMAND) -P CMakeFiles/ex.dir/cmake_clean.cmake
.PHONY : exercise/CMakeFiles/ex.dir/clean

exercise/CMakeFiles/ex.dir/depend:
	cd "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1" "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/exercise" "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build" "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise" "/home/arthur/Documents/Darmstadt/Capturing Reality/ep2/Capturing-Reality-Exercise-1/build/exercise/CMakeFiles/ex.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : exercise/CMakeFiles/ex.dir/depend
