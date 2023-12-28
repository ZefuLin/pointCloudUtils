# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zefu/Projects/pointCloudUtils

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zefu/Projects/pointCloudUtils/build

# Include any dependencies generated for this target.
include CMakeFiles/pointCloudUtils.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pointCloudUtils.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pointCloudUtils.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pointCloudUtils.dir/flags.make

CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o: CMakeFiles/pointCloudUtils.dir/flags.make
CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o: ../src/ICP2D.cpp
CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o: CMakeFiles/pointCloudUtils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o -MF CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o.d -o CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o -c /home/zefu/Projects/pointCloudUtils/src/ICP2D.cpp

CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zefu/Projects/pointCloudUtils/src/ICP2D.cpp > CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.i

CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zefu/Projects/pointCloudUtils/src/ICP2D.cpp -o CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.s

CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o: CMakeFiles/pointCloudUtils.dir/flags.make
CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o: ../src/icp.cpp
CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o: CMakeFiles/pointCloudUtils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o -MF CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o.d -o CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o -c /home/zefu/Projects/pointCloudUtils/src/icp.cpp

CMakeFiles/pointCloudUtils.dir/src/icp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointCloudUtils.dir/src/icp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zefu/Projects/pointCloudUtils/src/icp.cpp > CMakeFiles/pointCloudUtils.dir/src/icp.cpp.i

CMakeFiles/pointCloudUtils.dir/src/icp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointCloudUtils.dir/src/icp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zefu/Projects/pointCloudUtils/src/icp.cpp -o CMakeFiles/pointCloudUtils.dir/src/icp.cpp.s

CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o: CMakeFiles/pointCloudUtils.dir/flags.make
CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o: ../src/icpPointToPoint.cpp
CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o: CMakeFiles/pointCloudUtils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o -MF CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o.d -o CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o -c /home/zefu/Projects/pointCloudUtils/src/icpPointToPoint.cpp

CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zefu/Projects/pointCloudUtils/src/icpPointToPoint.cpp > CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.i

CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zefu/Projects/pointCloudUtils/src/icpPointToPoint.cpp -o CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.s

CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o: CMakeFiles/pointCloudUtils.dir/flags.make
CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o: ../src/kdtree.cpp
CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o: CMakeFiles/pointCloudUtils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o -MF CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o.d -o CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o -c /home/zefu/Projects/pointCloudUtils/src/kdtree.cpp

CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zefu/Projects/pointCloudUtils/src/kdtree.cpp > CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.i

CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zefu/Projects/pointCloudUtils/src/kdtree.cpp -o CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.s

CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o: CMakeFiles/pointCloudUtils.dir/flags.make
CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o: ../src/matrix.cpp
CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o: CMakeFiles/pointCloudUtils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o -MF CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o.d -o CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o -c /home/zefu/Projects/pointCloudUtils/src/matrix.cpp

CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zefu/Projects/pointCloudUtils/src/matrix.cpp > CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.i

CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zefu/Projects/pointCloudUtils/src/matrix.cpp -o CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.s

CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o: CMakeFiles/pointCloudUtils.dir/flags.make
CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o: ../src/pointCloud.cpp
CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o: CMakeFiles/pointCloudUtils.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o -MF CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o.d -o CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o -c /home/zefu/Projects/pointCloudUtils/src/pointCloud.cpp

CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zefu/Projects/pointCloudUtils/src/pointCloud.cpp > CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.i

CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zefu/Projects/pointCloudUtils/src/pointCloud.cpp -o CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.s

# Object files for target pointCloudUtils
pointCloudUtils_OBJECTS = \
"CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o" \
"CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o" \
"CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o" \
"CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o" \
"CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o" \
"CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o"

# External object files for target pointCloudUtils
pointCloudUtils_EXTERNAL_OBJECTS =

libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/src/ICP2D.cpp.o
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/src/icp.cpp.o
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/src/icpPointToPoint.cpp.o
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/src/kdtree.cpp.o
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/src/matrix.cpp.o
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/src/pointCloud.cpp.o
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/build.make
libpointCloudUtils.a: CMakeFiles/pointCloudUtils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zefu/Projects/pointCloudUtils/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libpointCloudUtils.a"
	$(CMAKE_COMMAND) -P CMakeFiles/pointCloudUtils.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pointCloudUtils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pointCloudUtils.dir/build: libpointCloudUtils.a
.PHONY : CMakeFiles/pointCloudUtils.dir/build

CMakeFiles/pointCloudUtils.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pointCloudUtils.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pointCloudUtils.dir/clean

CMakeFiles/pointCloudUtils.dir/depend:
	cd /home/zefu/Projects/pointCloudUtils/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zefu/Projects/pointCloudUtils /home/zefu/Projects/pointCloudUtils /home/zefu/Projects/pointCloudUtils/build /home/zefu/Projects/pointCloudUtils/build /home/zefu/Projects/pointCloudUtils/build/CMakeFiles/pointCloudUtils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pointCloudUtils.dir/depend

