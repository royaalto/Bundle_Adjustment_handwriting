# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/gim/softwares/Bundle_Adjustment_handwriting

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gim/softwares/Bundle_Adjustment_handwriting

# Include any dependencies generated for this target.
include CMakeFiles/ba_evaluate.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ba_evaluate.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ba_evaluate.dir/flags.make

CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o: CMakeFiles/ba_evaluate.dir/flags.make
CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o: src/ba_eigen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o -c /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_eigen.cpp

CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_eigen.cpp > CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.i

CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_eigen.cpp -o CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.s

CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.requires:

.PHONY : CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.requires

CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.provides: CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.requires
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.provides.build
.PHONY : CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.provides

CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.provides.build: CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o


CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o: CMakeFiles/ba_evaluate.dir/flags.make
CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o: src/ba_g2o.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o -c /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_g2o.cpp

CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_g2o.cpp > CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.i

CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_g2o.cpp -o CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.s

CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.requires:

.PHONY : CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.requires

CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.provides: CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.requires
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.provides.build
.PHONY : CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.provides

CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.provides.build: CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o


CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o: CMakeFiles/ba_evaluate.dir/flags.make
CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o: src/ba_evaluate.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o -c /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_evaluate.cpp

CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_evaluate.cpp > CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.i

CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gim/softwares/Bundle_Adjustment_handwriting/src/ba_evaluate.cpp -o CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.s

CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.requires:

.PHONY : CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.requires

CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.provides: CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.requires
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.provides.build
.PHONY : CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.provides

CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.provides.build: CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o


# Object files for target ba_evaluate
ba_evaluate_OBJECTS = \
"CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o" \
"CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o" \
"CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o"

# External object files for target ba_evaluate
ba_evaluate_EXTERNAL_OBJECTS =

ba_evaluate: CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o
ba_evaluate: CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o
ba_evaluate: CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o
ba_evaluate: CMakeFiles/ba_evaluate.dir/build.make
ba_evaluate: CMakeFiles/ba_evaluate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ba_evaluate"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ba_evaluate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ba_evaluate.dir/build: ba_evaluate

.PHONY : CMakeFiles/ba_evaluate.dir/build

CMakeFiles/ba_evaluate.dir/requires: CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o.requires
CMakeFiles/ba_evaluate.dir/requires: CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o.requires
CMakeFiles/ba_evaluate.dir/requires: CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o.requires

.PHONY : CMakeFiles/ba_evaluate.dir/requires

CMakeFiles/ba_evaluate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ba_evaluate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ba_evaluate.dir/clean

CMakeFiles/ba_evaluate.dir/depend:
	cd /home/gim/softwares/Bundle_Adjustment_handwriting && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gim/softwares/Bundle_Adjustment_handwriting /home/gim/softwares/Bundle_Adjustment_handwriting /home/gim/softwares/Bundle_Adjustment_handwriting /home/gim/softwares/Bundle_Adjustment_handwriting /home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles/ba_evaluate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ba_evaluate.dir/depend

