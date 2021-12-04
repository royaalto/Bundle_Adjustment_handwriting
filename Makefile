# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles /home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/gim/softwares/Bundle_Adjustment_handwriting/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named ba_evaluate

# Build rule for target.
ba_evaluate: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ba_evaluate
.PHONY : ba_evaluate

# fast build rule for target.
ba_evaluate/fast:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/build
.PHONY : ba_evaluate/fast

src/ba_eigen.o: src/ba_eigen.cpp.o

.PHONY : src/ba_eigen.o

# target to build an object file
src/ba_eigen.cpp.o:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.o
.PHONY : src/ba_eigen.cpp.o

src/ba_eigen.i: src/ba_eigen.cpp.i

.PHONY : src/ba_eigen.i

# target to preprocess a source file
src/ba_eigen.cpp.i:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.i
.PHONY : src/ba_eigen.cpp.i

src/ba_eigen.s: src/ba_eigen.cpp.s

.PHONY : src/ba_eigen.s

# target to generate assembly for a file
src/ba_eigen.cpp.s:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_eigen.cpp.s
.PHONY : src/ba_eigen.cpp.s

src/ba_evaluate.o: src/ba_evaluate.cpp.o

.PHONY : src/ba_evaluate.o

# target to build an object file
src/ba_evaluate.cpp.o:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.o
.PHONY : src/ba_evaluate.cpp.o

src/ba_evaluate.i: src/ba_evaluate.cpp.i

.PHONY : src/ba_evaluate.i

# target to preprocess a source file
src/ba_evaluate.cpp.i:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.i
.PHONY : src/ba_evaluate.cpp.i

src/ba_evaluate.s: src/ba_evaluate.cpp.s

.PHONY : src/ba_evaluate.s

# target to generate assembly for a file
src/ba_evaluate.cpp.s:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_evaluate.cpp.s
.PHONY : src/ba_evaluate.cpp.s

src/ba_g2o.o: src/ba_g2o.cpp.o

.PHONY : src/ba_g2o.o

# target to build an object file
src/ba_g2o.cpp.o:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.o
.PHONY : src/ba_g2o.cpp.o

src/ba_g2o.i: src/ba_g2o.cpp.i

.PHONY : src/ba_g2o.i

# target to preprocess a source file
src/ba_g2o.cpp.i:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.i
.PHONY : src/ba_g2o.cpp.i

src/ba_g2o.s: src/ba_g2o.cpp.s

.PHONY : src/ba_g2o.s

# target to generate assembly for a file
src/ba_g2o.cpp.s:
	$(MAKE) -f CMakeFiles/ba_evaluate.dir/build.make CMakeFiles/ba_evaluate.dir/src/ba_g2o.cpp.s
.PHONY : src/ba_g2o.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... ba_evaluate"
	@echo "... src/ba_eigen.o"
	@echo "... src/ba_eigen.i"
	@echo "... src/ba_eigen.s"
	@echo "... src/ba_evaluate.o"
	@echo "... src/ba_evaluate.i"
	@echo "... src/ba_evaluate.s"
	@echo "... src/ba_g2o.o"
	@echo "... src/ba_g2o.i"
	@echo "... src/ba_g2o.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
