# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build

# Include any dependencies generated for this target.
include CMakeFiles/SERIALECHOEX.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SERIALECHOEX.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SERIALECHOEX.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SERIALECHOEX.dir/flags.make

CMakeFiles/SERIALECHOEX.dir/serialecho.c.o: CMakeFiles/SERIALECHOEX.dir/flags.make
CMakeFiles/SERIALECHOEX.dir/serialecho.c.o: /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/serialecho.c
CMakeFiles/SERIALECHOEX.dir/serialecho.c.o: CMakeFiles/SERIALECHOEX.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/SERIALECHOEX.dir/serialecho.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/SERIALECHOEX.dir/serialecho.c.o -MF CMakeFiles/SERIALECHOEX.dir/serialecho.c.o.d -o CMakeFiles/SERIALECHOEX.dir/serialecho.c.o -c /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/serialecho.c

CMakeFiles/SERIALECHOEX.dir/serialecho.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/SERIALECHOEX.dir/serialecho.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/serialecho.c > CMakeFiles/SERIALECHOEX.dir/serialecho.c.i

CMakeFiles/SERIALECHOEX.dir/serialecho.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/SERIALECHOEX.dir/serialecho.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/serialecho.c -o CMakeFiles/SERIALECHOEX.dir/serialecho.c.s

# Object files for target SERIALECHOEX
SERIALECHOEX_OBJECTS = \
"CMakeFiles/SERIALECHOEX.dir/serialecho.c.o"

# External object files for target SERIALECHOEX
SERIALECHOEX_EXTERNAL_OBJECTS =

serialecho: CMakeFiles/SERIALECHOEX.dir/serialecho.c.o
serialecho: CMakeFiles/SERIALECHOEX.dir/build.make
serialecho: CMakeFiles/SERIALECHOEX.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable serialecho"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SERIALECHOEX.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SERIALECHOEX.dir/build: serialecho
.PHONY : CMakeFiles/SERIALECHOEX.dir/build

CMakeFiles/SERIALECHOEX.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SERIALECHOEX.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SERIALECHOEX.dir/clean

CMakeFiles/SERIALECHOEX.dir/depend:
	cd /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build /home/INTRANET.SALZ.GMBH/christopher.tan/code-projects/serial_echo/build/CMakeFiles/SERIALECHOEX.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SERIALECHOEX.dir/depend
