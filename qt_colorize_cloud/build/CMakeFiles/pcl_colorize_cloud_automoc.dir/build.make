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
CMAKE_SOURCE_DIR = /home/autonomoose/projects/qt_colorize_cloud

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/autonomoose/projects/qt_colorize_cloud/build

# Utility rule file for pcl_colorize_cloud_automoc.

# Include the progress variables for this target.
include CMakeFiles/pcl_colorize_cloud_automoc.dir/progress.make

CMakeFiles/pcl_colorize_cloud_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/autonomoose/projects/qt_colorize_cloud/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc, uic and rcc for target pcl_colorize_cloud"
	/usr/bin/cmake -E cmake_autogen /home/autonomoose/projects/qt_colorize_cloud/build/CMakeFiles/pcl_colorize_cloud_automoc.dir/ ""

pcl_colorize_cloud_automoc: CMakeFiles/pcl_colorize_cloud_automoc
pcl_colorize_cloud_automoc: CMakeFiles/pcl_colorize_cloud_automoc.dir/build.make

.PHONY : pcl_colorize_cloud_automoc

# Rule to build all files generated by this target.
CMakeFiles/pcl_colorize_cloud_automoc.dir/build: pcl_colorize_cloud_automoc

.PHONY : CMakeFiles/pcl_colorize_cloud_automoc.dir/build

CMakeFiles/pcl_colorize_cloud_automoc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pcl_colorize_cloud_automoc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pcl_colorize_cloud_automoc.dir/clean

CMakeFiles/pcl_colorize_cloud_automoc.dir/depend:
	cd /home/autonomoose/projects/qt_colorize_cloud/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/autonomoose/projects/qt_colorize_cloud /home/autonomoose/projects/qt_colorize_cloud /home/autonomoose/projects/qt_colorize_cloud/build /home/autonomoose/projects/qt_colorize_cloud/build /home/autonomoose/projects/qt_colorize_cloud/build/CMakeFiles/pcl_colorize_cloud_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pcl_colorize_cloud_automoc.dir/depend

