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
CMAKE_SOURCE_DIR = /home/ieee/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ieee/catkin_ws/build

# Utility rule file for ocams_gencfg.

# Include the progress variables for this target.
include ocams/CMakeFiles/ocams_gencfg.dir/progress.make

ocams/CMakeFiles/ocams_gencfg: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h
ocams/CMakeFiles/ocams_gencfg: /home/ieee/catkin_ws/devel/lib/python2.7/dist-packages/ocams/cfg/camConfig.py


/home/ieee/catkin_ws/devel/include/ocams/camConfig.h: /home/ieee/catkin_ws/src/ocams/cfg/cam.cfg
/home/ieee/catkin_ws/devel/include/ocams/camConfig.h: /opt/ros/melodic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/ieee/catkin_ws/devel/include/ocams/camConfig.h: /opt/ros/melodic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ieee/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/cam.cfg: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h /home/ieee/catkin_ws/devel/lib/python2.7/dist-packages/ocams/cfg/camConfig.py"
	cd /home/ieee/catkin_ws/build/ocams && ../catkin_generated/env_cached.sh /home/ieee/catkin_ws/build/ocams/setup_custom_pythonpath.sh /home/ieee/catkin_ws/src/ocams/cfg/cam.cfg /opt/ros/melodic/share/dynamic_reconfigure/cmake/.. /home/ieee/catkin_ws/devel/share/ocams /home/ieee/catkin_ws/devel/include/ocams /home/ieee/catkin_ws/devel/lib/python2.7/dist-packages/ocams

/home/ieee/catkin_ws/devel/share/ocams/docs/camConfig.dox: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ieee/catkin_ws/devel/share/ocams/docs/camConfig.dox

/home/ieee/catkin_ws/devel/share/ocams/docs/camConfig-usage.dox: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ieee/catkin_ws/devel/share/ocams/docs/camConfig-usage.dox

/home/ieee/catkin_ws/devel/lib/python2.7/dist-packages/ocams/cfg/camConfig.py: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ieee/catkin_ws/devel/lib/python2.7/dist-packages/ocams/cfg/camConfig.py

/home/ieee/catkin_ws/devel/share/ocams/docs/camConfig.wikidoc: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/ieee/catkin_ws/devel/share/ocams/docs/camConfig.wikidoc

ocams_gencfg: ocams/CMakeFiles/ocams_gencfg
ocams_gencfg: /home/ieee/catkin_ws/devel/include/ocams/camConfig.h
ocams_gencfg: /home/ieee/catkin_ws/devel/share/ocams/docs/camConfig.dox
ocams_gencfg: /home/ieee/catkin_ws/devel/share/ocams/docs/camConfig-usage.dox
ocams_gencfg: /home/ieee/catkin_ws/devel/lib/python2.7/dist-packages/ocams/cfg/camConfig.py
ocams_gencfg: /home/ieee/catkin_ws/devel/share/ocams/docs/camConfig.wikidoc
ocams_gencfg: ocams/CMakeFiles/ocams_gencfg.dir/build.make

.PHONY : ocams_gencfg

# Rule to build all files generated by this target.
ocams/CMakeFiles/ocams_gencfg.dir/build: ocams_gencfg

.PHONY : ocams/CMakeFiles/ocams_gencfg.dir/build

ocams/CMakeFiles/ocams_gencfg.dir/clean:
	cd /home/ieee/catkin_ws/build/ocams && $(CMAKE_COMMAND) -P CMakeFiles/ocams_gencfg.dir/cmake_clean.cmake
.PHONY : ocams/CMakeFiles/ocams_gencfg.dir/clean

ocams/CMakeFiles/ocams_gencfg.dir/depend:
	cd /home/ieee/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ieee/catkin_ws/src /home/ieee/catkin_ws/src/ocams /home/ieee/catkin_ws/build /home/ieee/catkin_ws/build/ocams /home/ieee/catkin_ws/build/ocams/CMakeFiles/ocams_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ocams/CMakeFiles/ocams_gencfg.dir/depend

