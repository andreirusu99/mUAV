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
CMAKE_SOURCE_DIR = /home/andrei/Desktop/mUAV/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andrei/Desktop/mUAV/catkin_ws/build

# Utility rule file for drone_generate_messages_py.

# Include the progress variables for this target.
include drone/CMakeFiles/drone_generate_messages_py.dir/progress.make

drone/CMakeFiles/drone_generate_messages_py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_ControlAxes.py
drone/CMakeFiles/drone_generate_messages_py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_Attitude.py
drone/CMakeFiles/drone_generate_messages_py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/__init__.py


/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_ControlAxes.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_ControlAxes.py: /home/andrei/Desktop/mUAV/catkin_ws/src/drone/msg/ControlAxes.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrei/Desktop/mUAV/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Python from MSG drone/ControlAxes"
	cd /home/andrei/Desktop/mUAV/catkin_ws/build/drone && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/andrei/Desktop/mUAV/catkin_ws/src/drone/msg/ControlAxes.msg -Idrone:/home/andrei/Desktop/mUAV/catkin_ws/src/drone/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p drone -o /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg

/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_Attitude.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_Attitude.py: /home/andrei/Desktop/mUAV/catkin_ws/src/drone/msg/Attitude.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrei/Desktop/mUAV/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Python from MSG drone/Attitude"
	cd /home/andrei/Desktop/mUAV/catkin_ws/build/drone && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/andrei/Desktop/mUAV/catkin_ws/src/drone/msg/Attitude.msg -Idrone:/home/andrei/Desktop/mUAV/catkin_ws/src/drone/msg -Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg -p drone -o /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg

/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/__init__.py: /opt/ros/melodic/lib/genpy/genmsg_py.py
/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/__init__.py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_ControlAxes.py
/home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/__init__.py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_Attitude.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/andrei/Desktop/mUAV/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating Python msg __init__.py for drone"
	cd /home/andrei/Desktop/mUAV/catkin_ws/build/drone && ../catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py -o /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg --initpy

drone_generate_messages_py: drone/CMakeFiles/drone_generate_messages_py
drone_generate_messages_py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_ControlAxes.py
drone_generate_messages_py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/_Attitude.py
drone_generate_messages_py: /home/andrei/Desktop/mUAV/catkin_ws/devel/lib/python2.7/dist-packages/drone/msg/__init__.py
drone_generate_messages_py: drone/CMakeFiles/drone_generate_messages_py.dir/build.make

.PHONY : drone_generate_messages_py

# Rule to build all files generated by this target.
drone/CMakeFiles/drone_generate_messages_py.dir/build: drone_generate_messages_py

.PHONY : drone/CMakeFiles/drone_generate_messages_py.dir/build

drone/CMakeFiles/drone_generate_messages_py.dir/clean:
	cd /home/andrei/Desktop/mUAV/catkin_ws/build/drone && $(CMAKE_COMMAND) -P CMakeFiles/drone_generate_messages_py.dir/cmake_clean.cmake
.PHONY : drone/CMakeFiles/drone_generate_messages_py.dir/clean

drone/CMakeFiles/drone_generate_messages_py.dir/depend:
	cd /home/andrei/Desktop/mUAV/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andrei/Desktop/mUAV/catkin_ws/src /home/andrei/Desktop/mUAV/catkin_ws/src/drone /home/andrei/Desktop/mUAV/catkin_ws/build /home/andrei/Desktop/mUAV/catkin_ws/build/drone /home/andrei/Desktop/mUAV/catkin_ws/build/drone/CMakeFiles/drone_generate_messages_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : drone/CMakeFiles/drone_generate_messages_py.dir/depend
