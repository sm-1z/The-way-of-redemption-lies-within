# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/aliykb/Documents/openloong-dyn-control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aliykb/Documents/openloong-dyn-control/build

# Include any dependencies generated for this target.
include CMakeFiles/core.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/core.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/core.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/core.dir/flags.make

CMakeFiles/core.dir/algorithm/foot_placement.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/foot_placement.cpp.o: ../algorithm/foot_placement.cpp
CMakeFiles/core.dir/algorithm/foot_placement.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/core.dir/algorithm/foot_placement.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/foot_placement.cpp.o -MF CMakeFiles/core.dir/algorithm/foot_placement.cpp.o.d -o CMakeFiles/core.dir/algorithm/foot_placement.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/foot_placement.cpp

CMakeFiles/core.dir/algorithm/foot_placement.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/foot_placement.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/foot_placement.cpp > CMakeFiles/core.dir/algorithm/foot_placement.cpp.i

CMakeFiles/core.dir/algorithm/foot_placement.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/foot_placement.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/foot_placement.cpp -o CMakeFiles/core.dir/algorithm/foot_placement.cpp.s

CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o: ../algorithm/gait_scheduler.cpp
CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o -MF CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o.d -o CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/gait_scheduler.cpp

CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/gait_scheduler.cpp > CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.i

CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/gait_scheduler.cpp -o CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.s

CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o: ../algorithm/joystick_interpreter.cpp
CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o -MF CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o.d -o CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/joystick_interpreter.cpp

CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/joystick_interpreter.cpp > CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.i

CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/joystick_interpreter.cpp -o CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.s

CMakeFiles/core.dir/algorithm/mpc.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/mpc.cpp.o: ../algorithm/mpc.cpp
CMakeFiles/core.dir/algorithm/mpc.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/core.dir/algorithm/mpc.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/mpc.cpp.o -MF CMakeFiles/core.dir/algorithm/mpc.cpp.o.d -o CMakeFiles/core.dir/algorithm/mpc.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/mpc.cpp

CMakeFiles/core.dir/algorithm/mpc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/mpc.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/mpc.cpp > CMakeFiles/core.dir/algorithm/mpc.cpp.i

CMakeFiles/core.dir/algorithm/mpc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/mpc.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/mpc.cpp -o CMakeFiles/core.dir/algorithm/mpc.cpp.s

CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o: ../algorithm/pino_kin_dyn.cpp
CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o -MF CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o.d -o CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/pino_kin_dyn.cpp

CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/pino_kin_dyn.cpp > CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.i

CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/pino_kin_dyn.cpp -o CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.s

CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o: ../algorithm/priority_tasks.cpp
CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o -MF CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o.d -o CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/priority_tasks.cpp

CMakeFiles/core.dir/algorithm/priority_tasks.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/priority_tasks.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/priority_tasks.cpp > CMakeFiles/core.dir/algorithm/priority_tasks.cpp.i

CMakeFiles/core.dir/algorithm/priority_tasks.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/priority_tasks.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/priority_tasks.cpp -o CMakeFiles/core.dir/algorithm/priority_tasks.cpp.s

CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o: ../algorithm/wbc_priority.cpp
CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o -MF CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o.d -o CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/algorithm/wbc_priority.cpp

CMakeFiles/core.dir/algorithm/wbc_priority.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/algorithm/wbc_priority.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/algorithm/wbc_priority.cpp > CMakeFiles/core.dir/algorithm/wbc_priority.cpp.i

CMakeFiles/core.dir/algorithm/wbc_priority.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/algorithm/wbc_priority.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/algorithm/wbc_priority.cpp -o CMakeFiles/core.dir/algorithm/wbc_priority.cpp.s

CMakeFiles/core.dir/common/PVT_ctrl.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/common/PVT_ctrl.cpp.o: ../common/PVT_ctrl.cpp
CMakeFiles/core.dir/common/PVT_ctrl.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/core.dir/common/PVT_ctrl.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/common/PVT_ctrl.cpp.o -MF CMakeFiles/core.dir/common/PVT_ctrl.cpp.o.d -o CMakeFiles/core.dir/common/PVT_ctrl.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/common/PVT_ctrl.cpp

CMakeFiles/core.dir/common/PVT_ctrl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/common/PVT_ctrl.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/common/PVT_ctrl.cpp > CMakeFiles/core.dir/common/PVT_ctrl.cpp.i

CMakeFiles/core.dir/common/PVT_ctrl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/common/PVT_ctrl.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/common/PVT_ctrl.cpp -o CMakeFiles/core.dir/common/PVT_ctrl.cpp.s

CMakeFiles/core.dir/common/data_logger.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/common/data_logger.cpp.o: ../common/data_logger.cpp
CMakeFiles/core.dir/common/data_logger.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/core.dir/common/data_logger.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/common/data_logger.cpp.o -MF CMakeFiles/core.dir/common/data_logger.cpp.o.d -o CMakeFiles/core.dir/common/data_logger.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/common/data_logger.cpp

CMakeFiles/core.dir/common/data_logger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/common/data_logger.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/common/data_logger.cpp > CMakeFiles/core.dir/common/data_logger.cpp.i

CMakeFiles/core.dir/common/data_logger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/common/data_logger.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/common/data_logger.cpp -o CMakeFiles/core.dir/common/data_logger.cpp.s

CMakeFiles/core.dir/math/LPF_fst.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/math/LPF_fst.cpp.o: ../math/LPF_fst.cpp
CMakeFiles/core.dir/math/LPF_fst.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/core.dir/math/LPF_fst.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/math/LPF_fst.cpp.o -MF CMakeFiles/core.dir/math/LPF_fst.cpp.o.d -o CMakeFiles/core.dir/math/LPF_fst.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/math/LPF_fst.cpp

CMakeFiles/core.dir/math/LPF_fst.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/math/LPF_fst.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/math/LPF_fst.cpp > CMakeFiles/core.dir/math/LPF_fst.cpp.i

CMakeFiles/core.dir/math/LPF_fst.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/math/LPF_fst.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/math/LPF_fst.cpp -o CMakeFiles/core.dir/math/LPF_fst.cpp.s

CMakeFiles/core.dir/math/bezier_1D.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/math/bezier_1D.cpp.o: ../math/bezier_1D.cpp
CMakeFiles/core.dir/math/bezier_1D.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/core.dir/math/bezier_1D.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/math/bezier_1D.cpp.o -MF CMakeFiles/core.dir/math/bezier_1D.cpp.o.d -o CMakeFiles/core.dir/math/bezier_1D.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/math/bezier_1D.cpp

CMakeFiles/core.dir/math/bezier_1D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/math/bezier_1D.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/math/bezier_1D.cpp > CMakeFiles/core.dir/math/bezier_1D.cpp.i

CMakeFiles/core.dir/math/bezier_1D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/math/bezier_1D.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/math/bezier_1D.cpp -o CMakeFiles/core.dir/math/bezier_1D.cpp.s

CMakeFiles/core.dir/math/ramp_trajectory.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/math/ramp_trajectory.cpp.o: ../math/ramp_trajectory.cpp
CMakeFiles/core.dir/math/ramp_trajectory.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/core.dir/math/ramp_trajectory.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/math/ramp_trajectory.cpp.o -MF CMakeFiles/core.dir/math/ramp_trajectory.cpp.o.d -o CMakeFiles/core.dir/math/ramp_trajectory.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/math/ramp_trajectory.cpp

CMakeFiles/core.dir/math/ramp_trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/math/ramp_trajectory.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/math/ramp_trajectory.cpp > CMakeFiles/core.dir/math/ramp_trajectory.cpp.i

CMakeFiles/core.dir/math/ramp_trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/math/ramp_trajectory.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/math/ramp_trajectory.cpp -o CMakeFiles/core.dir/math/ramp_trajectory.cpp.s

CMakeFiles/core.dir/math/useful_math.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/math/useful_math.cpp.o: ../math/useful_math.cpp
CMakeFiles/core.dir/math/useful_math.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/core.dir/math/useful_math.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/math/useful_math.cpp.o -MF CMakeFiles/core.dir/math/useful_math.cpp.o.d -o CMakeFiles/core.dir/math/useful_math.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/math/useful_math.cpp

CMakeFiles/core.dir/math/useful_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/math/useful_math.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/math/useful_math.cpp > CMakeFiles/core.dir/math/useful_math.cpp.i

CMakeFiles/core.dir/math/useful_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/math/useful_math.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/math/useful_math.cpp -o CMakeFiles/core.dir/math/useful_math.cpp.s

CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o: ../sim_interface/GLFW_callbacks.cpp
CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o -MF CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o.d -o CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/sim_interface/GLFW_callbacks.cpp

CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/sim_interface/GLFW_callbacks.cpp > CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.i

CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/sim_interface/GLFW_callbacks.cpp -o CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.s

CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o: CMakeFiles/core.dir/flags.make
CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o: ../sim_interface/MJ_interface.cpp
CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o: CMakeFiles/core.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o -MF CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o.d -o CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o -c /home/aliykb/Documents/openloong-dyn-control/sim_interface/MJ_interface.cpp

CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aliykb/Documents/openloong-dyn-control/sim_interface/MJ_interface.cpp > CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.i

CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aliykb/Documents/openloong-dyn-control/sim_interface/MJ_interface.cpp -o CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.s

# Object files for target core
core_OBJECTS = \
"CMakeFiles/core.dir/algorithm/foot_placement.cpp.o" \
"CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o" \
"CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o" \
"CMakeFiles/core.dir/algorithm/mpc.cpp.o" \
"CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o" \
"CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o" \
"CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o" \
"CMakeFiles/core.dir/common/PVT_ctrl.cpp.o" \
"CMakeFiles/core.dir/common/data_logger.cpp.o" \
"CMakeFiles/core.dir/math/LPF_fst.cpp.o" \
"CMakeFiles/core.dir/math/bezier_1D.cpp.o" \
"CMakeFiles/core.dir/math/ramp_trajectory.cpp.o" \
"CMakeFiles/core.dir/math/useful_math.cpp.o" \
"CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o" \
"CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o"

# External object files for target core
core_EXTERNAL_OBJECTS =

libcore.a: CMakeFiles/core.dir/algorithm/foot_placement.cpp.o
libcore.a: CMakeFiles/core.dir/algorithm/gait_scheduler.cpp.o
libcore.a: CMakeFiles/core.dir/algorithm/joystick_interpreter.cpp.o
libcore.a: CMakeFiles/core.dir/algorithm/mpc.cpp.o
libcore.a: CMakeFiles/core.dir/algorithm/pino_kin_dyn.cpp.o
libcore.a: CMakeFiles/core.dir/algorithm/priority_tasks.cpp.o
libcore.a: CMakeFiles/core.dir/algorithm/wbc_priority.cpp.o
libcore.a: CMakeFiles/core.dir/common/PVT_ctrl.cpp.o
libcore.a: CMakeFiles/core.dir/common/data_logger.cpp.o
libcore.a: CMakeFiles/core.dir/math/LPF_fst.cpp.o
libcore.a: CMakeFiles/core.dir/math/bezier_1D.cpp.o
libcore.a: CMakeFiles/core.dir/math/ramp_trajectory.cpp.o
libcore.a: CMakeFiles/core.dir/math/useful_math.cpp.o
libcore.a: CMakeFiles/core.dir/sim_interface/GLFW_callbacks.cpp.o
libcore.a: CMakeFiles/core.dir/sim_interface/MJ_interface.cpp.o
libcore.a: CMakeFiles/core.dir/build.make
libcore.a: CMakeFiles/core.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Linking CXX static library libcore.a"
	$(CMAKE_COMMAND) -P CMakeFiles/core.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/core.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/core.dir/build: libcore.a
.PHONY : CMakeFiles/core.dir/build

CMakeFiles/core.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/core.dir/cmake_clean.cmake
.PHONY : CMakeFiles/core.dir/clean

CMakeFiles/core.dir/depend:
	cd /home/aliykb/Documents/openloong-dyn-control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aliykb/Documents/openloong-dyn-control /home/aliykb/Documents/openloong-dyn-control /home/aliykb/Documents/openloong-dyn-control/build /home/aliykb/Documents/openloong-dyn-control/build /home/aliykb/Documents/openloong-dyn-control/build/CMakeFiles/core.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/core.dir/depend

