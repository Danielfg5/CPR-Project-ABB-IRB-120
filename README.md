# CPR-Project-ABB-IRB-120

Final project for the course "Control y Programación de Robots" at Universidad de Sevilla.

It consists of the development of a "Par Computado" inverse dynamics controller in ROS, then applied and tested on a simulated ABB IRB 120 manipulator robot.

## Dependencies:

This project depends on the following packages and libraries, which are easily installed with apt:

- ros-melodic-desktop-full
- ros-melodic-industrial-core
- ros-melodic-industrial-msgs
- ros-melodic-industrial-robot-client
- ros-melodic-industrial-robot-simulator
- ros-melodic-industrial-utils
- ros-melodic-ros-control
- ros-melodic-joint-trajectory-controller
- ros-melodic-abb
- ros-melodic-moveit
- ros-melodic-orocos-kdl
- libeigen3-dev

## How to install

1. Clone the folder CPR_project into the `src` folder of your catkin workspace.
2. Inside your catkin workspace root, launch `catkin_make`.

## How to launch the IRB 120 simulation

After installing and compiling the package, run `roslaunch irb120_moveit_config my_moveit_gazebo.launch`.

To control the position of the robot, you can use the Moveit interface inside Rviz.

You can also manually control each joint, by executing `rostopic pub -1 /arm_group_controller/qX_des_command std_msgs/Float64 "data: Y"`, substituting X with the joint number (1-6) and Y with the desired joint position in (rad).

### How to adjust controller parameters

The parameters for the controller in this simulation are located in `irb120_moveit_config/config/ros_controllers.yaml`

- To adjust the gains, modify the `p`,`i`,`d` values under `joint_*_control_params`.

- To change the controller from Par Computado to PID:
	1. Comment out the line `type: par_computado_6dof/ParComputado6Dof` and uncomment `type: my_pid_6dof/MyPid6Dof`
	2. Comment out the first set of `joint_X_control_params` and uncomment the second set.

### How to test the effect of model uncertainties in the Par Computado controller

To test the effect of model uncertainties in the Par Computado controller, open the source code of the controller, located in `par_computado_6dof/src/par_computado_6dof.cpp`.

Then, inside the `void update()` function, uncomment the lines under the section "ROBUSTNESS TEST".

## How to launch the 1-DOF pendulum simulation

After installing and compiling the package, run `roslaunch my_1dof_robot_moveit_config my_moveit_gazebo.launch`.

To control the position of the pendulum, you can use the Moveit interface inside Rviz.

You can also manually control the joint, by executing `rostopic pub -1 /pendulum_controller/q_des_command std_msgs/Float64 "data: Y"`, substituting Y with the desired joint position in (rad).

### How to adjust controller parameters

The parameters for the controller in this simulation are located in `my_1dof_robot_moveit_config/config/ros_controllers.yaml`

To adjust the gains, modify the `p`,`i`,`d` values under `joint_1_control_params`.

To change the controller from Par Computado to PID, change the line `type: par_computado_1dof/ParComputado1Dof` to `type: my_pid_controller1/MyPidController1`.

To test the effect of model uncertainties in the Par Computado controller, comment out the first set of `model_params` and uncomment the second one, which contains perturbed values.

## Project folder structure

- `par_computado_6dof`: it contains the code for the 6-DOF Par Computado controller we developed. Relevant files:
	- `par_computado_6dof.cpp`, inside `src`: it contains the source code of the controller
- `irb120_moveit_config`: it contains the Moveit! configuration for the simulation of the ABB IRB 120 robot. Most relevant files:
	- `my_moveit_gazebo.launch`, inside `launch`: main launch file for the simulation
	- `ros_controllers.yaml`, inside `config`: it contains the parameters for the joints controller
	- `abb_irb120_model.srdf`, inside `config`: it contains the standard test poses we defined
- `irb120`: it contains the simulation model of the ABB IRB 120 robot. Most relevant files:
	- `irb120.urdf`, inside `urdf` folder: it contains the URDF description of the robot
- `par_computado_1dof`: it contains the code for the 1-DOF Par Computado controller we developed. Relevant files:
	- `par_computado_1dof.cpp`, inside `src`: it contains the source code of the controller
- `my_1dof_robot`: it contains the simulation model of the 1-DOF pendulum robot
- `my_1dof_robot_moveit_config`: it contains the Moveit! configuration for the simulation of the 1-DOF pendulum robot
- `my_pid_6dof`: it contains the code for the 6-DOF PID controller we developed. Relevant files:
	- `my_pid_6dof.cpp`, inside `src`: it contains the source code of the controller
- `my_pid_controller1`: it contains the code for the 1-DOF PID controller we developed. Relevant files:
	- `my_pid_controller1.cpp`, inside `src`: it contains the source code of the controller
- `my_custom_msgs`: it contains the custom message `SixJointValues` we defined to send 6 joint quantities in a single message


