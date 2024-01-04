# CPR-Project-ABB-IRB-120

Proyecto de la asignatura Control y Programación de Robots del grado GIERM. Consiste en el control por par computado de un manipulador ABB de 6 gdl

## How to install:

Copy folder CPR_project into {ros_workspace}/src.  
Then, inside {ros_workspace}, launch:  
    catkin_make

## How to launch 1-dof pendulum simulation (with moveit motion planning):

	roslaunch my_1dof_robot_moveit_config my_moveit_gazebo.launch  
The parameters for the controller in this simulation are located in "my_1dof_robot_moveit_config/config/ros_controllers.yaml"
The joint can also be manually controlled by executing:  

	rostopic pub -1 /pendulum_controller/q_des_command std_msgs/Float64 "data: Y"

substituting Y with the joint position (rad)

## How to launch IRB120 simulation (with moveit motion planning):

	roslaunch irb120_moveit_config my_moveit_gazebo.launch  
The parameters for the controller in this simulation are located in "irb120_moveit_config/config/ros_controllers.yaml"
The joints can also be manually controlled by executing:  

	rostopic pub -1 /arm_group_controller/qX_des_command std_msgs/Float64 "data: Y"

substituting X with the joint number (1-6) and Y with the joint position (rad)

## How to launch IRB120 simulation (manual control of individual joints only, no motion planning):

	roslaunch irb120 irb120.launch
The parameters for the controller in this simulation are located in "irb120/config/joints_controllers.yaml"  

The joints can be manually controlled by executing:  

	rostopic pub -1 /irb120/joints_positions_controller/qX_des_command std_msgs/Float64 "data: Y"

substituting X with the joint number (1-6) and Y with the joint position (rad)
