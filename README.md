# CPR-Project-ABB-IRB-120

Proyecto de la asignatura Control y Programación de Robots del grado GIERM. Consiste en el control por par computado de un manipulador ABB de 6 gdl

## How to install:

Copy folder CPR_project into {ros_workspace}/src
Then, inside {ros_workspace}, launch:
    catkin_make

## How to launch simulation:

	roslaunch irb120 irb120.launch

## How to set desired joint positions:

	rostopic pub -1 /irb120/jointX_position_controller/command std_msgs/Float64 "data: Y"

substituting X with the joint number (1-6) and Y with the joint position (rad)