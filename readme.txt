Changelog:
	
	V0.0 : initial version. Joints individually controlled by PID controllers from libraries.
	V0.1 : joint_1 now controlled with custom P controller my_pid_controller1. Added my_pid_controller1 package.
	V0.2 : modified my_pid_controller1 to control joint_1 and joint_2 simultaneously (2 independent P controllers in one controller plugin),
	       just to test how to control multiple joints with one controller plugin.
	V0.3 : extended my_pid_controller1 to PID. Every joint is now controlled by an instance of my_pid_controller1.

Notes:

	The controller is exported as a plugin library. If I change the code of the controller (my_pid_controller1.cpp), for
	some reason catkin_make is not sufficient to update the controller. I solved by eliminating build and devel folders
	inside {ros_workspace} and then catkin_make.

Description:

	Package with only the model of the robot (taken from abb_irb120_support package and simplified removing unnecessary stuff)
	The joints are singularly controlled.
	All joints are controlled with custom PID controller my_pid_controller1.

How to "install":

	Copy folder CPR_project into {ros_workspace}/src
	launch catkin_make inside {ros_workspace}

How to launch simulation:

	roslaunch irb120 irb120.launch

How to set desired joint positions:

	rostopic pub -1 /irb120/jointX_position_controller/command std_msgs/Float64 "data: Y"
	substituting X with the joint number (1-6) and Y with the joint position (rad)
