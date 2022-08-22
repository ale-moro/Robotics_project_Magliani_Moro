# First Robotics Project 
A.Y. 2020/2021

Team components:
Magliani Martina
Moro Alessandra

## Project description

### The archive contains the ros package directory “robotics_pkg” structured as follows:

-CMakeList.txt: given automatically by creating the “robotics_pkg” package. 
                Every message, dependence, service and package file added have been updated in the CMakeList file.
		
-package.xml: automatically generated by ros with the purpose of setting all the packages, dependencies and output.

-frames.pdf: tree structure buffered in time that show the relationship between coordinate frames.

-[dir] src: contains the .cpp source files:

	• main_node_od.cpp is the handler of the odometry calculation;
	
	• ClientResetOdomToInit.cpp is the client node of the service “ResetOdometryToInit”;
	
	• ClientResetOdomToPose.cpp is the client node of the service “ResetOdometryToPose”.
	
-[dir] srv: contains the services files:

	• ResetOdometryToInit.srv that resets all the variables of the odometry to zero;
	
	• ResetOdometryToPose.srv that resets the x, y, theta variables with the requested ones.
	
-[dir] cfg: contains the “parameters.cfg” python code file. It describes the parameters available in the “main_node_od”;

-[dir] msg: contains the msg files which are customOdometry.msg and MotorSpeed.msg.

	• CustomOdometry.msg is the message type published on "customOdometry" topic.
	  It contains Odometry information on computed odometry and a string specifying the approximation mode.
	  
	• MotorSpeed.msg is the message type used to read messages generated by the given .bag file.

### Constants:

- The constant EULER_APPROXIMATION sets the approximation mode to the Euler Integration and is a boolean value true;
- The constant RUNGE_KUTTA_APPROXIMATION sets the approximation mode to the Runge kutta Integration and is a boolean value false;
- The GEAR RATIO is a constant between 35.0 and 40.0. The exact value 38.0 is achieved by dividing the output speed by the input speed. The input speed is given by the .bag file and the output speed is given by our calculation.
- The APPARENT_BASELINE is a constant that is approximatly two times value of the given real baseline.

### Odometry Structure:
- Odometry_values is a structure that contains all the odometry values.
TF Tree Structure: see the frames.pdf file.

### Custom Messages: 
- customOdometry.msg contains:
  std_msgs/String approxMode
  nav_msgs/Odometry odometry
- MotorSpeed.msg contains:
  Header header
  float64 rpm

## Startup command:
1. roscore
2. rosbag play namebagfile.bag
3. rosrun robotics_pkg main_node_eseg