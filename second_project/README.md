# Second Robotics Project 
A.Y. 2020/2021

Team member:
Magliani Martina
Moro Alessandra

## Project Description

### Description of the files in the archive

   1) In the launch directory we have:
   
      a. gmapping.launch: for the gmapping we have created gmapping.launch file that set the gmapping configuration. Coupled with this one there is the gmapping.launch.xml that sets all the parameters for the gmapping.
      
      b. amcl.launch: for the configuration of the amcl.
	 In this lauch file we set the angle for the creation of the map with a static_transform_publisher 0 0 0 0 0 0.707 0.707 base_link laser 100 that correspond 	      to 90 degrees in the quaternion unity.
	 Afterwards we launch the IMU.xml that is used to fuse with our data che IMU data, and set the sensors topic that we have choosen. We have taken the most 	   reliable sensors that are /camera/odom/sample and /imu/data_raw.  
    	 Afterwards we have created the map with the map_server node. 
         At the end we launch amcl.launch.xml file that sets all the params of amcl.launch.

  2) In the map directory we have:
  
	a. map.pgm that is the map created by visual odometry, used for move_base
	b. map.yaml
	c. maze.png
	d. maze.world

  3) In the param directory we have:
  
	a. Params.yaml file that sets the frequency of the main run loop, delay, magnetic_declination_radians and the yaw_offset.

  4) In the rviz directory we have:

  	a. Robot_navigation.rviz that sets the rviz parameters when this one is open for the map visualization and the robot trajectory in the map.
	
  5) For the structure of the tf tree we have the frames.pdf that shows the tf_graph.	

### Startup commands
We test our project on ubuntu 18.04 with ROS Melodic, Running on the terminal these commands:

1) In the first terminal roscore
2) run the bag in another terminal with clock  rosbag play --clock filename.bag	
3) In another one roslaunch rob_pkg gmapping.launch
4) In another terminal roslaunch rob_pkg amcl.launch

After this commands it will be automatically open the visualization on the map in rviz.
Rviz will show how the robot moves in the given map. 
The robot localization is represented by the green multiple arrows.
 
The project consists in comprehends which is the correct visualization of the map, then how use the given data, like sensors and the odometry of the robot (bag files), for the localization of the robot in this map.
The bigger difficulty is in setting right parameters that allows the robot to avoid the obstacles.