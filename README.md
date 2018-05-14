 
BSCVRoboticsProject 
===================


## Technical report on the Coperative task between Turtlebot2 and PhantomX Pincher-arm

![Project](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/project_pictures/Screenshot%20from%202018-05-10%2019:04:04.png)
=======================================================================================================================

Acknowledgments
---------------

Prof. Raph Seulin, Mr. Nathan CROMBEZ, Ms. Yifei ZHANG, Mr. Raphael DUVERNE, Marc BLANCHON

Table of contents
------------------

I. [PRESENTATION](https://github.com/Macaulay123/BSCVRoboticsProject#i-presentation)

II. [PROJECT OVERVIEW](https://github.com/Macaulay123/BSCVRoboticsProject#ii-project-overview)
   * [Workspace environment](https://github.com/Macaulay123/BSCVRoboticsProject#workspace-environment)
   * [Topics and Nodes](https://github.com/Macaulay123/BSCVRoboticsProject#topics-and-nodes)
   * [Packages](https://github.com/Macaulay123/BSCVRoboticsProject#packages)
   * [Launch Files](https://github.com/Macaulay123/BSCVRoboticsProject#launch-files)
   * [Scripts](https://github.com/Macaulay123/BSCVRoboticsProject#scripts)
   * [Bag File](https://github.com/Macaulay123/BSCVRoboticsProject#bag-file)

III. [PROJECT DESCRIPTION](https://github.com/Macaulay123/BSCVRoboticsProject#iii-project-description)

IV. [PROJECT SETUP](https://github.com/Macaulay123/BSCVRoboticsProject#iv-project-setup)
   * [Adding the 2D Laser sensor to the turtlebot2](https://github.com/Macaulay123/BSCVRoboticsProject#adding-the-2d-laser-sensor-to-the-turtlebot2)
   * [Map Creation](https://github.com/Macaulay123/BSCVRoboticsProject#map-creation)
   * [Installation of packages](https://github.com/Macaulay123/BSCVRoboticsProject#installation-of-packages)
   * [Navigation](https://github.com/Macaulay123/BSCVRoboticsProject#navigation)
   * [Obstacle avoidance](https://github.com/Macaulay123/BSCVRoboticsProject#obstacle-avoidance)
   * [PhantomX pincher-arm pick and place Operation](https://github.com/Macaulay123/BSCVRoboticsProject#phantomx-pincher-arm-pick-and-place-operation)
   
V. [PROJECT EXECUTION](https://github.com/Macaulay123/BSCVRoboticsProject#v-project-execution)
   * [Video](https://github.com/Macaulay123/BSCVRoboticsProject#video) 

 VI. [BUGS](https://github.com/Macaulay123/BSCVRoboticsProject#vi-bugs)
   * [Running the PhantomX pincher-arm](https://github.com/Macaulay123/BSCVRoboticsProject#running-the-phantomx-pincher-arm) 
   * [Planning of turtlebot trajectory](https://github.com/Macaulay123/BSCVRoboticsProject#planning-of-turtlebot-trajectory) 
   * [Problem with Obstacle Avoidance](https://github.com/Macaulay123/BSCVRoboticsProject#problem-with-obstacle-avoidance)
 
VII. [CONCLUSION](https://github.com/Macaulay123/BSCVRoboticsProject#vii-conclusion)

VIII. [REFERENCES](https://github.com/Macaulay123/BSCVRoboticsProject#viii-references)




I. PRESENTATION
------------

The present report is part of our semester project in the robotics engineering module. Working in ROS environment provides us a new approach to deal with large number of robots such as the Turtlebot 2 and the PhantomX Pincher arm. Those current autonomous mobile service robots are custom manufactured for research environments and limiting their availability. 
 The objective is to design and implement a cooperative task for a low- cost service robot based on TurtleBot 2 platform and the Pincher-arm.
 
II. PROJECT OVERVIEW
--------------------

#### Workspace environment

Before starting the project, we completed the ROS Installation as described in the technical survey. Then created our ROS Workspace:


`$ mkdir -p ~/catkin_ws/src`

`$ cd ~/catkin_ws/`

`$ catkin_make`



then we must source the setup after performing some settings on Workstation and TurtleBot2 Netbook. We have noticed that all workstations and netbooks in our robotics lab are already configured with ROS Indigo. Catkin here is the only build system used for new development, it is located at: 

`$ ~/ros/indigo/catkin_ws`

 we can edit the bash file by:
 
 `$ gedit ~/.bashrc`
 
  To control our turtlebot which is the netbook(Asus) via the workstation(Dell). 
  
`$ ssh turtlebot@192.168.0.100` 

#### Topics and Nodes

Nodes use a ROS client library like rospy or roscpp to communicate with other nodes. They can publish or subscribe to buses over which messages are exchanged called “Topics”. 

#### Packages

In our project work We created some packages ourselves, modify existing packages or install new packages to perform specific task with our robots and Pincher arm. However, some major packages that may be very essential for our project work are briefly described as follows:

* Creating packages: 
	* *turtlebot_nav*
	* *turtlebot_pose_commands*
	* *Joy2twist*
* Modified existing packages:
	* *turtlebot_navigation*
* New installed packages:
	* *rplidar-turtlebot2*
	* *arbotix_ros*
	* *turtlebot_arm*

#### Launch Files

Launch files were the only convenient way to start up multiple nodes and master, as well as other initialization requirements such as setting parameters for the initial pose, destination of our turtlebot within the map. 

#### Scripts

They are created to run some specific operations controlling the turtlebot using the joystick with joy2twist_listener.py inside the joy2twist package, also the pick_and_place.py inside the turtlebot_pose_commands package was used to control the robotic arm operation.

#### Bag File

This bag file was to record all topics which was used to create the map. Further information on how to create a map will be introduced in next section on Map creation.

III. PROJECT DESCRIPTION
------------------------

The project is about the implementation of the complete scenario of the turtlebot and the robotic PhantomX pincher-arm where the turtlebot is made to navigate to the position of the arm from its initial position, on arrival at the arm position, the arm will have to pick a cube that will be positioned on a specific location on the table and place it on the turtlebot then the turtlebot will take the cube to another location which in our case is the initial position of the turtlebot.

The project implementation is primarily based on the navigation of the turtlebot on a created map by giving it a goal position command on the map and the turtlebot could plan its path with reference to its initial position on the map and navigate to the goal position executing its trajectory using the laser scan sensor and as well avoid dynamically obstacles along its path.

![Scenario](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/project_pictures/turtlebot_arm.JPG)

IV. PROJECT SETUP
-----------------

__*On the turtlebot2:*__

#### Adding the 2D Laser sensor to the turtlebot2

The type of the laser scan sensor attached to our turtlebot2 is the RPLIDAR 2D laser sensor. 

![Laser sensor](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/project_pictures/RPLidar.JPG)

The rplidar-turtlebot2 is the Ros package designed to add the sensor to the turlebot2. This package can be clone and use from the repository; https://github.com/roboticslab-fr/rplidar-turtlebot2.git. 
Kindly refer to the README on the repository and follow the guides on its installation.


`$ cd ~/ros/indigo/catkin_ws/src`

`$ git clone https://github.com/roboticslab-fr/rplidar-turtlebot2.git` 

`$ cd .. && catkin_make`


#### Map Creation

We have created a map from the bag file we recorded when the turlebot was manually controlled with a Joypad through the space where all the task are to be carried out. The data required for the creation of the map is the scan topic recorded in the bag file which was published from the RPLIDAR 2D scan sensor we previously added to the turtlebot. By following the command lines given below we were able to successfully create the map called _`ClassMap.pgm`_ and its corresponding _`.yaml file`_  :_`ClassMap.yaml.`_


`$ rosmake gmapping`

`$ rosparam set use_sim_time true`

`$ rosrun gmapping slam_gmapping scan:=scan`

`$ rosbag play --clock ros/indigo/catkin_ws/Bags/ClassBag.bag`


After bags is finished playing then the map is obtained from the map server with the command;

`$ rosrun map_server map_saver -f ClassMap`

Picture of map obtained;

![map](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/project_pictures/map.png)

#### Installation of packages

In order to run the complete scenario for the project, a number of packages was put together including our own created packages with some modifications of script files, and parameters.

The following packages will need to be installed in the Catkin workspace for the execution of the scenario.

* __To launch the RPLIDAR sensor together with the turtlebot minimal launch and the *AMCL* launch in the *turtlebot_navigation* stack package.__

`$ cd ~/ros/indigo/catkin_ws/src`

`$ git clone https://github.com/Macaulay123/BSCVRoboticsProject/tree/master/turtlebot_nav` 

`$ cd .. && catkin_make`


__On the work station:__

* __The Arbotix ros driver to interface with the Arbotix board of the PhantomX pincher-arm.__


`$ cd ~/ros/indigo/catkin_ws/src`

`$ git clone https://github.com/Macaulay123/arbotix_ros.git`

`$ cd .. && catkin_make`


* __The *`turlebot_arm`* package to bring-up the PhantomX pincher-arm__


`$ cd ~/ros/indigo/catkin_ws/src`

`$ git clone https://github.com/Macaulay123/turtlebot_arm.git`

`$ cd .. && catkin_make`



* __The *`turtlebot_pose_commands`* package puts the different launch for complete scenario of the Project from the work station__

`$ cd ~/ros/indigo/catkin_ws/src`

`$ git clone https://github.com/Macaulay123/BSCVRoboticsProject/tree/master/turtlebot_pose_commands`

`$ cd .. && catkin_make`
  
haven completed all the set up on the the workstation and on the turlebot netbook as we have suggested, the next step that will be required will be to launch each process of the scenario which we have simplified by creating launch files that groups other launch files from different packages that is require and sets the different parameters to them in order to achieve individual particular processes.


#### Navigation

In this part of the project we want to be able to assign our map to the turtlebot to navigate on, and as well assume a default position of the turtlebot on the map in other to be able to give the turtlebot specific goal position command to reach. 
To achieve this we considered using the turtlebot_navigation stack package which uses the *AMCL* package. We have created our own package called *`turtlebot_nav`* it basically contains the launch files that is use to organize the processes involved for the turtlebot navigation processes. The content of this launch file is displayed below.

```
<launch>
		<!--Launch the LaserScan minimal-->
        <include file="$(find turtlebot_le2i)/launch/remap_rplidar_minimal.launch" />

		<!--Launch the AMCL with the initial position on the Map -->
        <include file="$(find turtlebot_navigation)/launch/amcl_demo.launch" >
          <arg name="map_file" value="/home/turtlebot/ros/indigo/catkin_ws/Bags/ClassMap.yaml" />
	  <arg name="initial_pose_x" default="9.541"/>
	  <arg name="initial_pose_y" default="-6.338"/>
	  <arg name="initial_pose_a" default="1.803"/>
        </include>
	
</launch>
```

*Explanation:*
The first part of the the code simply launches the remap_rplidar_minimal launch file from the *turtlebot_le2i* package. This launche file in itself does the following:
	*remap the 'cmd_vel_mux/input/navi' to 'cmd_vel'*
	*launches the turtlebot2 minimal launch,*
sets the envinronment variable for the rplaidar 2D scan sensor rather than the  default which is the 3D-sensor, and bring-up the rplidar 2D scan sensor.
This part put together basically enables us to be able to navigate our turtlebot using the scan message published by the 2D Rplidar scan sensor on the *cmd_vel* topic
 
The second part of our launch file code launches the amcl_demo.launch from the turtlebot_navigation package, and sets the our map as an argument to the mapserver for *amcl* and then set the assumed initial position of the turtlebot on the map (its linear position and the orientation).
With this process we can convinntly place our turtlebot on the marked initial position and give it a goal command on the map for it to navigate to by simply running the following command:

__On the turtlebot2:__

`$ roslaunch turtlebot_nav RP_nav_turt.launch`
 
__On the work station:__

`$ roslaunch turtlebot_pose_commands  tb_goal_pose.launch`

The __`tb_goal_pose.launch`__ is a lauunch file that simply contain a command to a target goal pose on the map which can take any value of the position and orientation of the turtlebot on the map depending on the user target goal position.
This command can as well be run directly on the workstation by publishing on the the topic _/move_base_simple/goal,_ however for convinience and the organisation of our work, we have chosen to rather have these launch files to use. 


#### Obstacle avoidance

In this case we want the turtlebot to be able to avoid dynamical obstacles along its path to its goal position. The turtlebot_navigation package performs this operation by default however, the package was created to work with the 3D sensor environment variable which seem to be responsible for the response to the velocity commands received on the _`cmd_vel`_  topic for the turtlebot base for obstacle avoidance. The Rplidar laser scan environment variable seems to publish its velocity commands to the _`cmd_vel_mux/input/naviand`_ as a result the turtlebot base receives two different velocity commands on different topics.  The topic to be executed is the one with the highest priority and then continues with the other with a lesser priority when the first stops.  Because of this, the turtlebot will continue to move in its regular path even when there is an obstacle. Also by default, the _amcl_ package sets the parameters for its obstacle avoidance to be responsive to the command velocity received on the _`cmd_vel`_. This problem was corrected eventually by first remapping the _`cmd_vel_mux/input/navi`_ to _`cmd_vel`_ as shown earlier in the __remap_rplidar_minimal.launch__, and it is also very important to change the parameter settings of the turtlebot_navigation by following the procedure highlighted below on the turtlebot which also requires administrative permission.

* move to the _turtlebot_navigation_ directory,

* back up the param folder with a different file name as you wish,

download the files (costmap_common_params.yaml, global_costmap_params.yaml, global_planner_params.yaml, local_costmap_params.yaml, and  move_base_params.yaml) from the  following links;

[base_local_planner_params.yaml](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/base_local_planner_params.yaml)

[costmap_common_params.yaml](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/costmap_common_params.yaml)

[global_costmap_params.yaml](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/global_costmap_params.yaml)

[global_planner_params.yaml](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/global_planner_params.yaml)

[local_costmap_params.yaml](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/local_costmap_params.yaml)

[move_base_params.yaml](https://github.com/Macaulay123/BSCVRoboticsProject/blob/master/move_base_params.yaml)

or simply copy the parameter settings in the each of the files with their corresponding files name in the _`turtlebot_navigation package`_.


Now by running the commands given below on the terminals

__On the turtlebot2:__

`roslaunch turtlebot_nav RP_nav_turt.launch`
 
__On the work station:__

`roslaunch turtlebot_pose_commands  tb_goal_pose.launch`

you will have the turtlebot navigate to the given goal pose on the map and will avoid any obstacle along its path.


#### PhantomX pincher arm pick and place Operation

The robotic arm operation was to pick a cube from the table on which it was mounted and place the cube on the turtlebot when the turtlebot arrive to the position of the table.
Since the idea was to manual control the robotic arm to pick the cube and place it on the turtlebot, a script named _`pick_and_place.py`_ in _turtlebot_pose_commands/bin_ directory was used together with the _`turtlebot_arm_moveit_config`_ package to control the arm to go to a marked out point on the table to pick the cube.
By running the command below, we were able to achieve the robotic arm pick and place operation.

__On the turtlebot2:__

`roslaunch turtlebot_nav RP_nav_turt.launch`

__On the work station:__

`roslaunch turtlebot_pose_commands  arm_pick_and_place.launch`


V. PROJECT EXECUTION
----------------------

The complete scenario of our project can be executed in a stream by following these steps bellow.

__On the turtlebot2:__

`roslaunch turtlebot_nav RP_nav_turt.launch`

__On the work station:__

_to view the turtlebot on the map with rviz GUI_

`roslaunch turtlebot_rviz_launchers view_navigation.launch --screen`


_first pose command (closer to the position of the robotic arm)_

`roslaunch turtlebot_pose_commands tb_goal_pose.launch`


_second pose command (to the exact position of the robotic arm)_

`roslaunch turtlebot_pose_commands  tb_goal_pose.launch`


_robotic arm pick and place operation_

`roslaunch turtlebot_pose_commands  arm_pick_and_place.launch`

after the completion of the arm pick and place operation, you one will have to kill all operation, and also the `turtlbot_nav RP_nav_turt.launch` operation on the turlebot, then you can continue with the process below

__On the turtlebot2:__

_to launch the turtlebot with its current position on the map_

`roslaunch turtlebot_nav RP_nav_turt2.launch`


__On the work station:__

_third pose command to prepare the turtlebot for its final destination on the map_

`$ roslaunch turtlebot_pose_commands  tb_return2.launch`


_fourth pose command (takes the turtlebot to its final destination on the map)_

`$ roslaunch turtlebot_pose_commands  tb_return_pose.launch.`

#### Video

[![SCENARIO](http://img.youtube.com/vi/KuOxycOS2-k/0.jpg)](http://www.youtube.com/watch?v=KuOxycOS2-k)
------------------------------------------------------------------------------------------------------


VI. Bugs
--------

#### Running the PhantomX pincher arm


There seems to be no package that was more efficient than the turtlebot_arm package which is supposed to run the robotic arm and the turtlebot2 together. A modified version of that package is used for the project, which was cloned from the repository https://github.com/NathanCrombez/turtlebot_arm we were able to run the robotic arm independent of the turtlebot successfully, however, with some warnings on the terminal. Those warnings prevent the turtlebot for further operation like giving it a new goal pose command. This was the reason we must kill all operation on the terminal and launch back the _RP_nav_turt2.launch_ before giving the turtlebot its consequent goal pose command as indicated above in the [project execution](https://github.com/Macaulay123/BSCVRoboticsProject#v-project-execution).

#### Planning of the Turtlebot Trajectory


It was quite difficult to give certain goal commands to the turtlebot depending on its initial orientation. The turtlebot would take more time to plan its trajectory to a target goal position if its initial orientation were about 180 degrees and so for a more effective planning of the turtlebot trajectory, it was required that we give the turtlebot goal position commands relative to its initial orientation. This was the reason we had to split the goal position command in the project scenario from two to four different positions for a faster and more effective execution of the project. 
This problem is because of the limitation in tuning the parameters of the  _`AMCL`_  package which was really complicated since we had to tune different parameters yet the function of one parameter always seem contradictory to another, making the operation of the robot erratic.

#### Problem with Obstacle Avoidance


this also had a lot to do with the parameter settings of the local and global costmap parameters of the _`AMCL`_ package. This involve the turtlebot perceiving the table at the position of the robot arm as an obstacle and so finding it difficult to move closer to the table. By adjusting the tolerance of the distance between the turtlebot and the obstacle will result in in turtlebot getting too close to the obstacle in other cases and a result colliding with the obstacle while trying to avoid it. We were able to resolve this to minimal by allowing the turtlebot reach the position from its side by taking its trajectory off the direction of the table and providing an extension to the turtlebot that makes it able to reach the robot arm at a reasonable distance from the table.



VII. CONCLUSION
----------------------

The project described the design and implementation of a turtlebot cooperative task with robotic arm. We were able to build the map, integrate the Rplidar laser sensor and test the avoidance obstacle on the turtlebot as well as the pick and place with the PhantomX pincher arm. In a nutshell, we can say that the overall scenario was executed successfully. Further interest will be on how to remove the bugs finding an adequate package that could run the pincher arm separately. As future work, we will be glad to implement, at latter stage, the ROS Smach to run the sequence of the operations.


VIII. REFERENCES
----------------

 * R. Patrick Goebel, ROS by Example: A do-it-yourself guide to the robot Operating System, pi      robot production,jan 2015, vol1.
 * R. Patrick Goebel, ROS by Example: A do-it-yourself guide to the robot Operating System, pi      robot production,jan 2015, vol2.
 
 * [ROS WIKI](http://wiki.ros.org/)
 * [Turtlebot2](https://www.turtlebot.com/turtlebot2/)
 * [PhantomX Pincher Arm](http://www.trossenrobotics.com/p/PhantomX-Pincher-Robot-Arm.aspx)
 * [RPlaider sensor](https://github.com/roboticslab-fr/rplidar-turtlebot2)
 * [Turtlebot_arm package](https://github.com/NathanCrombez/PhantomXPincherArmROS)

__Project repositories:__

[Macaulay Sadiq](https://github.com/Macaulay123/BSCVRoboticsProject)

[Deogratias Lukamba Nsadisa](https://github.com/nsadiasluk/Robotics-Engineering-Project)

[Antwi Kwaku Ebenezer](https://github.com/ebenezer11/BSCVRoboticsProject)




