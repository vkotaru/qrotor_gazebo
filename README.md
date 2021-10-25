# qrotor_gazebo
A simple gazebo plugin for quadrotor simulation



### Usage

- Compile
  ```
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/vkotaru/qrotor_gazebo.git
  $ cd ~/catkin_ws
  $ catkin_make
  ```
- Launch Simulation
  ```
  $ cd ~/catkin_ws
  $ source devel/setup.bash
  $ roslaunch qrotor_gazebo spwan_falcon.launch 
  ```
- Offboard Control
  ```
  $ rosrun qrotor_offboard publish_command.py
  ```
