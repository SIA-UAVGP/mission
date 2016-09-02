# How to build
  $ mkdir mission_ws/src
  
  $ cd mission_ws
  
  $ catkin init
  
  $ cd src
  
  $ git clone https://github.com/huanglilong/mission

  $ cd ..
  
  $ catkin build
  
# How to connect to pixhawk
  Read px4's [doc](http://dev.px4.io/pixhawk-companion-computer.html)
  
  $ roslaunch mavros px4.launch
  
  $ rosrun mission mission_node

  Switch to OFFBOARD mode

  Press safety switch and arm

# Simulating with gazebo
  $ make posix_sitl_default gazebo

  $ roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

  $ rosrun mission mission_node

  $ rosrun mavros mavsys mode -c OFFBOARD

  $ rosrun mavros mavsafety arm

# Set paramter
  $ rosrun rqt_reconfigure rqt_reconfigure
 
# Get the topic's messages
  $ rqt_graph
  
  $ rosrun rqt_topic rqt_topic
