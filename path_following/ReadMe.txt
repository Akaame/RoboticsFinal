# Istanbul Technical University BGL456E Robotics Term Project

## WP1: Path Planning and Following Part

## Running nodes:

### To run the explored map:

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/test_map.yaml

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch turtlebot_rviz_launchers view_navigation.launch

### To run path following part:

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch

source /opt/ros/kinetic/setup.bash
source /home/sddk/catkin_ws/devel/setup.bash
source /home/sddk/temp/devel/setup.bash
rosrun trajectory_referee_456 referee.py route6 dis

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch trajectory_skeleton_456 view_robot.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun trajectory_skeleton_456 high_speed_controller


*** Route6 path is created for following on this part. This path is following on 2. Homework environment with 2. Homework code pattern.***