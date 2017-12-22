# Istanbul Technical University BGL456E Robotics Term Project

## Running nodes:

### To run color_detect:

TURTLEBOT_GAZEBO_WORLD_FILE=/home/sddk/test_world
source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch turtlebot_gazebo turtlebot_world.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch project depth_proc.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun project run.py

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun rviz rviz

### To run localization:

cp -r gazebo/models/colorselectingplane ~/.gazebo/models

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch localization localize.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun localization explore.py

