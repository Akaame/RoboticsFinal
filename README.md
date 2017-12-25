# Istanbul Technical University BGL456E Robotics Term Project

## Running nodes:

### To set environment and install necessary packages:

```bash

sudo apt-get install python-tf2-sensor-msgs
sudo cp -r color_detect/sens.py /usr/lib/python2.7/dist-packages/sensor_msgs/point_cloud2.py
sudo cp -r color_detect/do_trans.py /usr/lib/python2.7/dist-packages/tf2_sensor_msgs/tf2_sensor_msgs.py
sudo cp -r gazebo/models/colorselectingplane ~/.gazebo/models
```
### To run the project GazeRo:

```bash

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch color_selector color_selector.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun localization explore.py

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun color_detect run.py
```
