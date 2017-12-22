# Istanbul Technical University BGL456E Robotics Term Project

## Running nodes:

### To run color_detect:

```bash

sudo apt-get install python-tf2-sensor-msgs
sudo cp -r color_detect/sens.py /usr/lib/python2.7/dist-packages/sensor_msgs/point_cloud2.py
sudo cp -r color_detect/do_trans.py /usr/lib/python2.7/dist-packages/tf2_sensor_msgs/tf2_sensor_msgs.py

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch color_detect color_detect.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun color_detect run.py
```
### To run localization:

```bash
cp -r gazebo/models/colorselectingplane ~/.gazebo/models

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
roslaunch localization localize.launch

source /opt/ros/kinetic/setup.bash
source devel/setup.bash
rosrun localization explore.py
```
