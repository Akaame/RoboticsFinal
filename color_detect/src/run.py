#!/usr/bin/env python

## Common ROS headers.
import rospy
## Required for some printing options
# rosdep install depth_image_proc
import colorsys
import sys
import pcl_ros
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointCloud
from visualization_msgs.msg import Marker, MarkerArray
import tf
from do_trans import do_transform_cloud # sudo apt-get install python-tf2-sensor-msgs
from sens import read_points
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32, String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import actionlib

listener = tf.TransformListener() # create transform listener for robot coordinates
marker_publisher = None # marker publisher for detected colors
marker_array=MarkerArray() # marker array for markers to be published
marker_cntr=1 # counter of markers
start = False # start control variable
detector = None 

# to hold detected color coordinates
marker_dict = {
    'left':{
        'red': [],
        'green': [],
        'blue': [],
        'yellow': [[0.5,1.2]] # hardcoded
    },
    'right':{
        'red': [],
        'green': [],
        'blue': [],
        'yellow': [[-1.3, 5.]] # hardcoded
    }
}

def get_current_no_colors():
    """ 
    Get currently detected number of colors. 
    1 for each color on each side.
    Max:8 
    """
    if not start:
        return
    l = lambda x: {1 if len(x)>0 else 0}
    a = map(l,marker_dict['left'].itervalues())
    b = map(l,marker_dict['right'].itervalues())
    total = 0
    for item in a+b:
        total+=sum(item)
    return total

import numpy as np
def get_cluster_mean(arr):
    """ Get cluster mean of XYZ data """
    nparr = np.array(arr)
    return np.mean(arr, axis=0)
    
def create_stamped_transform(t):
    """ Create a stamped transform from a normal Transform """
    st_tf = TransformStamped()
    st_tf.transform.translation.x = t[0][0]
    st_tf.transform.translation.y = t[0][1]
    st_tf.transform.translation.z = t[0][2]

    st_tf.transform.rotation.x = t[1][0]
    st_tf.transform.rotation.y = t[1][1]
    st_tf.transform.rotation.z = t[1][2]
    st_tf.transform.rotation.w = t[1][3]
    return st_tf

def get_rgb_from_cloud_field(rgb):
    """
    Extract RGB data field from PointCloud2 data field.
    The last entry of PointCloud2 data encodes color information in a floating point number.
    """
    import struct
    import ctypes
    test = rgb
    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f' ,test)
    i = struct.unpack('>l',s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack >> 16) & 0x000000FF
    #print r
    g = (pack >> 8) & 0x000000FF
    #print g
    b = (pack >>0) & 0x000000FF # Error at this unpacking TODO
    #print b
    a = (pack) & 0x000000FF
    return r,g,b # prints r,g,b values in the 0-255 range 

def check_red_hsv(h,s,v):
    """ Red threshold for HSV color space """
    if -1< h < 20 and 40<s<100 and 40<v<100:
        return True
    return False

def check_red_rgb(r,g,b):
    """ Red threshold for RGB color space """
    if r>80 and g <25 and b <25:
        return True
    return False

def check_blue_hsv(h,s,v):
    """ Blue threshold for HSV color space """
    if 140< h < 190 and 40<s<100 and 40<v<100:
        return True
    return False

def check_blue_rgb(r,g,b):
    """ Blue threshold for RGB color space """
    if b>80 and r <25 and g <25:
        return True
    return False

def check_green_hsv(h,s,v):
    """ Green threshold for HSV color space """
    if 100< h < 140 and 40<s<100 and 40<v<100:
        return True
    return False

def check_green_rgb(r,g,b):
    """ Green threshold for RGB color space """
    if g>80 and r <25 and b <25:
        return True
    return False

def check_yellow_rgb(r,g,b):
    """ Yellow threshold for RGB color space """
    if g>80 and r >80 and b <25:
        return True
    return False

dumb_cache = None # left right changes

def camera_depth_registered_callback(data):
    if not start:
        ## workaround to eliminate effect of data 
        ## that comes after subscriber unregistration
        return
    global dumb_cache # make local
    global listener
    global marker_publisher
    global marker_array
    global marker_cntr
    global detect_status_pub
    try:
        (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
        do_transform_cloud(data,create_stamped_transform( (translation,orientation) ))
        # Get transformation between odometry base_footprint
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print("EXCEPTION:",e)
        return

    # listener.transformPointCloud("/odom",data)  # works on PointCloud data
    ## We will use a transform that works on PointCloud2
    from math import isnan
    
    red_cnt = 0
    green_cnt = 0
    blue_cnt = 0
    yellow_cnt = 0
    total_count = 0
    nan_count = 0
    for d in read_points(data): # use read_points method
        total_count += 1
        if not isnan(d[3]):
            rgb_data = get_rgb_from_cloud_field(d[3])
            # EXTRACT RGB
            r = float(rgb_data[0])
            g = float(rgb_data[1])
            b = float(rgb_data[2])
            
            # Check matched colors
            if check_red_rgb(r,g,b):
                red_cnt +=1
            if check_green_rgb(r,g,b):
                green_cnt+=1
            if check_blue_rgb(r,g,b):
                blue_cnt +=1
            if check_yellow_rgb(r,g,b):
                yellow_cnt +=1
        else:
            nan_count += 1 # if NaN increase nan count
    print "Red_count",red_cnt,"Total count", total_count, "NaN count",nan_count ,red_cnt/(total_count-float(nan_count)+1)
    print "Green_count",green_cnt,"Total count", total_count, "NaN count",nan_count ,green_cnt/(total_count-float(nan_count)+1)
    print "Blue_count",blue_cnt,"Total count", total_count, "NaN count",nan_count ,blue_cnt/(total_count-float(nan_count)+1)
    print "Yellow_count",yellow_cnt,"Total count", total_count, "NaN count",nan_count ,yellow_cnt/(total_count-float(nan_count)+1)
    # create a sphere marker and assign its properties
    marker=Marker()
    marker.header.frame_id="odom"
    marker.id=marker_cntr
    marker_cntr=marker_cntr+1
    marker.type=Marker.SPHERE
    marker.pose.position.x=translation[0]
    marker.pose.position.y=translation[1]
    marker.scale.x=0.2
    marker.scale.y=0.2
    marker.scale.z=0.2
    marker.color.a=1.0
    pos = [translation[0], translation[1]] # get current position
    print pos
    marker.action = Marker.ADD
    detected_color = None # detected color variable
    if red_cnt/(total_count-float(nan_count)+1) >0.05: # check against threshold
        print "Red Detected"
        detected_color = 'red'
        marker.color.r=1.0
        marker.color.g=0.0
        marker.color.b=0.0
        marker_array.markers.append(marker)
    if green_cnt/(total_count-float(nan_count)+1) >0.1:
        print "Green Detected"
        detected_color = 'green'
        marker.color.r=0.0
        marker.color.g=1.0
        marker.color.b=0.0
        marker_array.markers.append(marker)
    if blue_cnt/(total_count-float(nan_count)+1) >0.1:
        print "Blue Detected"
        detected_color = 'blue'
        marker.color.r=0.0
        marker.color.g=0.0
        marker.color.b=1.0
        marker_array.markers.append(marker)
    if yellow_cnt/(total_count-float(nan_count)+1) >0.06:
        print "Yellow Detected"
        detected_color = 'yellow'
        marker.color.r=1.0
        marker.color.g=1.0
        marker.color.b=0.0
        marker_array.markers.append(marker)
    # publish marker
    marker_publisher.publish(marker_array)
    if detected_color: # left right logic
        # fails if a single color is not detected on the left side :/
        direction = 'right' if get_current_no_colors()>5 else 'left' 
        no_colors = get_current_no_colors()
        if no_colors == 5:
            if dumb_cache:
                if dumb_cache != detected_color: # check if a new color is detected at the left-right turn
                    direction = 'right'
            else:
                dumb_cache = detected_color
        print "No of colors: ", no_colors
        if not type(marker_dict[direction][detected_color])==type(np.array([])):
            marker_dict[direction][detected_color].append(pos) # update marker_dict with detected color

def lap_callback(msg):
    """ Check in which lap the robot is currently in """
    print msg.data # Int32 message
    global start
    global action_client
    global detector
    if msg.data == 1 and not start:
        start = True
        # use depth_image_proc ros package to generate xyzrgb images
        # subscribe to depth_registered/points   
        detector = rospy.Subscriber("camera/depth_registered/points", PointCloud2, camera_depth_registered_callback, queue_size = 10000000)
        # http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
        # create detector for points
    elif msg.data == 2: 
        detector.unregister() # destroy detector 
        start = False
        dirs = ['left', 'right']
        colors = ['red', 'green','blue','yellow']
        # perform calculation of cluster means
        for d in dirs:
            for c in colors:
                marker_dict[d][c] = get_cluster_mean(marker_dict[d][c])
        from pprint import pprint
        pprint(marker_dict) # print cluster means
        for c in colors:
            for d in dirs:        
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "base_link"
                goal.target_pose.header.stamp = rospy.Time.now()
                # go towards cluster means 
                if not type(marker_dict[d][c])==type(np.array([])):
                    continue
                goal.target_pose.pose.position.x = marker_dict[d][c][0]
                goal.target_pose.pose.position.y = marker_dict[d][c][1]
                goal.target_pose.pose.orientation.w = 1.0
                action_client.send_goal(goal)
                success = action_client.wait_for_result(rospy.Duration(60))
                if not success:
                    action_client.cancel_goal()
                    rospy.loginfo("The base failed to move to area for some reason")
                else:
                    # We made it!
                    state = action_client.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Hooray, the base moved to area")

def color_detection_node():
    global listener
    global marker_publisher
    global detect_status_pub
    rospy.init_node('color_detect')
    #rospy.Subscriber("/camera/depth/image_raw", Image, camera_raw_callback, queue_size = 10000)
    #rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback, queue_size = 10000)
    global action_client
    action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    action_client.wait_for_server(rospy.Duration(5))
    
    marker_publisher =  rospy.Publisher("/project/markers", MarkerArray,queue_size=10000000)
    rospy.Subscriber("/explorer", Int32, lap_callback, queue_size=1000)    ## define two tf transforms as in hw2 answer from camera to base base to global
    ## use tf transform to transform pointcloud data
    ## publish rviz markers as in hw2 referee
    detect_status_pub = rospy.Publisher("/color_detect", String, queue_size=1000)
    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    color_detection_node()
