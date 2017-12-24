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
from std_msgs.msg import Int32

listener = tf.TransformListener()
marker_publisher = None
marker_array=MarkerArray()
marker_cntr=0
start = False


marker_dict = {
    'left':{
        'red': [],
        'green': [],
        'blue': [],
        'yellow': []
    },
    'right':{
        'red': [],
        'green': [],
        'blue': [],
        'yellow': []
    }
}

def get_current_no_colors():
    """ 
    Get currently detected number of colors. 
    1 for each color on each side.
    Max:8 
    """
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
    if -1< h < 20 and 40<s<100 and 40<v<100:
        return True
    return False

def check_red_rgb(r,g,b):
    if r>80 and g <25 and b <25:
        return True
    return False

def check_blue_hsv(h,s,v):
    if 140< h < 190 and 40<s<100 and 40<v<100:
        return True
    return False

def check_blue_rgb(r,g,b):
    if b>80 and r <25 and g <25:
        return True
    return False

def check_green_hsv(h,s,v):
    if 100< h < 140 and 40<s<100 and 40<v<100:
        return True
    return False

def check_green_rgb(r,g,b):
    if g>80 and r <25 and b <25:
        return True
    return False

def check_yellow_rgb(r,g,b):
    if g>80 and r >80 and b <25:
        return True
    return False

def camera_depth_registered_callback(data):
    global listener
    global marker_publisher
    global marker_array
    global marker_cntr
    try:
        (translation,orientation) = listener.lookupTransform("/camera_depth_frame", "/odom", rospy.Time(0))
        do_transform_cloud(data,create_stamped_transform( (translation,orientation) ))
        # print "Odometry base translation", translation,orientation
    except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        print("EXCEPTION:",e)
        #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
        return

    #listener.transformPointCloud("/odom",data)  # works on pointcloud data DONE
    ## find a transform that works on pointcloud2
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
            # print get_rgb_from_cloud_field(d[3])
            # insert another conditional here only if desired color is matched 
            # for now seeing red is enough 
            # print get_rgb_from_cloud_field(d[3])
            rgb_data = get_rgb_from_cloud_field(d[3])
            r = float(rgb_data[0])
            g = float(rgb_data[1])
            b = float(rgb_data[2])
            
            #if check_red_hsv(*colorsys.rgb_to_hsv(r,g,b)):
            #    #print "RED"
            #    red_cnt += 1
            print r,g,b
            if check_red_rgb(r,g,b):
                red_cnt +=1
            if check_green_rgb(r,g,b):
                green_cnt+=1
            if check_blue_rgb(r,g,b):
                blue_cnt +=1
            if check_yellow_rgb(r,g,b):
                yellow_cnt +=1
        else:
            nan_count += 1
    print "Red_count",red_cnt,"Total count", total_count, "NaN count",nan_count ,red_cnt/(total_count-float(nan_count)+1)
    print "Green_count",green_cnt,"Total count", total_count, "NaN count",nan_count ,green_cnt/(total_count-float(nan_count)+1)
    print "Blue_count",blue_cnt,"Total count", total_count, "NaN count",nan_count ,blue_cnt/(total_count-float(nan_count)+1)
    print "Yellow_count",yellow_cnt,"Total count", total_count, "NaN count",nan_count ,yellow_cnt/(total_count-float(nan_count)+1)
    marker=Marker()
    marker.header.frame_id="odom"
    marker.id=marker_cntr
    marker_cntr=marker_cntr+1
    marker.type=Marker.SPHERE
    marker.pose.position.x=d[0]
    marker.pose.position.y=d[1]
    marker.pose.position.z=d[2]
    marker.scale.x=0.2
    marker.scale.y=0.2
    marker.scale.z=0.2
    marker.color.a=1.0
    pos = [d[0],d[1],d[2]]
    direction = 'right' if get_current_no_colors()>4 else 'left' 
    marker.action = Marker.ADD
    if red_cnt/(total_count-float(nan_count)+1) >0.1:
        print "Red Detected"
        marker_dict[direction]['red'].append(pos)
        marker.color.r=1.0
        marker.color.g=0.0
        marker.color.b=0.0
    if green_cnt/(total_count-float(nan_count)+1) >0.1:
        print "Green Detected"
        marker_dict[direction]['green'].append(pos)
        marker.color.r=0.0
        marker.color.g=1.0
        marker.color.b=0.0
    if blue_cnt/(total_count-float(nan_count)+1) >0.1:
        print "Blue Detected"
        marker_dict[direction]['blue'].append(pos)
        marker.color.r=0.0
        marker.color.g=0.0
        marker.color.b=1.0
    if yellow_cnt/(total_count-float(nan_count)+1) >0.1:
        print "Yellow Detected"
        marker_dict[direction]['yellow'].append(pos)
        marker.color.r=1.0
        marker.color.g=1.0
        marker.color.b=0.0
    marker_array.markers.append(marker)
    marker_publisher.publish(marker_array)

def lap_callback(msg):
    global start
    if msg.data == 1 and not start:
        start = True
        # use depth_image_proc ros package to generate xyzrgb images
        # subscribe to depth_registered/points
        global detector
        detector = rospy.Subscriber("camera/depth_registered/points", PointCloud2, camera_depth_registered_callback, queue_size = 10000000)
        # http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
    elif msg.data == 2:
        detector.unregister()

def color_detection_node():
    global listener
    global marker_publisher
    rospy.init_node('color_detect')
    #rospy.Subscriber("/camera/depth/image_raw", Image, camera_raw_callback, queue_size = 10000)
    #rospy.Subscriber("/camera/depth/points", PointCloud2, camera_depth_callback, queue_size = 10000)
    
    
    marker_publisher =  rospy.Publisher("/project/markers", MarkerArray,queue_size=10000000)
    rospy.Subscriber("/explorer", Int32, lap_callback, queue_size=1000)    ## define two tf transforms as in hw2 answer from camera to base base to global
    ## use tf transform to transform pointcloud data
    ## publish rviz markers as in hw2 referee

    ## spin is an infinite loop but it lets callbacks to be called when a new data available. That means spin keeps this node not terminated and run the callback when nessessary. 
    rospy.spin()
    
if __name__ == '__main__':
    color_detection_node()
