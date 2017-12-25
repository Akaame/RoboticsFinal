#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import pi, isnan
from visualization_msgs.msg import MarkerArray
import tf
from tf import transformations
from geometry_msgs.msg import Transform
from math import sqrt
from std_msgs.msg import Int32, String


wall_on_right = True
counter = 0
robot_base = tuple()
lap = 2
current_lap = 1
status = None
previous_dist = 1
jump = False

def map_callback(data):
    chatty_map = False
    if chatty_map:
        print "-------MAP---------"
        for x in range(0,data.info.width-1,5):
            for y in range(0,data.info.height-1,5):
                index = x+y*data.info.width
                if data.data[index] > 50:
                    sys.stdout.write('X')
                elif data.data[index] >= 0:
                    sys.stdout.write(' ')
                else:
                    sys.stdout.write('?')
            sys.stdout.write('\n')
        sys.stdout.flush()
        print "-------------------"

def laser_callback(data):
    motor_command = Twist()
    global motor_command_publisher
    global wall_on_right
    global listener
    global counter
    global robot_base
    global lap
    global finish_lap
    global current_lap
    global publish_lap
    global status
    global previous_dist
    global jump

    (translation, orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
    if counter == 0:
        robot_base = (translation[0], translation[1])
    elif counter != 0:
        robot_new = (translation[0], translation[1])
        distance = sqrt(pow(robot_base[0]-robot_new[0], 2) + pow(robot_base[1]-robot_new[1], 2))
        if distance < 0.2 and counter >= 15:
            publish_lap.publish(current_lap)
            if current_lap == lap:
                global laser
                laser.unregister()
                print "Stop"
            counter = 1 
            current_lap += 1
    distances = data.ranges
    length = len(distances)
    rightmost = distances[0]
    angle_diff = data.angle_increment
    


    

    print "Previous: " + str(previous_dist)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print '-------------------------------------------------------------------------------'

    if jump and rightmost > 1.3:
        print "JUMPPPPPPPPPPPPPPPPPPPPPPPP continue"
        motor_command.angular.z = -pi/10
        motor_command.linear.x = 0.3
    elif jump and rightmost > 1.0:
        motor_command.angular.z = 0
        motor_command.linear.x = 0.5
        jump = False
    elif jump and isnan(rightmost):
        motor_command.angular.z = -pi/3
        motor_command.linear.x = 0.3
    elif rightmost / previous_dist > 4.5:
        print "JUMPPPPPPPPPPPPPPPPPPPPPPPP"
        motor_command.angular.z = -pi/60
        motor_command.linear.x = 1
        jump = True
    elif rightmost > 1.3:
        motor_command.angular.z = -pi/3
        motor_command.linear.x = 0.2
    elif rightmost > 1.0:
        motor_command.angular.z = 0
        motor_command.linear.x = 0.5
        jump = False
    else:
        motor_command.angular.z = pi/2
        motor_command.linear.x = 0
    
    previous_dist = rightmost
    counter += 1
    motor_command_publisher.publish(motor_command)


def detect_callback(msg):
    global status
    status = msg.data


def explorer_node():
    rospy.init_node("explorer")

    global motor_command_publisher
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
    global listener
    listener = tf.TransformListener()   
    global laser 
    laser = rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1000)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1000)
    global publish_lap 
    publish_lap = rospy.Publisher("/explorer", Int32, queue_size=1000)
    global detect_sub
    detect_sub = rospy.Subscriber("/color_detect", String, detect_callback, queue_size=1000)
    rospy.spin()

if __name__=="__main__":
    explorer_node()
