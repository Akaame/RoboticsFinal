#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import pi, isnan


wall_on_right = True

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

    distances = data.ranges
    length = len(distances)
    leftmost = distances[-1]
    rightmost = distances[0]
    middle = distances[length/2]

    for val in distances[:(length/2)]:
        if not isnan(val):
            check = True
            break
        else:
            check = False
    print "wall on right:" + str(wall_on_right) + " check: " + str(check)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print 'The distance to the middle scanned point is: ', data.ranges[len(data.ranges)/2]
    print '-------------------------------------------------------------------------------'

    if rightmost > 1.9:
        motor_command.angular.z = -pi/3
        motor_command.linear.x = 0.2
    elif rightmost > 1.2:
        motor_command.angular.z = 0
        motor_command.linear.x = 0.5
    elif wall_on_right == False and check == False:
        motor_command.angular.z = -pi/3
        motor_command.linear.x = 0.3
    elif wall_on_right == False and check == True:
        motor_command.angular.z = -pi/2
        motor_command.linear.x = 0.3
    elif wall_on_right == True and check == False:
        motor_command.angular.z = pi/3
        motor_command.linear.x = 0.3
    else:
        motor_command.angular.z = pi/2
        motor_command.linear.x = 0
    
    wall_on_right = check
    motor_command_publisher.publish(motor_command)


def explorer_node():
    rospy.init_node("explorer")

    global motor_command_publisher
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1000)
    rospy.Subscriber("/map", OccupancyGrid, map_callback, queue_size=1000)
    rospy.spin()

if __name__=="__main__":
    explorer_node()
