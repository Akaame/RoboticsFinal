#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi, isnan, sqrt
from tf import transformations
from geometry_msgs.msg import Transform
from std_msgs.msg import Int32


counter = 0 #move counter for keeping track of lap end
robot_base = tuple() #odom based coordinate of the robot
lap = 2 #first one is exploring second one is color detect
current_lap = 1 #starting lap
previous_dist = 1 #keeping track of jumps for explorations if rightmost/current_dist too large then it sees the other side of the map
jump = False #jump detector


def laser_callback(data):
    motor_command = Twist()
    global motor_command_publisher
    global listener
    global counter
    global robot_base
    global lap
    global current_lap
    global publish_lap
    global previous_dist
    global jump

    #first find where the robot is
    (translation, orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
    if counter == 0:
        #get start point of the map
        robot_base = (translation[0], translation[1])
    elif counter != 0:
        #robot pose changing with moves
        robot_new = (translation[0], translation[1])
        #getting distance with lap start point
        distance = sqrt(pow(robot_base[0]-robot_new[0], 2) + pow(robot_base[1]-robot_new[1], 2))
        if distance < 0.2 and counter >= 15:
            # if distance is close and counter is at least 15 (assuming minimum number of moves) then lap is finished
            # publishing lap information with color detect node
            publish_lap.publish(current_lap)
            if current_lap == lap:
                # if current lap is the final lap
                global laser
                laser.unregister()
                # stop laserscan
                print "Stop"
            # set counter to 1 to indicate new lap
            counter = 1 
            current_lap += 1
    distances = data.ranges
    rightmost = distances[0]

    print "Previous: " + str(previous_dist)
    print 'The distance to the rightmost scanned point is: ', data.ranges[0]
    print '-------------------------------------------------------------------------------'
    '''
    Explanation of jump:
    I kept track of the current_move rightmost and the previous_move rightmost values
    If rightmost / previous_distance is too high, it means the rightmost sees the other side of the map, 
    so this is a jump point (corner)
    '''
    if jump and rightmost > 1.3:
        '''
        The jump continues until seeing rightmost smaller than 1.3
        '''
        print "JUMPPPPPPPPPPPPPPPPPPPPPPPP continue"
        motor_command.angular.z = -pi/10
        motor_command.linear.x = 0.3
    elif jump and rightmost > 1.0:
        '''
        The jump continues until seeing rightmost bigger than 1.0
        '''
        motor_command.angular.z = 0
        motor_command.linear.x = 0.5
        jump = False
    elif jump and isnan(rightmost):
        '''
        The jump continues until seeing rightmost seeing NaN is not a case
        '''
        motor_command.angular.z = -pi/3
        motor_command.linear.x = 0.3
    elif rightmost / previous_dist > 4.5:
        '''
        The jump point. Factor > 4.5. Starting the rotation
        '''
        print "JUMPPPPPPPPPPPPPPPPPPPPPPPP"
        motor_command.angular.z = -pi/60
        motor_command.linear.x = 1
        jump = True
    elif rightmost > 1.3:
        '''
        Normal wall following. Keeping the robot under limits 1.0 and 1.3
        '''
        motor_command.angular.z = -pi/3
        motor_command.linear.x = 0.2
    elif rightmost > 1.0:
        '''
        Normal wall following. Keeping the robot under limits 1.0 and 1.3
        '''
        motor_command.angular.z = 0
        motor_command.linear.x = 0.5
        jump = False
    else:
        '''
        Normal wall following for NaNs
        '''
        motor_command.angular.z = pi/2
        motor_command.linear.x = 0
    #set previous distance
    previous_dist = rightmost
    #increment the move
    counter += 1
    motor_command_publisher.publish(motor_command)


def explorer_node():
    rospy.init_node("explorer")
    #to control robot
    global motor_command_publisher
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
    # to get where the robot is tf is used
    global listener
    listener = tf.TransformListener()   
    # fake laser scans for autonomous exploration
    global laser 
    laser = rospy.Subscriber("/scan", LaserScan, laser_callback, queue_size=1000)
    # to send lap info to subscribers which is color detect
    global publish_lap 
    publish_lap = rospy.Publisher("/explorer", Int32, queue_size=1000)
    rospy.spin()

if __name__=="__main__":
    explorer_node()
