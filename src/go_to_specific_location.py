#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import cv2 
PI = 3.1415926535897
bridge = CvBridge()
class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	rospy.on_shutdown(self.shutdown)
	
	# Tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

    def goto(self, pos, quat):

        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            rospy.loginfo('Arrived safely')
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)	

def rotate(rotation_publisher):
    angle = 2*PI
    vel_msg = Twist()
    angular_speed = 0.1
    vel_msg.angular.z = angular_speed
    current_angle = 0
    t0 = rospy.Time.now().to_sec()
    while current_angle < angle:
        rotation_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1 - t0)
    rospy.loginfo("Rotated")

if __name__ == '__main__':
    try:    
        rospy.init_node('nav_test', anonymous=False)
        # Define the Twist compenent
        rotation_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        navigator = GoToPose()
        # Customize the following values so they are appropriate for your location
        position1 = {'x': 3.34, 'y': -3.86}
        #position1 = {'x': 0.0, 'y': 0.0}
        position2 = {'x': 0.64, 'y' : 0.13}
        position3 = {'x': 5.24, 'y': -0.08}
        position4 = {'x': 3.71, 'y': 2.3}
	position5 = {'x': 7.48, 'y': 2.02}
        position6 = {'x': 5.66, 'y': 4.39}
        position7 = {'x': 12.1, 'y': 3.88}
        position8 = {'x': 8.29, 'y': 6.63}
        positions = []
        positions.append(position1)
        positions.append(position2)
        positions.append(position3)
        positions.append(position4)
        positions.append(position5)
        positions.append(position6)
        positions.append(position7)
        positions.append(position8)
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        rospy.loginfo("Going to Position 1")
        success = navigator.goto(positions[0], quaternion)
        rotate(rotation_publisher)
        count = 1
        while count < len(positions):
            rospy.loginfo("Going to next position")
            success = navigator.goto(positions[count], quaternion)
            if success:
                rospy.loginfo("Got to location succesfully")
                rotate(rotation_publisher)
            else:
                rospy.loginfo("Did not get to location successfully")
            count += 1
            rospy.sleep(1)
        rospy.loginfo("completed")

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
