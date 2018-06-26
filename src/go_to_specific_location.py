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

def image_callback(msg):
	print("Reeived image")
	try:
		#convert image 
		cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError, e:
		print(e)
	else:
		cv2.imwrite('camera_image{}.jpeg'.format(rospy.Time.now().to_sec()), cv2_img)	

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        # Define the Twist compenent
        rotation_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        vel_msg = Twist()
	angular_speed = 1
        vel_msg.angular.z = 1
	angle = 360*2*PI/360
        navigator = GoToPose()
	# Subcribe to the images published by rgb and call the image_callback metho
	rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
        # Customize the following values so they are appropriate for your location
        position1 = {'x': 1.06, 'y' : -0.134}
        position2 = {'x': 3.42, 'y': -3.81}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
        success = navigator.goto(position1, quaternion)
        counter = 0
        while success: 
            if success and counter == 0:
                # Once succesful turn 360 degrees
		current_angle = 0
		t0 = rospy.Time.now().to_sec()
		while  current_angle < angle:
                	rotation_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1 - t0)
            	rospy.loginfo("Rotated")
                # Go to next desired location
                success = navigator.goto(position2, quaternion)
                counter = 1
                rospy.loginfo("Returning to position 2")
            elif success and counter == 1:
		current_angle = 0
		t0 = rospy.Time.now().to_sec()
		while current_angle < angle:
                	rotation_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed*(t1 - t0)
		rospy.loginfo("Rotated")
                success = navigator.goto(position1, quaternion)
                counter = 0
                rospy.loginfo("Going to position1")
            else:
                rospy.loginfo("The base failed to reach the desired pose")
            # Sleep to give the last log messages time to be sent
            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
