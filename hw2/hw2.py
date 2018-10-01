#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import LaserScan
from math import pi

class Bug2():
import rospy
from geometry_msgs.msg import Twist
from math import pi
import os
import math 

class OutAndBack():
    def __init__(self):
        # Give the node a name
        rospy.init_node('out_and_back', anonymous=False)

        # Set rospy to execute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # How fast will we update the robot's movement?
        self.rate = 50
        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)
        
        # Set the forward linear speed to 0.2 meters per second 
        self.linear_speed = 0.2
	
	# Set the rotation speed to 1.0 radians per second
	self.angular_speed = 1.0

	self.run_pathfinder()

    def run_pathfinder(self):
	pass

    def rotate(self, angle_in_radians):
	goal_angle = angle_in_radians * float(pi/180)
	angular_duration = goal_angle / self.angular_speed
	angular_duration = math.fabs(angular_duration)

	move_cmd = Twist()
	if goal_angle < 0:
		move_cmd.angular.z = self.angular_speed * -1.0
	else:
		move_cmd.angular.z = self.angular_speed 
	ticks = int(angular_duration * self.rate)
	
	for t in range(ticks):
		self.cmd_vel.publish(move_cmd)
		self.r.sleep()
	self.cmd_vel.publish(Twist())
	
    def translate(self, goal_distance):
        linear_duration = goal_distance / self.linear_speed
        linear_duration = math.fabs(linear_duration)

        move_cmd = Twist()
	if goal_distance < 0:
		move_cmd.linear.x = self.linear_speed * -1.0
	else:
		move_cmd.linear.x = self.linear_speed
        
	ticks = int(linear_duration * self.rate)

        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            self.r.sleep()
        self.cmd_vel.publish(Twist())
            
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def run_pathfinder(self):
	"""
	Pseudocode
	while self.not_reached(goal_point):
	    if self.obstacle_detected():
		store_hit_point()
		while 
	    else:
		self.move_on_m_line()
		
	"""
	pass


    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Bug2()
    except:
        rospy.loginfo("Bug2 node terminated.")

