#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import pi
import os
import math 

class Bug2():
    def __init__(self):
        rospy.init_node('bug2', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback) 
	# How fast will we update the robot's movement?
        self.rate = 200
        
        # Set the equivalent ROS rate variable
        self.r = rospy.Rate(self.rate)
	self.r.sleep()        
        # Set the forward linear speed to 0.2 meters per second 
        self.linear_speed = 0.3
	
	# Set the rotation speed to 1.0 radians per second
	self.angular_speed = 1.0

	self.objects_center = 10000
	self.objects_left = 10000
	self.objects_right = 10000
	self.front_nearness_ceiling = 0.7
	self.front_nearness_floor= 0.5
	self.right_nearness_ceiling = 0.7
	self.right_nearness_floor = 0.5
	self.translation_amount = 0.1
	self.rotation_amount = 5
	self.run_pathfinder()

    def run_pathfinder(self):
	print("Starting Pathfinder")
	# TODO: orient the object towards the goal (aka find the m-line)
	while not rospy.is_shutdown():
	    if not self.obstacle_in_front():
	        self.translate(self.translation_amount)
	        print("Object Distances: {} {} {}".format(self.objects_left, self.objects_center, self.objects_right))
   	    else:
		# TODO: record the hit point
		print("Hit obstacle")
		while self.obstacle_in_front():
		    print("Rotate left")
		    self.rotate(self.rotation_amount)
		# TODO: change this while loop to terminate when reaching hitpoint or destination
		while True:
		    if not self.obstacle_on_right():
			print("Rotate right")
			self.rotate(-self.rotation_amount)
		    else:
			print("Rotate let and move")
			self.rotate(self.rotation_amount)	
			self.translate(self.translation_amount)
	     
    def obstacle_in_front(self):
	result = math.isnan(self.objects_center) == False and self.objects_center < self.front_nearness_ceiling and self.objects_center > self.front_nearness_floor
	return result

    def obstacle_on_right(self):
	result = math.isnan(self.objects_right) == False and self.objects_right < self.right_nearness_ceiling and self.objects_right > self.right_nearness_floor
	return result
 
    def rotate(self, angle_in_degrees):
	goal_angle = angle_in_degrees * float(pi/180)
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
    
    def scan_callback(self, msg):
	self.objects_center = msg.ranges[len(msg.ranges)/2]
	self.objects_left = msg.ranges[len(msg.ranges)-1]
	self.objects_right = msg.ranges[0]
	
    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Bug2()
    except Exception as e:
	print(e)
        rospy.loginfo("Bug2 node terminated.")

