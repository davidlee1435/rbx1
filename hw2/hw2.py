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

	self.closeness_threshold = 0.005

	self.translation_amount = 0.1
	self.rotation_amount = 5
	# Run main method
	self.run_pathfinder()

    def run_pathfinder(self):
	"""
	High level overview of run_pathfinder:

	1. Move forward until bug hits obstacle
	2. When that happens, turn left until the right-most sensor barely registers. Rotate 30 degrees more to the left to account for the fact that the right most sensor is not 90 degrees from the center of the bug's viewpoint
	3. The bug then moves right until the right sensor starts to register the object. The right sensor must be within some bound of the min value of all the values from LaserScan
	4. After that happens, or until the bug has completed a full revolution, break out of the while loop, turn a bit left, and translate forward. The bug usually goes into a loop on corners
	"""
	print("Starting Pathfinder")
	# TODO: orient the object towards the goal (aka find the m-line)
	while not rospy.is_shutdown():
	    if not self.obstacle_in_front():
	        self.translate(self.translation_amount)

   	    else:
		# TODO: record the hit point
		while not self.is_close_to_min_distance(self.objects_right):
		    self.rotate(self.rotation_amount)

		self.rotate(30)
		self.r.sleep()
		# TODO: change this while loop to terminate when reaching hitpoint or destination
		while True:
		    right_rotation_amount = 0
		    while not self.is_close_to_min_distance(self.objects_right):
			amount = -self.rotation_amount
			self.rotate(amount)
			right_rotation_amount += amount

			if abs(right_rotation_amount) > 360:
			    # Usually happens on corners
			    break

			self.r.sleep()

		    self.rotate(2*self.rotation_amount)
		    self.translate(self.translation_amount)
	
     
    def obstacle_in_front(self):
	result = math.isnan(self.objects_center) == False and self.objects_center < self.front_nearness_ceiling and self.objects_center > self.front_nearness_floor
	return result


    def is_close_to_min_distance(self, distance):
	return math.isnan(distance)==False and distance >= self.objects_min and distance < self.objects_min + self.closeness_threshold and distance < 1.5


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
 	self.objects_min = min(msg.ranges)

    def print_distances(self):
	print("Object Distances: {} {} {} {}".format(self.objects_left, self.objects_center, self.objects_right, self.objects_min))	

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

