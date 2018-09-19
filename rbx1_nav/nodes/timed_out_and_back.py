#!/usr/bin/env python

""" timed_out_and_back.py - Version 1.2 2014-12-14

    A modified basic demo of the using odometry data to move the robot along
    and out-and-back trajectory.

    Original code created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

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

	while not rospy.is_shutdown():
                move = raw_input('Enter T for translation, R for rotation, or Q for quit: ')
		if move == 'T' or move == 't':
                        distance = float(input('Distance? '))
                        print(distance)
			self.translate(distance)
                elif move == 'R' or move == 'r':
                        angle = float(input('Angle? '))
                        self.rotate(angle)
                elif move == 'Q' or move == 'q':
                        break
                else:
                        print('Move not recognized. Try T, R, or Q.')
	

    def rotate(self, goal_angle):
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

 
if __name__ == '__main__':
    clear = os.system("killall -9 rviz")
    clear = os.system("killall -9 roscore")
    clear2 = os.system("killall -9 rosmaster")
    launch = os.system("roslaunch rbx1_bringup fake_turtlebot.launch &")
    result = os.system("rosrun rviz rviz -d `rospack find rbx1_nav`/sim.rviz &")
    
    try:
	OutAndBack()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

