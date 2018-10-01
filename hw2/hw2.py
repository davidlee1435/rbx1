#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import LaserScan
from math import pi

class Bug2():
    def __init__(self):
        # Give the node a name
        rospy.init_node('bug2', anonymous=False)

        # Set rospy to exectute a shutdown function when exiting       
        rospy.on_shutdown(self.shutdown)
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        
        # How fast will we update the robot's movement?
        rate = 50
        
        # Set the equivalent ROS rate variable
        r = rospy.Rate(rate)
        
        # Set the forward linear speed to 0.2 meters per second 
        linear_speed = 0.2
        
        # Set the travel distance to 1.0 meters
        goal_distance = 1.0
        
        # How long should it take us to get there?
        linear_duration = goal_distance / linear_speed
        
        # Set the rotation speed to 1.0 radians per second
        angular_speed = 1.0
        
        # Set the rotation angle to Pi radians (180 degrees)
        goal_angle = pi
        
        # How long should it take to rotate?
        angular_duration = goal_angle / angular_speed
     
        self.run_pathfinder() 
        # Stop the robot
        self.cmd_vel.publish(Twist())
    
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


    def shutdown(self):
        # Always stop the robot when shutting down the node.
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        Bug2()
    except:
        rospy.loginfo("Out-and-Back node terminated.")

