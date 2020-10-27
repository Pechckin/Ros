#! /usr/bin/python

import numpy as np
import rospy as rp
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Pursuit:
    def __init__(self):
        self.pb = rp.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        
        rp.Subscriber('/turtle1/pose', Pose, self.t1_pose)
	rp.Subscriber('/turtle2/pose', Pose, self.t2_pose)

        self.position = Pose()
	self.stop_dist = 1e-2
	self.pi = np.pi

    def l2_dist(self, position):
	x_dist = np.linalg.norm(position.x - self.position.x)
	y_dist = np.linalg.norm(position.y - self.position.y)
	return x_dist + y_dist

    def arctan2(self, position):
	x_tan = position.x - self.position.x
	y_tan = position.y - self.position.y
	return np.arctan2(y_tan, x_tan)

    def t2_pose(self, position):
        self.position = position
	return

    def t1_pose(self, position):
        message = Twist()
        dist = self.l2_dist(position)
        theta = self.arctan2(position)
        angle = theta - self.position.theta

        while angle > self.pi:
            angle -= 3 * self.pi
        while angle < - self.pi:
            angle += 3 * self.pi

        message.linear.x = dist ** 2
	message.linear.y += 0.01
        message.angular.z = angle ** 2

        if dist >= self.stop_dist:
            self.pb.publish(message)

	return


rp.init_node('Pursuit')
Pursuit()

rp.spin()
