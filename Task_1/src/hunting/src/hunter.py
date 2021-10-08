#! /usr/bin/python

import math

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Hunter:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

        rospy.Subscriber('/turtle1/pose', Pose, self.move)
        rospy.Subscriber('/hunter/pose', Pose, self.update_pos)
        self.publisher = rospy.Publisher('/hunter/cmd_vel', Twist, queue_size=10)

    def update_pos(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    def move(self, msg: Pose):
        x_bias = msg.x - self.x
        y_bias = msg.y - self.y
        angle = -self.theta + math.atan2(y_bias, x_bias)

        msg_hunter = Twist()
        msg_hunter.angular.z = angle
        msg_hunter.linear.x = x_bias
        msg_hunter.linear.y = y_bias

        self.publisher.publish(msg_hunter)


if __name__ == '__main__':
    rospy.init_node('hunting')
    Hunter()
    rospy.spin()
