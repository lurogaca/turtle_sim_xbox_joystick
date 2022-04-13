#!/usr/bin/python3


import rospy

from geometry_msgs.msg import Twist

from sensor_msgs.msg import Joy


def initial(data):
	move = Twist()
	move.linear.x = 2.5 * data.axes[1]
	move.angular.z = 2.5 * data.axes[0]

	pub.publish(move)

def movement():
	global pub
	pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size =1)
	rospy.init_node('Joy_Movement')
	rospy.Subscriber("joy", Joy, initial)

	rospy.spin()

if __name__ == '__main__':
	movement()
