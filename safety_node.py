#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import math

class Safety(object):
	def __init__(self):
		self.speed = 0
		self.brake_pub = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
		self.brake_bool_pub = rospy.Publisher('/brake_bool', Bool, queue_size=10)

		rospy.Subscriber('/odom', Odometry, self.odom_callback)
		rospy.Subscriber('/scan', LaserScan, self.scan_callback)

	def odom_callback(self, odom_msg):
		self.speed = odom_msg.twist.twist.linear.x

	def calculate_ttc(self, distance, relative_velocity):
		if relative_velocity > 0:
			ttc = distance / relative_velocity
		else:
			ttc = float('inf')
		return ttc

	def scan_callback(self, scan_msg):
		ranges = scan_msg.ranges
		intensities = scan_msg.intensities

		threshold_distance = 0.5
		threshold_ttc = 0.85

		for i in range(len(ranges)):
			distance = ranges[i]
			intensity = intensities[i]

			angle = scan_msg.angle_min + i * scan_msg.angle_increment

			relative_velocity = self.speed * math.cos(angle)

			relative_velocity_positive = max(relative_velocity, 0)

			ttc = self.calculate_ttc(distance, relative_velocity_positive)

			if distance < threshold_distance or ttc < threshold_ttc:
				brake_msg = AckermannDriveStamped()
				brake_msg.drive.speed = 0.0
				self.brake_pub.publish(brake_msg)

				brake_bool_msg = Bool(data=True)
				self.brake_bool_pub.publish(brake_bool_msg)
				return

			brake_bool_msg = Bool(data=False)
			self.brake_bool_pub.publish(brake_bool_msg)


def main():
	rospy.init_node('safety_node')
	sn = Safety()
	rospy.spin()
if __name__ == '__main__':
	main()
