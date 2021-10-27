#! /usr/bin/python

from math import cos, sin
from typing import List, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker 


class LaserScannProcessing:
	def __init__(self, radius: int = 5, resolution: float = 1e-1, eps: float = 1e-1):
		# grid params
		self.radius = radius
		self.resolution = resolution
		self.grid_size = 2 * int(self.radius / self.resolution) + 1
		
		# filter threshold
		self.eps = eps
		
		self.subscriber = rospy.Subscriber("/base_scan", LaserScan, self.callback)
		self.marker_publisher = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
		self.grid_publisher = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

	def _simple_laser_filter(self, ranges: Tuple[float, ...]) -> List[int]:
		valid_indices = [0]
		x_0, x_1 = ranges[0], ranges[1]
		
		for i, x in enumerate(ranges[2:]):
			mean = (x + x_0) / 2.0
			if abs(mean - x_1) < self.eps:
				 valid_indices.append(i + 1)
			x_0, x_1 = x_1, x

		valid_indices.append(len(ranges) - 1)	
		
		return valid_indices
	
	@staticmethod
	def _convert_to_cartesian(ranges, angle_min, angle_increment):
		angles = [angle_min + angle_increment * i for i in range(len(ranges))]
		x = [range_ * cos(angle) for range_, angle in zip(ranges, angles)]
		y = [range_ * sin(angle) for range_, angle in zip(ranges, angles)]
		
		return x, y
	
	@staticmethod
	def _create_marker():
		marker = Marker()
		
		marker.header.frame_id = "base_laser_link"
		marker.header.stamp = rospy.Time.now()

		# Set shape
		marker.type = 8
		marker.id = 0
		marker.action = 0

		# Set the scale of the marker
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		# Set the color
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0
		
		# Set the pose of the marker
		marker.pose.position.x = 0
		marker.pose.position.y = 0
		marker.pose.position.z = 0
		
		return marker
	
	def _create_grid(self, x_valid, y_valid):
		grid = OccupancyGrid()
		
		grid.header.frame_id = "base_laser_link"
		
		grid.info.resolution = self.resolution
		grid.info.width = self.grid_size
		grid.info.height = self.grid_size
		
		grid.info.origin.position.x = -self.radius
		grid.info.origin.position.y = -self.radius
		grid.info.origin.position.z = 0
		
		data = np.zeros((self.grid_size, self.grid_size), dtype=int)
		
		for x, y in zip(x_valid, y_valid):
			if abs(x) < self.radius and abs(y) < self.radius:
				i = int((x + self.radius) / self.resolution)
				j = int((y + self.radius) / self.resolution)
				data[j, i] = 100
		data = data.flatten()		
		grid.data = data
		
		return grid
		
	def callback(self, msg: LaserScan):		
		valid_indices = self._simple_laser_filter(msg.ranges)
		
		x_lst, y_lst = self._convert_to_cartesian(msg.ranges,
		 				           msg.angle_min, 
		 				           msg.angle_increment)
		
		valid_x = [x_lst[i] for i in valid_indices]
		valid_y = [y_lst[i] for i in valid_indices]
		
		marker = self._create_marker()
		marker.points = [Point(x, y, 0) for x, y, in zip(valid_x, valid_y)]
		self.marker_publisher.publish(marker)
		
		grid = self._create_grid(valid_x, valid_y)
		self.grid_publisher.publish(grid)
		

if __name__ == "__main__":
	rospy.init_node("laser_scann_processing")
	LaserScannProcessing()
	rospy.spin()
