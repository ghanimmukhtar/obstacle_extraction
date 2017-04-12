#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    19/03/2017
""" 

import numpy as np

import collisiondetection
import rospy

import sys

from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout



import time
# TODO: Update trajectory file
default_archive_file = "demo_trj.dat"


#TODO: Define message type and topic of the incoming bounding box data
#from whatever_ros_package.msg import whatever_the_boundingbox_message_type_is as BBoxesMsgType
#from std_msgs.msg import String as BBoxesMsgType # Test with dummy String messages
from std_msgs.msg import Float64MultiArray as BBoxesMsgType # Test with dummy String messages

# Bounding box topic
#input_msg_topic = "/path/to/obstacles/topic"
input_msg_topic = "/obstacle_corners" # Test with dummy String messages


#Output index type
from std_msgs.msg import Int64 as TrajIndexMsgType
#(Note : we could also use a Int64MultiArray to return all the good trajectories
#and choose one in the downstream node)
#Output index topic
output_msg_topic = "/trajselector/trajectory_index"


class RosTrajSelector:
	def __init__(self,archivefile,visualisation=True):
		self.archivefile = archivefile
		self.archive = collisiondetection.load_file(archivefile)
		self.current_obstacles = ()
		self.visu = visualisation
		if(self.visu):
			collisiondetection.plt.show()

		
	def select_trajectory_among_goods(self, good_trajectories, distances):
		# If you want to add some more elaborate code to
		# select the right trajectory (softmax weighted by
		# distances to obstacles ?) do it here
		# * good_trajectories: indices of valid tajectories
		#   sorted by decreasing min distance to obstacle
		# * distances: min distances to obstacles (in m) for
		#   each good_trajectory. (If no obstacle was there
		#   they will just all be np.inf)
		
		# For now: just select the best trajectory if there is an
		# obstacle, or a random one if there is no obstacle
		if(distances[0] == np.inf): # No obstacle: random choice
			i = np.random.randint(len(good_trajectories))
		else:
			i = 0 # Obstacle: select the best
		return good_trajectories[i], distances[i]

	def process_obstacles_message(self,rosmessage):
		# TODO: Process the message here and return obstacle data.
		# - Obstacle data is a tuple of obstacles. You must make a tuple even
		#   if there is only 1 obstacle) (If there is no obstacle you can give
		#   None or an empty tuple)
		# - Each obstacle is a tuple of 2 points (two opposite corners of the
		#   bbox)
		# - Each point is a tuple of (x,y,z) coordinates
		
		# Float64MultiArray processing		
		#if(rosmessage.layout.dim[0].size > 0):
		
		[x1, y1, z1, x2, y2, z2] = rosmessage.data
		obstacle = ((x1,y1,z1),(x2,y2,z2))
		obstacledata = (obstacle,)
#		
		#else:
		#	obstacledata = ()
		
		
		# Test: we will just use the test_obstacle from collisiondetection if
		#       the string message says "obstacle" and no obstacle otherwise		
#		self.current_obstacles = (collisiondetection.test_obstacle,) if(rosmessage.data == "obstacle") else ()
#		return True
		
		if(obstacledata == self.current_obstacles):
			return False # has not changed
		else:
			self.current_obstacles = obstacledata
			return True # new data


	
	def cb_obstacles(self,message):
		#print("Obstacle info received.")
		is_new_obstacles = self.process_obstacles_message(message)
		if(is_new_obstacles):
			self.select_trajectory()

	def plot_trajectory(self,index):
		collisiondetection.plt.close()
		collisiondetection.plot_traj(self.archive[index],obstacles=self.current_obstacles)
		collisiondetection.plt.pause(.001)
		

	def select_trajectory(self):		
		print("Selecting best trajectory")
		then = time.time()
		sorted_goods, sorted_good_dists, bads = collisiondetection.eval_trajs(self.archive, self.current_obstacles)
		now = time.time()
		print("Trajectories evaluated in %.2f seconds" % (now - then))
		print("Found %d valid trajectories." % len(sorted_goods))
		print("They are, in order: %s" % str(sorted_goods))
		if not sorted_goods:
			print("FAIL: No valid trajectory found :(")
		else:
			selected, d = self.select_trajectory_among_goods(sorted_goods,sorted_good_dists)
			if(self.visu):
				self.plot_trajectory(selected)
			print("Selecting trajectory #%d (distance = %.3f m)" % (selected,d))
			msgout = TrajIndexMsgType(selected)
			self.result_publisher.publish(msgout)
	
	def start_node(self):
		rospy.init_node('trajselector', anonymous=True)
		self.obstacles_sub = rospy.Subscriber(input_msg_topic, BBoxesMsgType, self.cb_obstacles)
		self.result_publisher = rospy.Publisher(output_msg_topic, TrajIndexMsgType, queue_size=1)
		rospy.spin()

#
#
if(__name__ == '__main__'):
	myfile = default_archive_file if(len(sys.argv) < 2) else sys.argv[1]
	print("Using archive file %s" % myfile)
	node = RosTrajSelector(myfile)
	node.start_node()
#


