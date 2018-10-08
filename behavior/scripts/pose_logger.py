#!/usr/bin/env python
import rospy
import pandas as pd
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseWithCovariance
from behavior.srv import SavePose

class PoseLogger(object):
	def __init__(self):
		"""Parameters Inicialization """
		save_request_service = rospy.get_param("~save_request_service","save_request")   #Topic for the status of the Roombot behavior
		pose_update_topic = rospy.get_param("~pose_update_topic","/roombot/poseupdate")   #Topic for the status of the Roombot behavior
		"""Service"""
		self.save_pose = rospy.ServiceProxy(save_request_service,SavePose)
		"""Subscriber"""
		self.sub_pose_update = rospy.Subscriber(pose_update_topic,PoseWithCovarianceStamped,self.callback_pose_update)
		"""Node Configuration"""
		self.get_pose = False
		self.current_pose = PoseWithCovariance()
		rospy.wait_for_service(save_request_service)
		self.pose = {"pose_name":"","pose":self.current_pose}
		self.main()

	"""Subscriber Callback"""
	def callback_pose_update(self,msg):
		if self.get_pose:
			self.current_pose = msg.pose
			self.get_pose = False

	"""Other Methods"""
	def main(self):
		while not rospy.is_shutdown():
			try:
				input = raw_input("Current Pose Name, floor (\'e\' to exit): ")
				pose_name,floor = input.split(',')
				self.get_pose = True
				while(self.get_pose): pass
				self.current_pose.pose.position.z = float(floor)
				self.pose = {"pose_name":pose_name,"pose":self.current_pose}
				response = self.save_pose(self.pose["pose_name"],self.pose["pose"])
				if response.error:
					rospy.logerr(response.description)
				else:
					rospy.loginfo("Pose \"%s\" has been saved",pose_name)
			except:
				break
		exit()

if __name__ == '__main__':
	rospy.init_node("pose_logger", anonymous = True)
	try:
		PoseLogger = PoseLogger()
	except rospy.ROSInterruptException:
		pass
