#!/usr/bin/env python
import rospy
import pandas as pd
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Bool,UInt8
from behavior.srv import SavePose,GetPose,GetButton
from behavior.srv import SavePoseResponse,GetButtonResponse,GetPoseResponse

class DataBase(object):
	def __init__(self):
		"""Parameters Inicialization """
		self.poses_path = rospy.get_param("~poses_db_path","/home/roombot/ros_workspaces/behavior_ws/src/behavior/include/poses.csv")     #Pose Data Base path
		self.buttons_path = rospy.get_param("~buttons_db_path","/home/roombot/ros_workspaces/behavior_ws/src/behavior/include/buttons.csv")   #Buttons Data Base path
		r_nw_data_base = rospy.get_param("~r_nw_data_base",True)   #Topic for the status of the Roombot behavior
		save_request_service = rospy.get_param("~save_request_service","save_request")   #Topic for the status of the Roombot behavior
		pose_request_service = rospy.get_param("~pose_request_service","pose_request")   #Topic for the status of the Roombot behavior
		button_request_service = rospy.get_param("~button_request_service","button_request")   #Topic for the status of the Roombot behavior
		self.pose_frame_id = rospy.get_param("~pose_frame_id","map")
		"""Services"""
		if not r_nw_data_base:
			self.srv_save_request =	rospy.Service(save_request_service,SavePose,self.callback_save_request)
		else:
			self.srv_pose_request =	rospy.Service(pose_request_service,GetPose,self.callback_pose_request)
			self.srv_button_request = rospy.Service(button_request_service,GetButton,self.callback_button_request)
		"""Node Configuration"""
		self.poses_db = pd.read_csv(self.poses_path,index_col = False)
		self.buttons_db = pd.read_csv(self.buttons_path,dtype={"Nombres":str,"Boton":str},index_col = False)
		if r_nw_data_base:
			rospy.init_node("data_base", anonymous = True)
			rospy.loginfo("Data Bases in read mode")
		else:
			rospy.init_node("feed_poses_db", anonymous = True)
			rospy.loginfo("Data Bases in read/write mode")
		rospy.spin()

	"""Services Callbacks"""
	def callback_save_request(self,msg):
		rospy.loginfo("A save pose request was recieved")
		error = [False,""] #[error,description]
		if msg.pose_name.strip() == "":
			error = [True,"No name was acquired"]
		else:
			try:
				new_name = list(self.poses_db.Nombres).index(msg.pose_name)
				error = [True,"The pose "+msg.name+" is already saved"]
			except:
				pose = [None]*9
				pose[0] = msg.pose_name
				pose[1] = msg.pose.pose.position.x
				pose[2] = msg.pose.pose.position.y
				pose[3] = msg.pose.pose.position.z
				pose[4] = msg.pose.pose.orientation.x
				pose[5] = msg.pose.pose.orientation.y
				pose[6] = msg.pose.pose.orientation.z
				pose[7] = msg.pose.pose.orientation.w
				pose[8] = ""
				self.poses_db = self.poses_db.append(dict(zip(self.poses_db.columns,pose)),ignore_index=True)
				self.poses_db.to_csv(self.poses_path,index = False)
		return SavePoseResponse(error[0],error[1])

	def callback_pose_request(self,msg):
		pose = PoseStamped()
		try:
			pose_index = list(self.poses_db.Nombre).index(msg.pose_name)
			raw_pose = list(self.poses_db.iloc[[pose_index]].values[0])[1:8]
			pose.header.frame_id = self.pose_frame_id
			pose.pose.position.x = raw_pose[0]
			pose.pose.position.y = raw_pose[1]
			pose.pose.position.z = raw_pose[2]
			pose.pose.orientation.x = raw_pose[3]
			pose.pose.orientation.y = raw_pose[4]
			pose.pose.orientation.z = raw_pose[5]
			pose.pose.orientation.w = raw_pose[6]
			rospy.loginfo("The %s pose was sent",msg.pose_name)
		except:
			rospy.logerr("The %s pose does not exist",msg.pose_name)
		return GetPoseResponse(pose)

	def callback_button_request(self,msg):
		button = -1
		try:
			button_index = list(self.buttons_db.Nombre).index(msg.button_name)
			button = int(list(self.buttons_db.iloc[[button_index]].values[0])[1])
		except:
			rospy.logerr("The %s button does not exist",msg.button_name)
		return GetButtonResponse(button)

if __name__ == '__main__':
	try:
		DataBase = DataBase()
	except rospy.ROSInterruptException:
		pass
