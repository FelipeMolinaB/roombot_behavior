#!/usr/bin/env python
import rospy
import random
from behavior.srv import Request
from behavior.msg import RequestMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool,Int8,UInt8
from sensor_msgs.msg import BatteryState

class RoombotBehavior(object):
	def __init__(self):
		"""Parameters Inicialization """
		request_service = rospy.get_param("~request_service", "incoming_request")
		current_floor_topic = rospy.get_param("~current_floor_topic","/current_floor")   		    #Roombot's current_floor_topic
		target_floor_topic = rospy.get_param("~target_floor_topic","/target_floor")               #Topic for the target button publisher
		press_button_cmplt_topic = rospy.get_param("~press_button_cmplt_topic","/press_button_complete") #Topic for trigger a change in Roombot's behavior
		new_mission_topic = rospy.get_param("~new_mission_topic","/new_mission")
		battery_status_topic = rospy.get_param("~battery_status_topic","/battery_status")#Topic for trigger a change in Roombot's behavior
		rate = rospy.get_param("~rate",10)                                                           #behavior rate
		"""Services"""
		self.svr_request = rospy.ServiceProxy(request_service,Request)
		"""Subscribers"""
		self.sub_press_button_cmplt = rospy.Subscriber(press_button_cmplt_topic,Int8,self.callback_press_button_cmplt)		#Sensors Module
		self.sub_new_mission = rospy.Subscriber(new_mission_topic,Int8,self.callback_new_mission)
		"""Publishers"""
		self.pub_current_floor = rospy.Publisher(current_floor_topic,UInt8,queue_size = 10)
		self.pub_target_floor = rospy.Publisher(target_floor_topic,UInt8, queue_size = 10)
		self.pub_battery_status = rospy.Publisher(battery_status_topic,BatteryState,queue_size = 10)
		"""Node Configuration"""
		self.rate = rospy.Rate(rate)
		self.current_floor = rospy.get_param("~current_floor", 3)
		self.target_floor = 3
		self.request_id = 0
		self.rooms = ["paulo","felipe","daniel","cristian","juancho","david","julio","john"]
		self.products = ["Agua","Gaseosa","Toalla","Pan","Almuerzo","Desayuno","Cena","Cigarrillos","M&Ms","Papas"]
		"""Run Node's Main"""
		self.main()

	"""Subscribers Topics Callbacks"""
	def callback_press_button_cmplt(self,msg):
		self. current_floor = self.target_floor


	def callback_new_mission(self,msg):
		self.target_floor = msg.data
		request = RequestMsg()
		self.pub_target_floor.publish(self.target_floor)
		self.request_id += 1
		request.request_id
		request.type = request.DELIVERY
		request.floor = msg.data
		request.room = "felipe"#random.choice(self.rooms)
		items = random.randint(1,6)
		request.request = []
		products = []
		for i in range(items):
			while True:
				product = random.choice(self.products)
				if not (product in products):
					products.append(product)
					break
		for product in products:
			request.request.append(product+";"+str(random.randint(1,10)))
		print(request)
		"""if self.svr_request(request):
			print("OK")
		else:
			print("error")"""

	def main(self):
		battery = BatteryState()
		battery.power_supply_health = battery.POWER_SUPPLY_HEALTH_GOOD
		print("running")
		while not rospy.is_shutdown():
			self.pub_current_floor.publish(self.current_floor)
			self.pub_target_floor.publish(self.target_floor)
			self.pub_battery_status.publish(battery)
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node("Roombot_behavior", anonymous = True)
	try:
		Roombotbehavior = RoombotBehavior()
	except rospy.ROSInterruptException:
		pass
