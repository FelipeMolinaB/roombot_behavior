#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from behavior.msg import Status
from behavior.msg import State1Action,State1Goal,State1Feedback,State1Result
from behavior.msg import State2Action,State2Goal,State2Feedback,State2Result
from behavior.msg import State3Action,State3Goal,State3Feedback,State3Result
from behavior.msg import State4Action,State4Goal,State4Feedback,State4Result
from behavior.msg import State5Action,State5Goal,State5Feedback,State5Result
from behavior.msg import State7Action,State7Goal,State7Feedback,State7Result
from behavior.srv import RequestResponse
from behavior.srv import Request,GetPose,GetButton,ConfirmLoad,GuestConfirm
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool,Int8,Float32,UInt8
from sensor_msgs.msg import BatteryState

class BehaviorActions(object):
    ACTIVE    = 1
    SUCCEED   = 3
    ABORTED   = 4
    def __init__(self):
        """Parameters Inicialization """
        #Actions Parameters
        state1_action = rospy.get_param("~state1_action","state1")
        state2_action = rospy.get_param("~state2_action","state2")
        state3_action = rospy.get_param("~state3_action","state3")
        state4_action = rospy.get_param("~state4_action","state4")
        state5_action = rospy.get_param("~state5_action","state5")
        state7_action = rospy.get_param("~state7_action","state7")
        #Services Parameters
        confirm_load_service = rospy.get_param("~confirm_load_service", "confirm_load")
        guest_confirm_service = rospy.get_param("~guest_confirm_service", "guest_confirm")
        get_pose_service = rospy.get_param("~get_pose_service", "pose_request")
        get_button_service = rospy.get_param("~get_button_service", "button_request")
        #Subscribers topics Parameters
        orientation_adj_topic = rospy.get_param("~orientation_adj_topic", "/orientation_adjust")#Is there a request? topic
        press_button_cmplt_topic = rospy.get_param("~press_button_cmplt_topic","/press_button_complete") #Topic for trigger a change in Roombot's behavior
        battery_status_topic = rospy.get_param("~battery_status_topic","/battery_status")#Topic for trigger a change in Roombot's behavior
        current_floor_topic = rospy.get_param("~current_floor_topic","/current_floor")#Roombot's current_floor_topic
        goal_status_topic = rospy.get_param("~goal_status_topic","/roombot/move_base/status")#Roombot's current_floor_topic
        #Publishers topics Parameters
        target_button_topic = rospy.get_param("~target_button_topic","/target_button")               #Topic for the target button publisher
        goal_pose_topic = rospy.get_param("~goal_pose_topic", "/roombot/move_base_simple/goal")              #Topic for the goal pose publisher
        behevior_substatus_topic = rospy.get_param("~behevior_substatus_topic","/behevior_substatus")
        target_floor_topic = rospy.get_param("~target_floor_topic","/target_floor")#Roombot's target_floor_topic
        #Node Parameters
        rate = rospy.get_param("~rate",10)                                                      #behavior rate
        """Actions"""
        self.act_state1 = actionlib.SimpleActionServer(state1_action,State1Action,self.callback_satate1,False)
        self.act_state2 = actionlib.SimpleActionServer(state2_action,State2Action,self.callback_satate2,False)
        self.act_state3 = actionlib.SimpleActionServer(state3_action,State3Action,self.callback_satate3,False)
        self.act_state4 = actionlib.SimpleActionServer(state4_action,State4Action,self.callback_satate4,False)
        self.act_state5 = actionlib.SimpleActionServer(state5_action,State5Action,self.callback_satate5,False)
        self.act_state7 = actionlib.SimpleActionServer(state7_action,State7Action,self.callback_satate7,False)
        """Services"""
        #GUI module
        self.confirm_load = rospy.ServiceProxy(confirm_load_service,ConfirmLoad)
        self.guest_confirm = rospy.ServiceProxy(guest_confirm_service,GuestConfirm)
        #Data Bases module
        self.get_pose = rospy.ServiceProxy(get_pose_service,GetPose)
        self.get_button_code = rospy.ServiceProxy(get_button_service,GetButton)
        """Subscribers"""
        #Robotic Arm Module Topics
        self.sub_orientation_adj = rospy.Subscriber(orientation_adj_topic,Float32,self.callback_orientation_adj)
        self.sub_press_button_cmplt = rospy.Subscriber(press_button_cmplt_topic,Int8,self.callback_press_button_cmplt)
        #Sensors Module
        self.sub_battery_status = rospy.Subscriber(battery_status_topic,BatteryState,self.callback_battery_status)
        #Navigation Module
        self.sub_current_floor = rospy.Subscriber(current_floor_topic,UInt8,self.callback_current_floor)
        self.sub_goal_status = rospy.Subscriber(goal_status_topic,GoalStatusArray,self.callback_goal_status)
        """Publishers"""
        #Robotic Arm Module
        self.pub_target_button = rospy.Publisher(target_button_topic,Int8,queue_size = 10)
        #Navigation Module
        self.pub_goal_pose = rospy.Publisher(goal_pose_topic,PoseStamped, queue_size = 10)
        #FeedbackTrigger Module
        self.pub_behevior_substatus = rospy.Publisher(behevior_substatus_topic,UInt8, queue_size = 10)
        self.pub_target_floor = rospy.Publisher(target_floor_topic,UInt8, queue_size = 10)
        """Inicializations"""
        self.current_floor = 3
        self.goal_status = self.ACTIVE #0 WARNING: Changed for trials
        self.low_battery = False
        self.orientation_adj = False
        self.press_button_cmplt = False
        self.prev_goal_pose = None
        self.prev_substatus = None
        self.target_floor = UInt8()
        self.toggle = True
        self.stop = False
        self.substatus = 0
        """"Waiting for the services"""
        rospy.loginfo("Waiting for all services")
        rospy.wait_for_service(get_pose_service)
        rospy.wait_for_service(get_button_service)
        rospy.loginfo("Database services ready")
        rospy.wait_for_service(confirm_load_service)
        rospy.wait_for_service(guest_confirm_service)
        rospy.loginfo("Roombot GUI services Ready")
        """Starting Actions"""
        rospy.loginfo("Starting all actions")
        self.act_state1.start()
        self.act_state2.start()
        self.act_state3.start()
        self.act_state4.start()
        self.act_state5.start()
        self.act_state7.start()
        """Run Node's Main"""
        rospy.spin()

    """Actions Callbacks"""
    def callback_satate1(self,goal):
        self.go_to("delivery_room",None)
        self.stop = False
        result = State1Result()
        while(not rospy.is_shutdown()):
            if self.low_battery:
                result.end = False
                self.act_state1.set_aborted(result,"State 1 aborted due to low battery")
                return

            if self.act_state1.is_preempt_requested():
                result.end = True
                self.stop = True
                self.act_state1.set_succeeded(result,"State 1 succeed ")
                return

            if self.substatus != 0:
                self.go_to_other_floor(self.target_floor.data)
            else:
                if self.goal_status == self.ABORTED:
                    #Run a method when the goal is not reachable
                    rospy.loginfo("Goal pose can't be reached")
                    return
                else:
                    rospy.sleep(0.1)

    def callback_satate2(self,goal):
        self.go_to("delivery_room",None)
        self.stop = False
        result = State2Result()
        while(not rospy.is_shutdown()):
            if self.low_battery:
                self.stop = True
                result.end = False
                self.act_state2.set_aborted(result,"State 2 aborted due to low battery")
                return

            if self.substatus != 0:
                self.go_to_other_floor(self.target_floor.data)
            else:
                if self.goal_status == self.SUCCEED:
                    result.end = True
                    self.act_state2.set_succeeded(result,"Delivery room reached")
                    return
                elif self.goal_status == self.ABORTED:
                    result.end = False
                    self.act_state2.set_aborted(result,"Goal pose can't be reached")
                    return
                else:
                    rospy.sleep(0.1)

    def callback_satate3(self,goal):
        ## # WARNING: the action difiition has changed
        ids = []
        posible_requests = 0
        for request in goal.requests:
            if request.type == request.DELIVERY:
                posible_requests += 1
        counter = 0
        for request in goal.requests:
            if request.type == request.DELIVERY:
                counter += 1
                was_the_last = counter == posible_requests
                confirmation = self.confirm_load(request.request,was_the_last).confirmation
                if confirmation:
                    ids.append(request.request_id)
                else:
                    pass
                    ##### WARNING: Falta definir que pasa si da el servicio FAlse
        result = State3Result()
        result.ids = ids
        self.act_state3.set_succeeded(result)

    def callback_satate4(self,goal):
        #WARNING the action4 declaration was change for trial, please delete floor variable when
        self.go_to(goal.room,goal.floor)
        result = State4Result()
        rospy.sleep(0.1)
        while(not rospy.is_shutdown()):
            if self.substatus != 0:
                self.go_to_other_floor(self.target_floor.data)
            else:
                if self.goal_status == self.ABORTED:
                    #Run a method when the goal is not reachable
                    rospy.loginfo("Goal pose can't be reached")
                    result.end = False
                    self.act_state4.set_aborted(result)
                    return

                if self.goal_status == self.SUCCEED:
                    result.end = True
                    self.act_state4.set_succeeded(result)
                    return
                else:
                    rospy.sleep(0.1)

    def callback_satate5(self,goal):
        self.guest_confirm(goal.request)
        result = State5Result()
        result.id = goal.request_id
        self.act_state5.set_succeeded(result)

    def callback_satate7(self,goal):
        self.go_to("charge_base",None)
        charging = False
        result = State7Result()
        while(not(rospy.is_shutdown()) and self.low_battery):
            if not charging:
                if self.substatus != 0:
                    self.go_to_other_floor(self.target_floor.data)
                else:
                    if self.goal_status == self.SUCCEED:
                        charging = True
                        rospy.loginfo("Starting battery charge")
                    elif self.goal_status == self.ABORTED:
                        result.end = False
                        self.act_state7.set_aborted(result,"Goal pose can't be reached")
                        return
            rospy.sleep(0.1)
        result.end = True
        self.act_state7.set_succeeded(result,"Roombot has full battery")

    """Subscriber Topics Callbacks"""
    def callback_battery_status(self,msg):
        if msg.power_supply_health == msg.POWER_SUPPLY_HEALTH_DEAD:
            self.low_battery = True
        elif msg.power_supply_health == msg.POWER_SUPPLY_HEALTH_GOOD:
            self.low_battery = False

    def callback_current_floor(self,msg):
        self.current_floor = msg.data

    def callback_goal_status(self,msg):
        if len( msg.status_list ) != 0:
            last_goal_status = msg.status_list[-1]
            self.goal_status = last_goal_status.status

    def callback_orientation_adj(self,msg):
        self.orientation_adj = True

    def callback_press_button_cmplt(self,msg):
        self.press_button_cmplt = True

    """Publishers Topics Methods"""

    """Other Methods"""
    def go_to(self,pose_name,request_floor): ## WARNING: request_floor was added for trials
        goal_pose = self.get_pose(pose_name).pose
        ## WARNING: Changed for trials
        if int(goal_pose.pose.position.z) == -1.0:
            self.target_floor.data = self.current_floor
        elif int(goal_pose.pose.position.z) == -2.0:
            self.target_floor.data = request_floor
        else:##
            self.target_floor.data = float(goal_pose.pose.position.z)
        self.pub_target_floor.publish(self.target_floor)
        
        goal_pose.pose.position.z = 0
        if self.current_floor != int(self.target_floor.data):
            self.prev_goal_pose = goal_pose
            if self.substatus == 0:
                self.substatus = 1
        else:
            self.substatus = 0
            self.pub_goal_pose.publish(goal_pose)

    def wait(self,use_aborted_func=False,aborted_func=None,*func_args):
        while(not(rospy.is_shutdown()) and not self.stop):
            if self.goal_status == self.SUCCEED:
                break
            elif self.goal_status == self.ABORTED:
                if use_aborted_func:
                    if callable(aborted_func):
                        #Run Method when the elevator's outer panel is not reachable
                        if len(func_args) == 0:
                            aborted_func()
                        else:
                            aborted_func(func_args)
                else:
                    break
            elif self.goal_status == self.ACTIVE:
                rospy.sleep(0.1)
            else:
                rospy.logerr("Unknow goal status: %d",self.goal_status)
                rospy.sleep(0.1)
        if self.stop:
            return False
        else:
            return True

    def go_inside_elevator(self,elevators):
        self.toggle = not self.toggle
        if toggle:
            rospy.loginfo("Checking elevator 1 availability")
            self.pub_goal_pose.publish(elevators[0])
        else:
            rospy.loginfo("Checking elevator 2 availability")
            goal_pub.publish(elevators[1])
        rospy.sleep(0.1)

    def orientation_adjustment(self,button):
        rospy.loginfo("Performing the orientation adjustment")
        rospy.sleep(0.1)
        while self.goal_status != self.SUCCEED:
            rospy.sleep(0.1)
        rospy.sleep(0.5)
        self.pub_target_button.publish(button)
        self.orientation_adj = False
        self.press_button_cmplt = False
        while(not(rospy.is_shutdown()) and not self.press_button_cmplt):
            if (self.orientation_adj):
                self.orientation_adj = False
                rospy.sleep(0.5)
                self.pub_target_button.publish(button)
            rospy.sleep(0.1)
        rospy.loginfo("Elevator's inner panel was reached")

    def go_to_other_floor(self,target_floor):#  redefine this method using a while and substatus method
        self.pub_behevior_substatus.publish(self.substatus)
        if self.substatus == 1:
            #going to the outer panel
            rospy.loginfo("Going to the elevator's outer panel in floor %d, target floor: %d",self.current_floor,target_floor)
            goal_pose = self.get_pose('e3opanel').pose #str(self.current_floor)+"opanel").pose # WARNING:changed for trials
            self.pub_goal_pose.publish(goal_pose)
            rospy.sleep(0.1)
            if self.wait():
                if self.goal_status == self.SUCCEED:
                    self.prev_substatus = 1
                    self.substatus = 2
                elif self.goal_status == self.ABORTED:
                    rospy.loginfo("Elevator's outer panel in floor %d wasn't reached",self.current_floor)
                    #Define why the outer panel is unreachable
        #Performing the orientation adjustment
        elif self.substatus == 2:
            rospy.loginfo("Elevator's outer panel in floor %d was reached",self.current_floor)
            #Do orientation adjustment
            button = Int8()
            if self.prev_substatus == 1:
                if self.current_floor>target_floor:
                    button_name = "down"
                    button.data = self.get_button_code(button_name).button
                else:
                    button_name = "up"
                    button.data = self.get_button_code(button_name).button
                rospy.loginfo("Waiting the elevator")
                self.prev_substatus = 2
                self.substatus = 4 #3 WARNING: Changed for trials
            elif self.prev_substatus == 4:
                button_name = str(int(target_floor))
                button.data = self.get_button_code(button_name).button
                rospy.loginfo("Waiting to be in the correct floor")
                self.prev_substatus = 2
                self.substatus = 5
                rospy.sleep(0.1)
            self.orientation_adjustment(button)
        elif self.substatus == 3:
            rospy.loginfo("Going inside of the elevator")
            #
            elevator1 = self.get_pose("elevator1").pose
            elevetor2 = self.get_pose("elevator1").pose #2").pose WARNING: changed for trials
            self.pub_goal_pose.publish(elevator1)
            self.toggle = True
            if self.wait(True,self.go_inside_elevator,elevator1,elevetor2):
                if self.toggle: rospy.loginfo("Taking the elevator No. 1")
                else: rospy.loginfo("Taking the elevator No. 2")
                self.prev_substatus = 3
                self.substatus = 4
        elif self.substatus == 4:
            rospy.loginfo("Going to the elevator's inner panel ")
            #Going to the inner panel
            if self.target_floor.data%2 == 0:
                goal_pose = self.get_pose('e3ipanel_even').pose#+self.current_floor+"ipanel_even").pose # WARNING: Chancged for trial
            else:
                goal_pose = self.get_pose('e3ipanel_odd').pose#+self.current_floor+"ipanel_odd").pose # WARNING: Chancged for trial
            self.pub_goal_pose.publish(goal_pose)
            rospy.sleep(0.1)
            if self.wait():
                if self.goal_status == self.SUCCEED:
                    rospy.loginfo("Elevator's inner panel was reached")
                    self.prev_substatus = 4
                    self.substatus = 2
                elif self.goal_status == self.ABORTED:
                    rospy.loginfo("Elevator's inner panel wasn't reached")
                    #Define why the outer panel is unreachable
        #Waiting to be in the correct floor
        elif self.substatus == 5:
            rospy.loginfo("Waiting to be in the correct floor")
            while(not rospy.is_shutdown()):
                if self.current_floor == target_floor: break
                rospy.sleep(0.05)
            goal_pose = self.get_pose('e3opanel').pose #str(self.current_floor)+"opanel").pose # WARNING:changed for trials
            rospy.sleep(0.1)
            if self.wait():
                self.prev_substatus = 5
                self.substatus = 6
            else:
                self.prev_substatus = 5
                self.substatus = 4
        elif self.substatus == 6:
            rospy.loginfo("Going to the goal")
            self.pub_goal_pose.publish(self.prev_goal_pose)
            rospy.sleep(0.1)
            self.prev_substatus = 6
            self.substatus = 0
        else:
            rospy.logerr("Substautus error: substatus %d is invalid",self.substatus)

if __name__ == '__main__':
    rospy.init_node("Roombot_behavior", anonymous = True)
    BehaviorActions = BehaviorActions()
