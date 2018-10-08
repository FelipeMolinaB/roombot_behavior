#!/usr/bin/env python
# coding: utf-8
import rospy
import actionlib
from behavior.msg import Status
from behavior.msg import RequestStatus, RequestStatusArray
from behavior.msg import State1Action,State1Goal,State1Feedback,State1Result
from behavior.msg import State2Action,State2Goal,State2Feedback,State2Result
from behavior.msg import State3Action,State3Goal,State3Feedback,State3Result
from behavior.msg import State4Action,State4Goal,State4Feedback,State4Result
from behavior.msg import State5Action,State5Goal,State5Feedback,State5Result
from behavior.msg import State7Action,State7Goal,State7Feedback,State7Result
from behavior.srv import RequestResponse
from behavior.srv import Request,GetPose,GetButton
from std_msgs.msg import Bool,UInt8
from sensor_msgs.msg import BatteryState

class BehaviorController(object):
    PENDING    = 0
    ACTIVE     = 1
    SUCCEED    = 3
    CANCELED   = 4
    ABORTED    = 4
    def __init__(self):
        """Parameters Inicialization """
        #Actions Parameters
        state1_action = rospy.get_param("~state1_action","state1")                                       #Waiting for a request
        state2_action = rospy.get_param("~state2_action","state2")                                       #Going to the delivery room
        state3_action = rospy.get_param("~state3_action","state3")                                       #Waiting to be loaded
        state4_action = rospy.get_param("~state4_action","state4")                                       #Going to the guest's room
        state5_action = rospy.get_param("~state5_action","state5")                                       #Waiting for the guest's room
        state7_action = rospy.get_param("~state7_action","state7")                                       #Charging the roombot's battery
        #Services Parameters
        request_service = rospy.get_param("~request_service", "incoming_request")                        #From Hotel App to Roombot
        #Subscribers Parameters
        battery_status_topic = rospy.get_param("~battery_status_topic","/battery_status")                #Topic for monitoring the roombot's battery status
        current_floor_topic = rospy.get_param("~current_floor_topic","/current_floor")                   #Topic that advertise the current floor
        behevior_substatus_topic = rospy.get_param("~behevior_substatus_topic","/behevior_substatus")    #Topic that advertise the behavior substatus
        #requests_status_topic = rospy.get_param("~requests_status_topic","/requests_status")             ## WARNING: Please Check
        #Publishers Parameters
        behavior_status_topic = rospy.get_param("~behavior_status_topic","/behevior_status")             #Topic that advertise the behavior status
        #Other parameters
        self.floors = rospy.get_param("~floors", list(range(8)))#[1,3])                                                  #List of the floors available
        self.floors.sort()
        """Actions"""
        self.act_state1 = actionlib.SimpleActionClient(state1_action,State1Action)
        self.act_state2 = actionlib.SimpleActionClient(state2_action,State2Action)
        self.act_state3 = actionlib.SimpleActionClient(state3_action,State3Action)
        self.act_state4 = actionlib.SimpleActionClient(state4_action,State4Action)
        self.act_state5 = actionlib.SimpleActionClient(state5_action,State5Action)
        self.act_state7 = actionlib.SimpleActionClient(state7_action,State7Action)
        """Services"""
        self.svr_request = rospy.Service(request_service,Request,self.callback_incoming_request)
        """Publishers"""
        self.pub_behavior_status = rospy.Publisher(behavior_status_topic,Status,queue_size = 10)
        """Subscribers"""
        self.sub_battery_status = rospy.Subscriber(battery_status_topic,BatteryState,self.callback_battery_status)
        self.sub_behevior_substatus = rospy.Subscriber(behevior_substatus_topic,UInt8,self.callback_behevior_substatus)
        self.sub_current_floor = rospy.Subscriber(current_floor_topic,UInt8,self.callback_current_floor)
        """Waiting for Actions Servers"""
        rospy.loginfo("Waiting for all actions")
        self.act_state1.wait_for_server()
        self.act_state2.wait_for_server()
        self.act_state3.wait_for_server()
        self.act_state4.wait_for_server()
        self.act_state5.wait_for_server()
        self.act_state7.wait_for_server()
        rospy.loginfo("Behavior Actions Ready")
        """Node Configuration"""
        self.status = 1
        self.substatus = 0
        self.low_battery = False
        self.current_floor = None
        self.requests = dict(zip(self.floors,[[]]*len(self.floors)))
        self.requests_status = []
        """Run the node's main"""
        self.main()

    """Sercive Callback"""
    def callback_incoming_request(self,msg):
        rospy.loginfo("A Request was recieved")
        if True: ### WARNING: Here is missing the conditions that generates a False response
            msg = msg.request
            if msg.type == msg.DELIVERY:
                achivable = False
            else: #if msg.type == msg.ṔICK_UP
                achivable = True
            request = {"info":msg,"achivable":achivable}
            if len(self.requests[msg.floor]) == 0:
                self.requests[msg.floor] = [request]
            else:
                self.requests[msg.floor].append(request)
            self.requests[msg.floor] = sorted(self.requests[msg.floor], key=lambda request: request["info"].room)
            status = RequestStatus()
            status.request_id = msg.request_id
            status.status = status.PENDING
            self.requests_status.append(status)
            if self.act_state1.get_state() == self.PENDING or self.act_state1.get_state() == self.ACTIVE:
                self.act_state1.cancel_goal()
            response = True
        else:
            response = False
        return RequestResponse(response)

    """Subscriber Topics Callbacks"""
    def callback_battery_status(self,msg):
        if msg.power_supply_health == msg.POWER_SUPPLY_HEALTH_DEAD:
            self.low_battery = True
        elif msg.power_supply_health == msg.POWER_SUPPLY_HEALTH_GOOD:
            self.low_battery = False

    def callback_behevior_substatus(self,msg):
        self.substatus = msg.data

    def callback_current_floor(self,msg):
        self.current_floor = msg.data

    """Publisher Topics Methods"""
    def publish_status(self):
        status_msg = Status()
        if self.status == 1:
            status_msg.id = 1.0
            status_msg.description = "Roombot is wiating for a guest request"
        elif self.status == 2:
            status_msg.id = 2.0
            status_msg.description = "Request In Progess: Roombot has recieved a guest request"
        elif self.status == 3:
            status_msg.id = 3.0
            status_msg.description = "Roombot is waiting to be loaded with the guests' request"
        elif self.status == 4:
            status_msg.id = 4.0
            status_msg.description = "Roombot has been sent to the guest room"
        elif self.status == 5:
            status_msg.id = 5.0
            status_msg.description = "Roombot is waiting to the guest confirmation"
        elif self.status == 6:
            status_msg.id = 6.0
            status_msg.description = "Roombot is planning the next mission"
        elif self.status == 7:
            status_msg.id = 7.0
            status_msg.description = "Roombot is Returning to its charge Base"
        else:
            print("Invalid Status Error")
            exit()
        if self.substatus != 0:
            if self.substatus == 1:
                status_msg.id += 0.1
                status_msg.description += "\n\tRoombot is going to the elevator's outer panel"
            elif self.substatus == 2:
                status_msg.id += 0.2
                status_msg.description += "\n\tRoombot is performing a orientation adjustment"
            elif self.substatus == 3:
                status_msg.id += 0.3
                status_msg.description += "\n\tRoombot is going inside of the elevator"
            elif self.substatus == 4:
                status_msg.id += 0.4
                status_msg.description += "\n\tRoombot is going to the elevaotor's inner"
            elif self.substatus == 5:
                status_msg.id += 0.5
                status_msg.description += "\n\tRoombot is waiting to be in the correct floor"
            elif self.substatus == 6:
                status_msg.id += 0.6
                status_msg.description += "\n\tRoombot is going to the goal pose"
            else:
                print("\n\tInvalid Substatus Error")
                exit()
        self.pub_behavior_status.publish(status_msg)
        rospy.loginfo("Roombot Status: ID = %.1f , Description: %s", status_msg.id,status_msg.description)

    """Other Methods"""
    def set_status(self,request_id,status):
        for i,request_status in enumerate(self.requests_status):
            if request_status.request_id == request_id: break
        self.requests_status[i].status = status

    def set_achivable(self,request_id):
        for floor in self.floors:
            requests_list = self.requests[floor]
            if len(requests_list) == 0:
                continue
            request_ind = -1
            for i,r in enumerate(requests_list):
                if r["info"].request_id == request_id:
                    request_ind = i
                    break
            if request_ind != -1:
                break
        self.requests[floor][request_ind]["achivable"] = True

    def get_next_achivable(self,requests_list):
        request_ind = -1
        r = None
        print("requests_list",requests_list,list(enumerate(requests_list)))
        for i,r in enumerate(requests_list):
            print(i,r)
            if r["achivable"]:
                request_ind = i
                break

        return request_ind,r

    def main(self):
        while not rospy.is_shutdown():
            self.publish_status()
            if self.status == 1:
                self.act_state1.send_goal(State1Goal())
                self.act_state1.wait_for_result()
                state = self.act_state1.get_state()
                if state!=self.ABORTED: #Succeed or cancelled due to a Request
                    any_deliveryr = False
                    for requests_list in self.requests.values():
                        for request in  requests_list:
                            if request["info"].type == request["info"].DELIVERY:
                                any_deliveryr = True
                                break
                        if any_deliveryr: break
                    if any_deliveryr:
                        self.status = 2
                    else: #Only pick_up requests
                        self.status = 4
                else: #Aborted due to Low Battery
                    self.status = 7
            elif self.status == 2:
                self.act_state2.send_goal(State2Goal())
                self.act_state2.wait_for_result()
                self.status = 3
            elif self.status == 3:
                goal = State3Goal()
                """goal.requests = []
                request_templete = Request()
                for requests_list in self.requests.values():
                    for request in  requests_list:
                        print(type(request["info"]))
                        request_templete.request_id = request["info"].request_id
                        request_templete.type = request["info"].type
                        request_templete.request = request["info"].request
                        goal.requests.append(request_templete)"""## WARNING:
                goal.requests = [request["info"] for requests_list in self.requests.values() for request in  requests_list if request["info"].type == request["info"].DELIVERY]
                print(goal.requests)
                self.act_state3.send_goal(goal)
                self.act_state3.wait_for_result()
                ids = self.act_state3.get_result()
                for id in ids.ids:
                    self.set_achivable(id)
                self.status = 4
            elif self.status == 4:
                start_floor = self.current_floor
                skip = False
                if start_floor == self.floors[-1]:
                    request_ind,r = self.get_next_achivable(self.requests[start_floor])
                    print(request_ind)
                    if request_ind != -1:
                        skip = True
                    else:
                        start_floor = self.floors[0]
                if not skip:
                    print(start_floor,self.floors[-1]+1)
                    for floor in range(start_floor,self.floors[-1]+1):
                        requests_list = self.requests[floor]
                        request_ind,r = self.get_next_achivable(requests_list)
                        if request_ind != -1:
                            break
                else:
                    floor = start_floor
                self.set_status(r["info"].request_id,status=self.ACTIVE)
                goal_room = State4Goal()
                goal_room.room = r["info"].room
                goal_room.floor = r["info"].floor
                self.act_state4.send_goal(goal_room)
                self.act_state4.wait_for_result()
                self.status = 5
            elif self.status == 5:
                goal = State5Goal()
                goal.request_id = r["info"].request_id
                goal.request = r["info"].request
                self.act_state5.send_goal(goal)
                self.act_state5.wait_for_result()
                id_complete = self.act_state5.get_result()
                print(r)
                self.set_status(id_complete,status=self.SUCCEED)
                self.requests[floor].pop(request_ind)
                self.status = 6
            elif self.status == 6:
                any_achivable = False
                missing_requests = 0
                deliveryr = 0
                pick_upr = 0
                for requests_list in self.requests.values():
                    missing_requests += len(requests_list)
                    for request in requests_list:
                        if request["info"].type == request["info"].DELIVERY:
                            deliveryr += 1
                        else: #request["info"].type == request["info"].PICK_UP
                            pick_upr += 1
                        if request["achivable"]:
                            any_achivable = True
                if missing_requests == 0:
                    self.status = 1
                    self.request_status = []
                elif deliveryr == 0 and self.low_battery:
                    self.status = 7
                else:
                    if any_achivable:
                        self.status = 4
                    else:
                        self.status = 2
            elif self.status == 7:
                self.act_state7.send_goal(State7Goal())
                self.act_state7.wait_for_result()
                self.status = 1

if __name__ == '__main__':
    rospy.init_node("behavior_controller", anonymous = True)
    try:
        BehaviorController = BehaviorController()
    except rospy.ROSInterruptException:
        pass
