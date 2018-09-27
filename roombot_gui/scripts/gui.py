#!/usr/bin/env python
import rospy
from rospkg import RosPack
from std_msgs.msg import Bool
from behavior.srv import ConfirmLoad,GuestConfirm
from behavior.srv import ConfirmLoadResponse,GuestConfirmResponse
from time import sleep
from PyQt5 import QtCore
#from threading import Thread

class RobotGUI(QtCore.QObject):#object):
    def __init__(self):
        super(RobotGUI,self).__init__(None)
        """Parameters Inicialization """
        confirm_load_service = rospy.get_param("~confirm_load_service", "confirm_load")
        guest_confirm_service = rospy.get_param("~guest_confirm_service", "guest_confirm")
        eyes_ui_path = rospy.get_param("~eyes_ui_path", RosPack().get_path("roombot_gui")+"/scripts/eyes3.py")
        self.confirmation_ui_path = rospy.get_param("~confirmation_ui_path",RosPack().get_path("roombot_gui")+"/scripts/confirm.py")
        """Services"""
        self.srv_confirm_load = rospy.Service(confirm_load_service,ConfirmLoad,self.callback_confirm_load)
        self.srv_guest_confirm = rospy.Service(guest_confirm_service,GuestConfirm,self.callback_guest_confirm)
        """Node Configuration"""
        self.eyes = QtCore.QProcess(self)
        self.confirm = QtCore.QProcess(self)
        self.eyes.start("python " + eyes_ui_path)
        self.eyes.waitForStarted()
        self.confirmation = False
        rospy.spin()
        self.kill_all()

    """Services Callbacks"""
    def callback_confirm_load(self,msg):
        self.confirm.start("python " + self.confirmation_ui_path)
        self.confirm.waitForFinished()
        return ConfirmLoadResponse(True)

    def callback_guest_confirm(self,msg):
        self.confirm.start("python " + self.confirmation_ui_path)
        self.confirm.waitForFinished()
        return GuestConfirmResponse(True)

    def kill_all(self):
        self.eyes.kill()
        self.eyes.terminate()
        self.confirm.kill()
        self.confirm.terminate()

if __name__ == '__main__':
    rospy.init_node("gui_node", anonymous = True,disable_signals=True)
    try:
        RobotGUI = RobotGUI()
    except rospy.ROSInterruptException:
        print("run")
        RobotGUI.eyes.kill()
        RobotGUI.eyes.terminate()
        RobotGUI.confirm.kill()
        RobotGUI.confirm.terminate()
