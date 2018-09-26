import rospy
import random
from PyQt5 import QtCore, QtGui, QtWidgets
from gtk.gdk import screen_width, screen_height
from std_msgs.msg import Float32

class UiEyes(object):
    def __init__(self):
        rospy.init_node("eyes", anonymous = True)
        """Parameters Inicialization """
        detected_person_topic = rospy.get_param("~detected_person_topic","/detected_person_zone")
        self.eyes_path = rospy.get_param("eyes_path", "/home/innovacion/roombot_ws/src/roombot_gui/ui/img/w_eyes2.svg")
        self.close_eyes_path = rospy.get_param("eyes_path", "/home/innovacion/roombot_ws/src/roombot_gui/ui/img/b_eyes2.svg")
        self.pupil_path = rospy.get_param("pupil_path","/home/innovacion/roombot_ws/src/roombot_gui/ui/img/pupil2.svg")
        self.blink_duration = rospy.get_param("blink_duration",0.15)
        self.blink_period = rospy.get_param("blink_duration",5.0)
        """Subscribers"""
        self.sub_detected_person = rospy.Subscriber(detected_person_topic, Float32, self.callback_detected_person, queue_size = 1)
        """Node Configuration"""
        self.MainWindow = QtWidgets.QMainWindow()
        self.setupUi()
        self.offset = 0.5
        self.target_offset = 0.5
        self.move = False
        self.open_eyes = False
        self.start_time = rospy.get_time()
        self.people_tracking = PeopleTracking()
        rospy.Timer(rospy.Duration(self.blink_duration), self.blink, oneshot=False)

    def callback_detected_person(self,msg):
        if msg.data != -1.0:
            if msg.data >= 1:
                self.target_offset = 1
            elif msg.data <= 0.0:
                self.target_offset = 0
            else:
                self.target_offset = msg.data
            self.is_big = True
            self.move = True
        else:
            self.is_big = False
            self.move = False

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(screen_width(), screen_height()-10)
        self.MainWindow.setStyleSheet("border-image: url("+self.eyes_path+") 0 0 0 0 stretch stretch;")
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.d = int(self.MainWindow.size().width()*0.515)
        self.eye_size = (int(self.MainWindow.size().width()*0.46666),self.MainWindow.size().height())
        self.pupil_size_l = (int(self.MainWindow.size().width()*0.30),int(self.MainWindow.size().width()*0.30))
        self.pupil_size_s = (int(self.MainWindow.size().width()*0.20),int(self.MainWindow.size().width()*0.20))
        self.is_big = True
        self.initial_pose = (int((self.eye_size[0]/2)-(self.pupil_size_s[0]/2)),int((self.eye_size[1]/2)-(self.pupil_size_s[1]/2)))


        self.pupil_r = QtWidgets.QLabel(self.centralwidget)
        self.pupil_r.setGeometry(QtCore.QRect(self.initial_pose[0], self.initial_pose[1], self.pupil_size_s[0], self.pupil_size_s[1]))
        self.pupil_r.setStyleSheet("border-image: url("+self.pupil_path+") 0 0 0 0 stretch stretch;")
        self.pupil_r.setText("")
        self.pupil_r.setObjectName("pupil_r")
        self.pupil_l = QtWidgets.QLabel(self.centralwidget)
        self.pupil_l.setGeometry(QtCore.QRect(self.initial_pose[0]+self.d, self.initial_pose[1], self.pupil_size_s[0], self.pupil_size_s[1]))
        self.pupil_l.setStyleSheet("border-image: url("+self.pupil_path+") 0 0 0 0 stretch stretch;")
        self.pupil_l.setText("")
        self.pupil_l.setObjectName("pupil_l")
        self.MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.MainWindow.setWindowTitle(_translate("MainWindow", "Eyes"))
        self.pupil_r.setText(_translate("MainWindow", "TextLabel"))
        self.pupil_l.setText(_translate("MainWindow", "TextLabel"))

    def move_pupils(self,x = None):
        if self.is_big:
            size = self.pupil_size_l
        else:
            size = self.pupil_size_s

        if x != None:
            y = 0.115*x + 0.02
            new_pos = [int(y*self.MainWindow.size().width()),int((self.eye_size[1]/2)-(size[1]/2))]
        else:
            cx = ui.pupil_r.geometry().center().x()
            new_pos = [int(cx-(size[1]/2)),int((self.eye_size[1]/2)-(size[1]/2))]
        self.pupil_r.setGeometry(QtCore.QRect(new_pos[0],new_pos[1],size[0],size[1]))
        self.pupil_l.setGeometry(QtCore.QRect(new_pos[0]+self.d,new_pos[1],size[0],size[1]))

    def blink(self,event):
        if not(self.open_eyes):
            if (rospy.get_time() - self.start_time) >= self.blink_period:
                self.pupil_r.hide()
                self.pupil_l.hide()
                self.MainWindow.setStyleSheet("border-image: url("+self.close_eyes_path+") 0 0 0 0 stretch stretch;")
                self.open_eyes = True
                self.start_time = rospy.get_time()
        else:
            self.MainWindow.setStyleSheet("border-image: url("+self.eyes_path+") 0 0 0 0 stretch stretch;")
            self.pupil_r.show()
            self.pupil_l.show()
            self.open_eyes = False

    def show(self):
        self.MainWindow.show()
        self.people_tracking.start()

    def close(self):
        self.MainWindow.close()

class PeopleTracking(QtCore.QThread):
    def __init__(self):
        super(PeopleTracking,self).__init__()

    def run(self):
        while  self.isRunning() and not rospy.is_shutdown():
            if ui.move:
                if(ui.target_offset != ui.offset):
                    step = (ui.target_offset - ui.offset) / 5
                else:
                    step = 0
                ui.offset += step
                ui.move_pupils(ui.offset)
            else:
                ui.move_pupils()
            #rospy.sleep(50e-3)

"""
class Blinking(QtCore.QThread):
    def __init__(self):
        super(Blinking,self).__init__()

    def run(self):
        while  self.isRunning() and not rospy.is_shutdown():
            ui.blink(0)
            sleep(0.3)
            ui.blink(1)
            sleep(round(random.uniform(3.0, 5.0),2))
"""
if __name__ == "__main__":
    try:
        import sys
        app = QtWidgets.QApplication(sys.argv)
        ui = UiEyes()
        print("a")
        ui.show()
        print("b")
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        print("run")
        ui.move_eyes.terminate()
        ui.close()
