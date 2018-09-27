import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from rospkg import RosPack
from PyQt5 import QtCore, QtGui, QtWidgets
from pyautogui import size
from time import sleep
from std_msgs.msg import Float32

class UiEyes(object):
    def __init__(self):
        """Parameters Inicialization """
        detected_person_topic = rospy.get_param("~detected_person_topic","/detected_person_zone")
        self.eyes_path = rospy.get_param("~eyes_path", RosPack().get_path("roombot_gui")+"/ui/img/w_eyes2.svg")
        self.close_eyes_path = rospy.get_param("~close_eyes_path", RosPack().get_path("roombot_gui")+"/ui/img/b_eyes2.svg")
        self.pupil_path = rospy.get_param("~pupil_path",RosPack().get_path("roombot_gui")+"/ui/img/pupil2.svg")
        self.blink_duration = rospy.get_param("~blink_duration",0.15)
        self.blink_period = rospy.get_param("~blink_period",5.0)
        """Subscribers"""
        self.sub_detected_person = rospy.Subscriber(detected_person_topic, Float32, self.callback_detected_person, queue_size = 1)
        """Node Configuration"""
        self.MainWindow = QtWidgets.QMainWindow()
        self.setupUi()
        self.offset = 0.5
        self.target_offset = 0.5
        self.people_tracking = PeopleTracking()
        self.blinking = Blinking()
        self.people_tracking.update.connect(self.move_pupils)
        self.blinking.update.connect(self.blink)
        self.people_tracking.start()
        self.blinking.start()
        self.MainWindow.show()

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(size()[0],size()[1])
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.d = int(self.MainWindow.size().width()*0.5344)
        self.eye_size = (int(self.MainWindow.size().width()*0.46666),self.MainWindow.size().height())
        self.pupil_size = (int(self.MainWindow.size().width()*0.30),int(self.MainWindow.size().width()*0.30))
        initial_pose_r = (int((self.eye_size[0]/2)-(self.pupil_size[0]/2)),int((self.eye_size[1]/2)-(self.pupil_size[1]/2)))
        initial_pose_l = (int(size()[0]-(self.eye_size[0]/2)-(self.pupil_size[0]/2)),int((self.eye_size[1]/2)-(self.pupil_size[1]/2)))

        self.eyes = QtWidgets.QLabel(self.centralwidget)
        self.eyes.setGeometry(QtCore.QRect(0, 0, size()[0], size()[1]))
        self.eyes.setStyleSheet("border-image: url("+self.eyes_path+") 0 0 0 0 stretch stretch;")
        self.eyes.setText("")
        self.eyes.setObjectName("eyes")
        self.pupil_r = QtWidgets.QLabel(self.centralwidget)
        self.pupil_r.setGeometry(QtCore.QRect(initial_pose_r[0], initial_pose_r[1], self.pupil_size[0], self.pupil_size[1]))
        self.pupil_r.setStyleSheet("border-image: url("+self.pupil_path+") 0 0 0 0 stretch stretch;")
        self.pupil_r.setText("")
        self.pupil_r.setObjectName("pupil_r")
        self.pupil_l = QtWidgets.QLabel(self.centralwidget)
        self.pupil_l.setGeometry(QtCore.QRect(initial_pose_l[0],initial_pose_l[1], self.pupil_size[0], self.pupil_size[1]))
        self.pupil_l.setStyleSheet("border-image: url("+self.pupil_path+") 0 0 0 0 stretch stretch;")
        self.pupil_l.setText("")
        self.pupil_l.setObjectName("pupil_l")
        self.MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.MainWindow.setWindowTitle(_translate("MainWindow", "Eyes"))

    def callback_detected_person(self,msg):
        if msg.data >= 1:
            self.target_offset = 1
        elif msg.data <= 0.0:
            self.target_offset = 0
        else:
            self.target_offset = msg.data

    def move_pupils(self,x):
        y = 0.135*x + 0.02
        new_pos = [int(y*self.MainWindow.size().width()),int((self.eye_size[1]/2)-(self.pupil_size[1]/2))]
        self.pupil_r.setGeometry(QtCore.QRect(new_pos[0],new_pos[1],self.pupil_size[0],self.pupil_size[1]))
        self.pupil_l.setGeometry(QtCore.QRect(new_pos[0]+self.d,new_pos[1],self.pupil_size[0],self.pupil_size[1]))

    def blink(self,event):
        if event == 0:
            self.pupil_r.hide()
            self.pupil_l.hide()
            self.eyes.setStyleSheet("border-image: url("+ui.close_eyes_path+") 0 0 0 0 stretch stretch;")
        else:
            self.eyes.setStyleSheet("border-image: url("+ui.eyes_path+") 0 0 0 0 stretch stretch;")
            self.pupil_r.show()
            self.pupil_l.show()

class PeopleTracking(QtCore.QThread):
    update = QtCore.pyqtSignal(float)
    def __init__(self):
        super(PeopleTracking,self).__init__()

    def run(self):
        while  self.isRunning() and not rospy.is_shutdown():
            if(ui.target_offset != ui.offset):
                step = (ui.target_offset - ui.offset) / 5
                ui.offset += step
                self.update.emit(ui.offset)
            #sleep(50e-3)

class Blinking(QtCore.QThread):
    update = QtCore.pyqtSignal(int)
    def __init__(self):
        super(Blinking,self).__init__()

    def run(self):
        while  self.isRunning() and not rospy.is_shutdown():
            sleep(ui.blink_period)
            self.update.emit(0)
            sleep(ui.blink_duration)
            self.update.emit(1)

if __name__ == "__main__":
    rospy.init_node("eyes", anonymous = True)
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ui = UiEyes()
    sys.exit(app.exec_())
