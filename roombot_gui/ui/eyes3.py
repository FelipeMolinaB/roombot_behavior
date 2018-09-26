import rospy
from PyQt5 import QtCore, QtGui, QtWidgets
from gtk.gdk import screen_width, screen_height
from std_msgs.msg import Float32

class UiEyes(object):
    def __init__(self):
        rospy.init_node("eyes", anonymous = True)
        """Parameters Inicialization """
        detected_person_topic = rospy.get_param("~detected_person_topic","/detected_person_zone")
        self.image_width = rospy.get_param("~image_width",306)
        self.image_height = rospy.get_param("~image_height",226)
        """Subscribers"""
        self.sub_detected_person = rospy.Subscriber(detected_person_topic, Float32, self.callback_detected_person, queue_size = 1)
        """Node Configuration"""
        self.MainWindow = QtWidgets.QMainWindow()
        self.setupUi()
        self.move_pupils(1)
        """self.m = self.MainWindow.size().width()- self.image_width -45
        self.offset = self.pose_x_value
        self.target_offset = self.pose_x_value"""
        self.move = False
        self.move_eyes = MoveEyes()#self.pose_y_value)
        self.move_eyes.start()

    def callback_detected_person(self,msg):
        if msg.data != -1:
            self.target_offset = int(msg.data*self.m)
            self.move = True
            self.is_big = True
        else:
            self.is_big = False

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(screen_width(), screen_height()-10)
        self.MainWindow.setStyleSheet("border-image: url(/home/innovacion/roombot_ws/src/roombot_gui/ui/img/w_eyes.svg) 0 0 0 0 stretch stretch;")
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.d = int(self.MainWindow.size().width()*0.53333)
        self.eye_size = (int(self.MainWindow.size().width()*0.46666),self.MainWindow.size().height())
        self.pupil_size_l = (int(self.MainWindow.size().width()*0.20),int(self.MainWindow.size().width()*0.20))
        self.pupil_size_s = (int(self.MainWindow.size().width()*0.16666),int(self.MainWindow.size().width()*0.16666))
        self.is_big = True
        self.initial_pose = (int((self.eye_size[0]/2)-(self.pupil_size_s[0]/2)),int((self.eye_size[1]/2)-(self.pupil_size_s[1]/2)))


        self.pupil_r = QtWidgets.QLabel(self.centralwidget)
        self.pupil_r.setGeometry(QtCore.QRect(self.initial_pose[0], self.initial_pose[1], self.pupil_size_s[0], self.pupil_size_s[1]))
        self.pupil_r.setStyleSheet("border-image: url(/home/innovacion/roombot_ws/src/roombot_gui/ui/img/pupil.svg) 0 0 0 0 stretch stretch;")
        self.pupil_r.setText("")
        self.pupil_r.setObjectName("pupil_r")
        self.pupil_l = QtWidgets.QLabel(self.centralwidget)
        self.pupil_l.setGeometry(QtCore.QRect(self.initial_pose[0]+self.d, self.initial_pose[1], self.pupil_size_s[0], self.pupil_size_s[1]))
        self.pupil_l.setStyleSheet("border-image: url(/home/innovacion/roombot_ws/src/roombot_gui/ui/img/pupil.svg) 0 0 0 0 stretch stretch;")
        self.pupil_l.setText("")
        self.pupil_l.setObjectName("pupil_l")
        self.MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.MainWindow.setWindowTitle(_translate("MainWindow", "Eyes"))
        self.pupil_r.setText(_translate("MainWindow", "TextLabel"))
        #self.pupil_l.setText(_translate("MainWindow", "TextLabel"))

    def move_pupils(self,x):
        y = 0.21333*x + 0.02333
        new_pos = [int(y*self.MainWindow.size().width()),0]
        if self.is_big:
            size = self.pupil_size_l
        else:
            size = self.pupil_size_s
        new_pos[1] = int((self.eye_size[1]/2)-(size[1]/2))
        self.pupil_r.setGeometry(QtCore.QRect(new_pos[0],new_pos[1],size[0],size[1]))
        self.pupil_l.setGeometry(QtCore.QRect(new_pos[0]+self.d,new_pos[1],size[0],size[1]))


    def show(self):
        self.MainWindow.show()

    def close(self):
        self.MainWindow.close()

class MoveEyes(QtCore.QThread):
    def __init__(self):#,pose_y_value):
        super(MoveEyes,self).__init__()
        #self.pose_y_value = pose_y_value

    def run(self):
        sleep = True
        x = 0.5
        step = 0.05
        while  self.isRunning() and not rospy.is_shutdown():
            """if ui.move:
                if(ui.target_offset > ui.offset):
                    step = 1
                else:
                    step = -1
                    # move right
                for i in range(ui.offset, ui.target_offset,step):
                    ui.label.setGeometry(QtCore.QRect(i, ui.pose_y_value, ui.image_width,ui.image_height))
                    ui.offset = i
                    rospy.sleep(10e-3)
                    sleep = False
                ui.move = False
            else:
                sleep = True
            if sleep:
                rospy.sleep(50e-3)"""
            ui.move_pupils(x)
            x += step
            print(x,x == 1.0,x == 0.0)
            if round(x,1) == 1 or round(x,1) == 0.0:
                step *= -1
            rospy.sleep(0.1)

if __name__ == "__main__":
    try:
        import sys
        app = QtWidgets.QApplication(sys.argv)
        ui = UiEyes()
        ui.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        print("run")
        ui.move_eyes.terminate()
        ui.close()
