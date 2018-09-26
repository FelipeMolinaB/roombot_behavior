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
        self.m = self.MainWindow.size().width()- self.image_width -45
        self.offset = self.pose_x_value
        self.target_offset = self.pose_x_value
        self.move = False
        self.move_eyes = MoveEyes(self.pose_y_value)
        self.move_eyes.start()

    def callback_detected_person(self,msg):
        self.target_offset = int(msg.data*self.m)
        self.move = True

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(screen_width(), screen_height()-10)
        self.MainWindow.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.pose_y_value = self.MainWindow.size().height()/2 - self.image_height/2
        self.pose_x_value = self.MainWindow.size().width()/2 - self.image_width/2
        self.label.setGeometry(QtCore.QRect(self.pose_x_value, self.pose_y_value, self.image_width,self.image_height))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setStyleSheet("border-image: url(/home/roombot/ros_workspaces/behavior_ws/src/roombot_gui/ui/img/eyes.svg) 0 0 0 0 stretch stretch;\n")
        self.label.setFrameShape(QtWidgets.QFrame.Box)
        self.label.setText("")
        self.label.setObjectName("label")
        self.MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.MainWindow.setWindowTitle(_translate("MainWindow", "Eyes"))

    def show(self):
        self.MainWindow.show()

    def close(self):
        self.MainWindow.close()

class MoveEyes(QtCore.QThread):
    def __init__(self,pose_y_value):
        super(MoveEyes,self).__init__()
        self.pose_y_value = pose_y_value

    def run(self):
        sleep = True
        while  self.isRunning() and not rospy.is_shutdown():
            if ui.move:
                if(ui.target_offset != ui.offset):
                    step = int((ui.target_offset - ui.offset) / 5)
                else:
                    step = 0
                ui.offset += step
                ui.label.setGeometry(QtCore.QRect(ui.offset, ui.pose_y_value, ui.image_width,ui.image_height))

            else:
                sleep = True
            if sleep:
                rospy.sleep(50e-3)

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
