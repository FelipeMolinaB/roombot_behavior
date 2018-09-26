from PyQt5 import QtCore, QtGui, QtWidgets
from gtk.gdk import screen_width, screen_height

class UiEyes(object):
    def __init__(self):
        self.MainWindow = QtWidgets.QMainWindow()
        self.setupUi()

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(screen_width(), screen_height()-10)
        self.MainWindow.setStyleSheet("")
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(0, 0, self.MainWindow.size().width(), self.MainWindow.size().height()))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setStyleSheet("border-image: url(/home/roombot/ros_workspaces/behavior_ws/src/roombot_gui/ui/img/64945.svg) 0 0 0 0 stretch stretch;\n")
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


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    #MainWindow = QtWidgets.QMainWindow()
    ui = UiEyes()
    ui.show()
    #ui.setupUi(self.MainWindow)
    #self.MainWindow.show()

    sys.exit(app.exec_())
