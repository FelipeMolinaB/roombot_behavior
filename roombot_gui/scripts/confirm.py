from PyQt5 import QtCore, QtGui, QtWidgets
import signal

class UiComfirm(object):
    def __init__(self):
        self.MainWindow = QtWidgets.QMainWindow()
        self.setupUi()

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(519, 237)
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(0, 0, 521, 191))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.pushButton = QtWidgets.QPushButton(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy)
        self.pushButton.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout.addWidget(self.pushButton)
        self.MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "Pedido".center(100," ")))
        self.pushButton.setText(_translate("MainWindow", "OK"))

    def show(self):
        self.MainWindow.show()

    def close(self):
        self.MainWindow.close()

    def set_button_callback(self,func=exit):
        self.pushButton.clicked.connect(func)


class Wait2Confirm(QtCore.QThread):
    signal = QtCore.pyqtSignal(float)
    def __init__(self):
        super(MoveEyes,self).__init__()
        self.signal.connect(ui.update)

    def run(self):
        while  self.isRunning():

            if RobotGUI.people_detected and RobotGUI.enable_move_eyes:
                sleep(0.1)

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ui = UiComfirm()
    ui.set_button_callback()
    ui.show()
    #signal.signal(signal.SIGKILL,)
    sys.exit(app.exec_())
