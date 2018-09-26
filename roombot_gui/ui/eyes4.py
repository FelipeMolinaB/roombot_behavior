# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '/home/innovacion/roombot_ws/src/roombot_gui/ui/eyes3.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(676, 441)
        MainWindow.setStyleSheet("border-image: url(:/eyes/w_eyes.svg) 0 0 0 0 stretch stretch;\n"
"")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pupil_r = QtWidgets.QLabel(self.centralwidget)
        self.pupil_r.setGeometry(QtCore.QRect(0, 0, 131, 131))
        self.pupil_r.setStyleSheet("border-image: url(:/eyes/pupil.svg) 0 0 0 0 stretch stretch;")
        self.pupil_r.setText("")
        self.pupil_r.setObjectName("pupil_r")
        self.pupil_l = QtWidgets.QLabel(self.centralwidget)
        self.pupil_l.setGeometry(QtCore.QRect(0, 140, 131, 131))
        self.pupil_l.setStyleSheet("border-image: url(:/eyes/pupil.svg) 0 0 0 0 stretch stretch;")
        self.pupil_l.setText("")
        self.pupil_l.setObjectName("pupil_l")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))

import background_imgs_rc

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

