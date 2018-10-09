#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from pyautogui import size
from PyQt5 import QtCore, QtGui, QtWidgets
from time import sleep
from behavior.srv import ConfirmLoad,GuestConfirm
from behavior.srv import ConfirmLoadResponse,GuestConfirmResponse

class UiConfirmation(QtCore.QObject):
    request = QtCore.pyqtSignal(tuple)

    def __init__(self):
        super(UiConfirmation,self).__init__(parent = None)
        """Parameters Inicialization """
        confirm_load_service = rospy.get_param("~confirm_load_service", "confirm_load")
        guest_confirm_service = rospy.get_param("~guest_confirm_service", "guest_confirm")
        self.view_rows = rospy.get_param("~view_rows", 5.0)
        """Services"""
        self.confirm_load = rospy.Service(confirm_load_service,ConfirmLoad,self.callback_confirm_load)
        self.guest_confirm = rospy.Service(guest_confirm_service,GuestConfirm,self.callback_guest_confirm)
        """Node Configuration"""
        self.MainWindow = QtWidgets.QMainWindow()
        self.setupUi()
        self.request.connect(self.setup_table)
        self.confirmation = False
        self.source = 0
        rospy.loginfo("Confirmation Window OK")
        #self.MainWindow.show()

    def setupUi(self):
        self.MainWindow.setObjectName("MainWindow")
        self.MainWindow.resize(size()[0],size()[1])
        margin = (size()[0]*0.02,size()[1]*0.025)
        self.centralwidget = QtWidgets.QWidget(self.MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.request_info = QtWidgets.QTableWidget(self.centralwidget)
        self.request_info.setGeometry(QtCore.QRect(size()[0]*0.36, margin[1], size()[0]*0.62,size()[1]*0.925))
        self.request_info.setAlternatingRowColors(True)
        self.request_info.setObjectName("request_info")
        font = QtGui.QFont()
        font.setPointSize(size()[1]*0.027)
        self.request_info.setFont(font)
        self.table_container = (self.request_info.geometry().width(),self.request_info.geometry().height())
        self.request_info.itemClicked.connect(self.handleItemClicked)
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(margin[0], margin[1]*1.5, size()[0]*0.32, size()[1]*0.925))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setObjectName("verticalLayout")
        container = (self.verticalLayoutWidget.geometry().width(),self.verticalLayoutWidget.geometry().height()*0.45)
        self.groupBox = QtWidgets.QGroupBox(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(size()[1]*0.06)
        font.setItalic(False)
        self.groupBox.setFont(font)
        self.groupBox.setObjectName("groupBox")
        self.room = QtWidgets.QLabel(self.groupBox)
        self.room.setGeometry(QtCore.QRect(0,container[1]*0.33,container[0]*0.94,container[1]*0.34))
        self.room.setFrameShape(QtWidgets.QFrame.Box)
        self.room.setText("")
        self.room.setAlignment(QtCore.Qt.AlignCenter)
        self.room.setObjectName("room")
        self.verticalLayout.addWidget(self.groupBox)
        self.groupBox_2 = QtWidgets.QGroupBox(self.verticalLayoutWidget)
        self.groupBox_2.setFont(font)
        self.groupBox_2.setObjectName("groupBox_2")
        self.guest_name = QtWidgets.QLabel(self.groupBox_2)
        self.guest_name.setGeometry(QtCore.QRect(0,container[1]*0.33,container[0]*0.94,container[1]*0.34))
        self.guest_name.setFrameShape(QtWidgets.QFrame.Box)
        self.guest_name.setText("")
        self.guest_name.setAlignment(QtCore.Qt.AlignCenter)
        self.guest_name.setObjectName("guest_name")
        self.verticalLayout.addWidget(self.groupBox_2)
        self.confirm = QtWidgets.QPushButton(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(size()[1]*0.04)
        self.confirm.setFont(font)
        self.confirm.setEnabled(False)
        self.confirm.clicked.connect(self.confirmation_done)
        self.confirm.setObjectName("confirm")
        self.verticalLayout.addWidget(self.confirm)
        self.MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi()
        QtCore.QMetaObject.connectSlotsByName(self.MainWindow)

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.MainWindow.setWindowTitle(_translate("MainWindow", "Confirmation Window"))
        self.groupBox.setTitle(_translate("MainWindow", "Habitación:"))
        self.groupBox_2.setTitle(_translate("MainWindow", "Sr(a).:"))
        self.confirm.setText(_translate("MainWindow", "OK"))

    def handleItemClicked(self,item):
            if self.source == 0 and item.column() == 0:
                self.all_check[item.row()] = item.checkState()==QtCore.Qt.Checked
                if self.all_check == [True]*self.request_info.rowCount():
                    self.confirm.setEnabled(True)
                else:
                    self.confirm.setEnabled(False)
            else:
                self.confirm.setEnabled(True)

    def confirmation_done(self):
        self.confirmation = True

    def callback_confirm_load(self,msg):
        print("Load")
        self.source = 0
        self.request.emit((0,msg.request))
        while not self.confirmation:
            sleep(0.1)
        self.confirmation = False

        if msg.was_the_last:
            self.MainWindow.hide()
        else:
            self.confirm.setEnabled(False)
            self.request_info.clear()
        return ConfirmLoadResponse(True)

    def callback_guest_confirm(self,msg):
        print("Guest")
        self.source = 1
        self.request.emit((1,msg.request))
        while not self.confirmation:
            sleep(0.1)
        self.confirmation = False
        self.MainWindow.hide()
        self.request_info.clear()
        return GuestConfirmResponse(True)

    def setup_table(self,request):
        source = request[0]
        request = request[1]
        if source == 0:
            self.confirm.setEnabled(False)
            self.request_info.setColumnCount(3)
            self.request_info.setRowCount(len(request))
            self.request_info.setHorizontalHeaderLabels(['OK', 'Producto', 'Cantidad'])
            self.request_info.setColumnWidth(0,self.table_container[0]*0.07)
            self.request_info.setColumnWidth(1,self.table_container[0]*0.7)
            self.request_info.setColumnWidth(2,self.table_container[0]*0.20)
            for i in range(self.request_info.rowCount()):
                self.request_info.setRowHeight(i,self.table_container[0]*(1/self.view_rows))
                item =  QtWidgets.QTableWidgetItem("")
                item.setFlags(QtCore.Qt.ItemIsUserCheckable|QtCore.Qt.ItemIsEnabled)
                item.setCheckState(QtCore.Qt.Unchecked)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.request_info.setItem(i,0,item)
            self.all_check = [False]*self.request_info.rowCount()
            initial_col = 1
        else:
            self.confirm.setEnabled(True)
            self.request_info.setColumnCount(2)
            self.request_info.setRowCount(len(request))
            self.request_info.setHorizontalHeaderLabels(['Producto', 'Cantidad'])
            self.request_info.setColumnWidth(0,self.table_container[0]*0.77)
            self.request_info.setColumnWidth(1,self.table_container[0]*0.2)
            for i in range(self.request_info.rowCount()):
                self.request_info.setRowHeight(i,self.table_container[0]*(1/self.view_rows))
            initial_col = 0

        for i,product in enumerate(request):
            p,q = product.split(";")
            item =  QtWidgets.QTableWidgetItem(p)
            item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.request_info.setItem(i,initial_col,item)
            item =  QtWidgets.QTableWidgetItem(q)
            item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.request_info.setItem(i,initial_col+1,item)
        print("table setted up")
        self.MainWindow.show()

if __name__ == "__main__":
    rospy.init_node("confirmation_window", anonymous = True)
    import sys
    app = QtWidgets.QApplication([])
    ui = UiConfirmation()
    sys.exit(app.exec_())
