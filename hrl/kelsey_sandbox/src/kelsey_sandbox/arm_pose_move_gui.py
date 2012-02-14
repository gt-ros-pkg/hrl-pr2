# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'arm_pose_move_gui.ui'
#
# Created: Tue Feb 14 08:25:12 2012
#      by: PyQt4 UI code generator 4.7.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_Frame(object):
    def setupUi(self, Frame):
        Frame.setObjectName("Frame")
        Frame.resize(566, 439)
        Frame.setFrameShape(QtGui.QFrame.StyledPanel)
        Frame.setFrameShadow(QtGui.QFrame.Raised)
        self.line = QtGui.QFrame(Frame)
        self.line.setGeometry(QtCore.QRect(10, 360, 451, 16))
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName("line")
        self.joint_combo = QtGui.QComboBox(Frame)
        self.joint_combo.setGeometry(QtCore.QRect(10, 390, 451, 31))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint_combo.sizePolicy().hasHeightForWidth())
        self.joint_combo.setSizePolicy(sizePolicy)
        self.joint_combo.setObjectName("joint_combo")
        self.widget = QtGui.QWidget(Frame)
        self.widget.setGeometry(QtCore.QRect(40, 320, 221, 41))
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtGui.QHBoxLayout(self.widget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.forward_button = QtGui.QRadioButton(self.widget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.forward_button.sizePolicy().hasHeightForWidth())
        self.forward_button.setSizePolicy(sizePolicy)
        self.forward_button.setChecked(True)
        self.forward_button.setObjectName("forward_button")
        self.horizontalLayout.addWidget(self.forward_button)
        self.reverse_button = QtGui.QRadioButton(self.widget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.reverse_button.sizePolicy().hasHeightForWidth())
        self.reverse_button.setSizePolicy(sizePolicy)
        self.reverse_button.setObjectName("reverse_button")
        self.horizontalLayout.addWidget(self.reverse_button)
        self.label = QtGui.QLabel(Frame)
        self.label.setGeometry(QtCore.QRect(20, 210, 91, 61))
        self.label.setStyleSheet("font: 14pt;")
        self.label.setObjectName("label")
        self.label_2 = QtGui.QLabel(Frame)
        self.label_2.setGeometry(QtCore.QRect(150, 220, 391, 41))
        self.label_2.setStyleSheet("font: 14pt ;")
        self.label_2.setObjectName("label_2")
        self.groupBox = QtGui.QGroupBox(Frame)
        self.groupBox.setGeometry(QtCore.QRect(450, 210, 120, 221))
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.traj_button = QtGui.QRadioButton(self.groupBox)
        self.traj_button.setGeometry(QtCore.QRect(20, 70, 93, 22))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.traj_button.sizePolicy().hasHeightForWidth())
        self.traj_button.setSizePolicy(sizePolicy)
        self.traj_button.setChecked(False)
        self.traj_button.setObjectName("traj_button")
        self.joint_pose_button = QtGui.QRadioButton(self.groupBox)
        self.joint_pose_button.setGeometry(QtCore.QRect(20, 180, 93, 22))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joint_pose_button.sizePolicy().hasHeightForWidth())
        self.joint_pose_button.setSizePolicy(sizePolicy)
        self.joint_pose_button.setChecked(True)
        self.joint_pose_button.setObjectName("joint_pose_button")
        self.traj_combo = QtGui.QComboBox(Frame)
        self.traj_combo.setGeometry(QtCore.QRect(10, 280, 451, 31))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.traj_combo.sizePolicy().hasHeightForWidth())
        self.traj_combo.setSizePolicy(sizePolicy)
        self.traj_combo.setObjectName("traj_combo")
        self.setup_box = QtGui.QCheckBox(Frame)
        self.setup_box.setGeometry(QtCore.QRect(290, 330, 181, 21))
        self.setup_box.setObjectName("setup_box")
        self.widget1 = QtGui.QWidget(Frame)
        self.widget1.setGeometry(QtCore.QRect(20, 10, 521, 191))
        self.widget1.setObjectName("widget1")
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.widget1)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.start_button = QtGui.QPushButton(self.widget1)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.start_button.sizePolicy().hasHeightForWidth())
        self.start_button.setSizePolicy(sizePolicy)
        self.start_button.setStyleSheet("font: 16pt;\n"
"background-color: rgb(120, 255, 96);")
        self.start_button.setObjectName("start_button")
        self.horizontalLayout_2.addWidget(self.start_button)
        self.stop_button = QtGui.QPushButton(self.widget1)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stop_button.sizePolicy().hasHeightForWidth())
        self.stop_button.setSizePolicy(sizePolicy)
        self.stop_button.setStyleSheet("background-color: rgb(255, 67, 67);\n"
"font: 16pt;")
        self.stop_button.setObjectName("stop_button")
        self.horizontalLayout_2.addWidget(self.stop_button)
        self.reset_button = QtGui.QPushButton(self.widget1)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.reset_button.sizePolicy().hasHeightForWidth())
        self.reset_button.setSizePolicy(sizePolicy)
        self.reset_button.setStyleSheet("background-color: rgb(70, 144, 255);\n"
"font: 16pt;")
        self.reset_button.setObjectName("reset_button")
        self.horizontalLayout_2.addWidget(self.reset_button)

        self.retranslateUi(Frame)
        QtCore.QMetaObject.connectSlotsByName(Frame)

    def retranslateUi(self, Frame):
        Frame.setWindowTitle(QtGui.QApplication.translate("Frame", "Frame", None, QtGui.QApplication.UnicodeUTF8))
        self.forward_button.setText(QtGui.QApplication.translate("Frame", "Forward", None, QtGui.QApplication.UnicodeUTF8))
        self.reverse_button.setText(QtGui.QApplication.translate("Frame", "Reverse", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Frame", "Currently\n"
"Running:", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("Frame", "None", None, QtGui.QApplication.UnicodeUTF8))
        self.traj_button.setText(QtGui.QApplication.translate("Frame", "Trajectory", None, QtGui.QApplication.UnicodeUTF8))
        self.joint_pose_button.setText(QtGui.QApplication.translate("Frame", "Joint Pose", None, QtGui.QApplication.UnicodeUTF8))
        self.setup_box.setText(QtGui.QApplication.translate("Frame", "Move to Setup Only", None, QtGui.QApplication.UnicodeUTF8))
        self.start_button.setText(QtGui.QApplication.translate("Frame", "Start/Restart", None, QtGui.QApplication.UnicodeUTF8))
        self.stop_button.setText(QtGui.QApplication.translate("Frame", "Pause", None, QtGui.QApplication.UnicodeUTF8))
        self.reset_button.setText(QtGui.QApplication.translate("Frame", "Reset", None, QtGui.QApplication.UnicodeUTF8))

