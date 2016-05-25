#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 22/05/16
@author: sampfeiffer

GUI to capture play motions for REEM
"""

import rospy
import rospkg
from sensor_msgs.msg import JointState
from play_motion_msgs.msg import PlayMotionActionGoal
from copy import deepcopy
import sys
import signal
from PyQt4 import QtGui, uic
from PyQt4.QtCore import QTimer


HEADER = '\033[95m'
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
WARNING = '\033[93m'
FAIL = '\033[91m'
ENDC = '\033[0m'


def get_joint_val(joint_name, joint_states):
    for j_name in joint_states.name:
        if j_name == joint_name:
            j_idx = joint_states.name.index(j_name)
            j_val = joint_states.position[j_idx]
            return j_val


def load_params_from_yaml(complete_file_path):
    from rosparam import upload_params
    from yaml import load
    f = open(complete_file_path, 'r')
    yamlfile = load(f)
    f.close()
    upload_params('/', yamlfile)


class CaptureGUI(QtGui.QMainWindow):
    def __init__(self):
        super(CaptureGUI, self).__init__()
        self.central_widget = QtGui.QStackedWidget()
        self.setCentralWidget(self.central_widget)
        self.myGUIwidget = myGUIwidget()
        self.central_widget.addWidget(self.myGUIwidget)
        self.show()


class myGUIwidget(QtGui.QWidget):

    joint_name_list = ['head_1_joint', 'head_2_joint', 'torso_1_joint', 'torso_2_joint',
                       'arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint',
                       'arm_right_4_joint', 'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint',
                       'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
                       'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint',
                       'hand_right_thumb_joint', 'hand_right_index_joint', 'hand_right_middle_joint',
                       'hand_left_thumb_joint', 'hand_left_index_joint', 'hand_left_middle_joint']

    def __init__(self, parent=None):
        super(myGUIwidget, self).__init__(parent)
        self.rospack = rospkg.RosPack()
        path = self.rospack.get_path('reem_movements_creator')
        print "path is: " + str(path)
        uic.loadUi(path + '/resources/capture_motion_reem.ui', self)

        self.update_gui_rate = 2.0  # Hz
        # set subscribers
        self.last_js = None
        self.joint_states_sub = rospy.Subscriber(
            '/joint_states', JointState, self.js_cb, queue_size=1)
        # the ones checked true
        self.current_interesting_joints = self.get_enabled_joints()
        self.play_motion_pub = rospy.Publisher(
            '/play_motion/goal', PlayMotionActionGoal, queue_size=1)

        self.update_timer = QTimer(self)
        self.update_timer.setInterval(1000.0 /
                                      self.update_gui_rate)
        self.update_timer.timeout.connect(self.update_current_joints_label)
        self.update_timer.start()

        # set callbacks to buttons (checkboxes are just monitored)
        self.capture_button.clicked.connect(self.capture_joints)
        self.upload_button.clicked.connect(self.upload_to_param_server)
        self.play_button.clicked.connect(self.play_motion)

    def update_current_joints_label(self):
        if self.last_js is None:
            return
        text_str = ""
        last_js = deepcopy(self.last_js)
        self.current_interesting_joints = self.get_enabled_joints()

        GREENCOLOR = "<font color='green'>"
        REDCOLOR = "<font color='red'>"
        CLOSECOLOR = "</font>"

        for enum_idx, j_name in enumerate(self.joint_name_list):
            # Set the color of the text
            if j_name in self.current_interesting_joints:
                text_str += GREENCOLOR
            else:
                text_str += REDCOLOR

            # Set the actual text
            print j_name
            print last_js
            print get_joint_val(j_name, last_js)
            text_str += j_name + ": " + \
                str(round(get_joint_val(j_name, last_js), 3)) + CLOSECOLOR

            if enum_idx == 0:  # head 1
                text_str += ", "
            elif enum_idx == 1:  # head 2
                text_str += "&nbsp;&nbsp;&nbsp;&nbsp;"
            elif enum_idx == 2:  # torso 1
                text_str += ", "
            elif enum_idx == 3:  # torso 2
                text_str += "<br>"
            elif enum_idx >= 4 and enum_idx < 10:  # arm right
                text_str += ", "
            elif enum_idx == 10:
                text_str += "<br>"
            elif enum_idx > 10 and enum_idx <= 16:  # arm left
                text_str += ", "
            elif enum_idx == 17:
                text_str += "<br>"
            elif enum_idx >= 18 and enum_idx < 20:  # right hand
                text_str += ", "
            elif enum_idx == 20:  # right hand spacer
                text_str += "&nbsp;&nbsp;&nbsp;&nbsp;"
            elif enum_idx > 20 and enum_idx < 23:
                text_str += ", "
            # elif enum_idx == 23:
            #     text_str += "<br>"

        self.current_joint_values_label.setText(text_str)

    def capture_joints(self):
        """
    open_hand:
      joints: [hand_thumb_joint, hand_index_joint, hand_mrl_joint]
      points:
      - positions: [-1.0, -1.0, -1.0]
        time_from_start: 0.0
      - positions: [0.0, 0.0, 0.0]
        time_from_start: 1.5
      meta:
        name: open_hand
        usage: demo
        description: 'open_hand'
        """

        motion_name = "change_me_motion_name"
        play_motion_str = "    " + motion_name + ":\n"
        play_motion_str += "      joints: " + \
            str(self.current_interesting_joints) + "\n"
        play_motion_str += "      points:\n"
        play_motion_str += "      - positions: "
        positions = []
        for j_name in self.current_interesting_joints:
            positions.append(round(get_joint_val(j_name, self.last_js), 3))
        play_motion_str += str(positions) + "\n"
        play_motion_str += "        time_from_start: 0.0\n"
        play_motion_str += """      meta:
        name: change_me_motion_name
        usage: demo
        description: 'change_me_motion_name'"""

        self.current_pose.setText(play_motion_str)
        # if nothing was written in the last field
        if self.full_motion.toPlainText() == "":
            extra_header = """play_motion:
  motions:\n"""
            self.full_motion.setText(extra_header + play_motion_str)

    def upload_to_param_server(self):
        if self.full_motion.toPlainText() != "":
            # To ease our life, create a tmp file to upload from it
            tmp_filename = '/tmp/capture_gui_tmp_yaml_file_to_upload_params.yaml'
            with open(tmp_filename, 'w') as f:
                f.write(self.full_motion.toPlainText())
            load_params_from_yaml(tmp_filename)
            rospy.loginfo("Loaded into param server the motion.")
        else:
            rospy.logerr(
                "Nothing in full motion field to upload to param server")
            return

    def play_motion(self):
        pmg = PlayMotionActionGoal()
        pmg.goal.motion_name = str(self.motion_name.toPlainText())
        pmg.goal.skip_planning = False
        rospy.loginfo("Sending goal: " + str(pmg.goal))
        self.play_motion_pub.publish(pmg)

    def js_cb(self, data):
        """
        :type data: JointState
        """
        self.last_js = data

    def get_enabled_joints(self):
        enabled_joints = []
        for j_name in self.joint_name_list:
            # this is like doing self.checkboxname.isChecked()
            if self.__dict__.get(j_name + "_c").isChecked():
                enabled_joints.append(j_name)
        rospy.logdebug("Found enabled joints: " + str(enabled_joints))
        return enabled_joints


if __name__ == '__main__':
    rospy.init_node('capture_gui')
    app = QtGui.QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    window = CaptureGUI()
    sys.exit(app.exec_())
