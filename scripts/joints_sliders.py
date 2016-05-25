#!/usr/bin/env python
"""
Sliders for every robot joint
Author: Sammy Pfeiffer
"""

from PyQt4 import QtGui, QtCore
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import xml.dom.minidom


# Based on the implementation of joint_state_publisher
def get_joint_limits():
    """Gets the joint limits from the uploaded URDF
    /robot_description parameter"""
    # limits = {'arm_right_1_joint_min': -1.0,
    #           'arm_right_1_joint_max': 1.0}
    limits = {}
    description = rospy.get_param('robot_description')
    robot = xml.dom.minidom.parseString(
        description).getElementsByTagName('robot')[0]
    # Find all non-fixed joints
    for child in robot.childNodes:
        if child.nodeType is child.TEXT_NODE:
            continue
        if child.localName == 'joint':
            jtype = child.getAttribute('type')
            if jtype == 'fixed' or jtype == 'floating':
                continue
            name = child.getAttribute('name')
            try:
                limit = child.getElementsByTagName('limit')[0]
                minval = float(limit.getAttribute('lower'))
                maxval = float(limit.getAttribute('upper'))
            except:
                rospy.logdebug(
                    "%s is not fixed, nor continuous, but limits are not specified!" % name)
                continue

            safety_tags = child.getElementsByTagName('safety_controller')
            if len(safety_tags) == 1:
                tag = safety_tags[0]
                if tag.hasAttribute('soft_lower_limit'):
                    minval = max(
                        minval, float(tag.getAttribute('soft_lower_limit')))
                if tag.hasAttribute('soft_upper_limit'):
                    maxval = min(
                        maxval, float(tag.getAttribute('soft_upper_limit')))
            limits[name + "_min"] = minval
            limits[name + "_max"] = maxval
    return limits


class Main(QtGui.QMainWindow):

    def __init__(self, parent=None):
        super(Main, self).__init__(parent)

        self.last_js = None
        self.js_sub = rospy.Subscriber('/joint_states',
                                       JointState,
                                       self.js_cb,
                                       queue_size=1)
        rospy.loginfo("Subscribed to: " + str(self.js_sub.resolved_name))

        self.arm_right_pub = rospy.Publisher('/arm_right_controller/command',
                                             JointTrajectory,
                                             queue_size=1)
        self.arm_left_pub = rospy.Publisher('/arm_left_controller/command',
                                            JointTrajectory,
                                            queue_size=1)
        self.head_pub = rospy.Publisher('/head_controller/command',
                                        JointTrajectory,
                                        queue_size=1)

        self.torso_pub = rospy.Publisher('/torso_controller/command',
                                         JointTrajectory,
                                         queue_size=1)

        self.hand_right_pub = rospy.Publisher('/hand_right_controller/command',
                                              JointTrajectory,
                                              queue_size=1)

        self.hand_left_pub = rospy.Publisher('/hand_left_controller/command',
                                             JointTrajectory,
                                             queue_size=1)

        self.joint_names = ['head_1_joint',
                            'head_2_joint',
                            'arm_right_1_joint',  # 2
                            'arm_right_2_joint',
                            'arm_right_3_joint',
                            'arm_right_4_joint',
                            'arm_right_5_joint',
                            'arm_right_6_joint',
                            'arm_right_7_joint',
                            'arm_left_1_joint',  # 9
                            'arm_left_2_joint',
                            'arm_left_3_joint',
                            'arm_left_4_joint',
                            'arm_left_5_joint',
                            'arm_left_6_joint',
                            'arm_left_7_joint',
                            'hand_right_thumb_joint',  # 16
                            'hand_right_index_joint',
                            'hand_right_middle_joint',
                            'hand_left_thumb_joint',  # 19
                            'hand_left_index_joint',
                            'hand_left_middle_joint',
                            'torso_1_joint',  # 22
                            'torso_2_joint']

        self.joint_limits = get_joint_limits()

        # Create every slider row...
        self.sliders = []
        # for every joint name...
        for joint in self.joint_names:
            min_l = self.get_joint_limit_min(joint)
            max_l = self.get_joint_limit_max(joint)
            self.sliders.append(
                self.create_slider(joint, min_l, max_l))

        hspacer = QtGui.QSplitter()
        vspacer = QtGui.QSplitter(1)

        self.layout = QtGui.QVBoxLayout()
        self.head_layout = QtGui.QVBoxLayout()
        self.head_layout.addLayout(self.sliders[0])
        self.head_layout.addLayout(self.sliders[1])

        self.arms_layout = QtGui.QHBoxLayout()
        self.left_layout = QtGui.QVBoxLayout()
        self.right_layout = QtGui.QVBoxLayout()
        self.arms_layout.addLayout(self.left_layout)
        self.arms_layout.addWidget(vspacer)
        self.arms_layout.addLayout(self.right_layout)
        # arm left sliders
        for i in range(9, 16):
            self.left_layout.addLayout(self.sliders[i])
        self.left_layout.addWidget(hspacer)
        # hand left
        for i in range(19, 22):
            self.left_layout.addLayout(self.sliders[i])
        # arm right sliders
        for i in range(2, 9):
            self.right_layout.addLayout(self.sliders[i])
        self.right_layout.addWidget(hspacer)
        # hand right
        for i in range(16, 19):
            self.right_layout.addLayout(self.sliders[i])

        self.torso_layout = QtGui.QVBoxLayout()
        self.torso_layout.addLayout(self.sliders[-2])
        self.torso_layout.addLayout(self.sliders[-1])

        self.layout.addLayout(self.head_layout)
        self.layout.addWidget(vspacer)
        self.layout.addLayout(self.arms_layout)
        self.layout.addWidget(vspacer)
        self.layout.addLayout(self.torso_layout)

        self.central_widget = QtGui.QWidget()
        self.central_widget.setLayout(self.layout)

        self.setCentralWidget(self.central_widget)

        self.timer = QtCore.QTimer()
        self.timer.setInterval(100)  # ms
        self.timer.timeout.connect(self.update_gui)
        self.timer.start()

    def js_cb(self, data):
        self.last_js = data

    def get_joint_position(self, joint_name):
        if self.last_js is None:
            return

        idx = self.last_js.name.index(joint_name)
        if idx == -1:
            rospy.logerr(
                "joint: " + joint_name + " is not in last joint states")
        return self.last_js.position[idx]

    def update_gui(self, *args):
        if self.last_js is None:
            return
        for joint in self.joint_names:
            # Get the joint position
            position = self.get_joint_position(joint)
            # Get the slider, and update the pose
            slider = self.__getattribute__(joint + "_slider")
            # Block signals to not send goals on joint states updates
            slider.blockSignals(True)
            slider.setValue(int(position * 100))
            slider.blockSignals(False)
            # Get the spinbox, and update the pose
            # TODO: check how to check if the field is being edited to not
            # modify it
            spinbox = self.__getattribute__(joint + "_spinbox")
            spinbox.blockSignals(True)
            spinbox.setValue(position)
            spinbox.blockSignals(False)

    def get_joint_limit_min(self, joint_name):
        return self.joint_limits[joint_name + '_min']

    def get_joint_limit_max(self, joint_name):
        return self.joint_limits[joint_name + '_max']

    def get_pub_for(self, joint_name):
        group = joint_name.replace('_joint', '')
        # To remove _1 from head[_1]_joint
        if group[-1] in ['1', '2', '3', '4', '5', '6', '7']:
            group = group[:-2]
        return self.__getattribute__(group + '_pub')

    def create_slider(self, joint_name, min_limit, max_limit):
        row = QtGui.QHBoxLayout()
        slider = QtGui.QSlider(1)
        # TODO: add dots to the slider
        slider.setRange(int(min_limit * 100), int(max_limit * 100))
        # A bit of magic to create dynamically a method to be called
        method_name = self.add_cb_to_class_by_joint_name(joint_name, 'slider')
        slider.valueChanged.connect(self.__getattribute__(method_name))
        # To easy access the slider later on
        self.__setattr__(joint_name + "_slider", slider)
        spinbox = QtGui.QDoubleSpinBox()
        spinbox.setDecimals(2)
        spinbox.setRange(min_limit, max_limit)
        method_name = self.add_cb_to_class_by_joint_name(joint_name, 'spinbox')
        spinbox.valueChanged.connect(self.__getattribute__(method_name))
        self.__setattr__(joint_name + "_spinbox", spinbox)
        text = QtGui.QLabel(joint_name)

        row.addWidget(text)
        row.addWidget(slider)
        row.addWidget(spinbox)
        return row

    def create_goal_for(self, joint_name, new_value):
        print "Creating a goal for... " + str(joint_name)
        jt = JointTrajectory()
        if "head" in joint_name:
            jt.joint_names = ['head_1_joint', 'head_2_joint']
        elif "torso" in joint_name:
            jt.joint_names = ['torso_1_joint', 'torso_2_joint']
        elif "arm" in joint_name:
            if "right" in joint_name:
                jt.joint_names = ['arm_right_1_joint',
                                  'arm_right_2_joint',
                                  'arm_right_3_joint',
                                  'arm_right_4_joint',
                                  'arm_right_5_joint',
                                  'arm_right_6_joint',
                                  'arm_right_7_joint']
            elif "left" in joint_name:
                jt.joint_names = ['arm_left_1_joint',
                                  'arm_left_2_joint',
                                  'arm_left_3_joint',
                                  'arm_left_4_joint',
                                  'arm_left_5_joint',
                                  'arm_left_6_joint',
                                  'arm_left_7_joint']
        elif "hand" in joint_name:
            if "right" in joint_name:
                jt.joint_names = ['hand_right_thumb_joint',
                                  'hand_right_index_joint',
                                  'hand_right_middle_joint']
            elif "left" in joint_name:
                jt.joint_names = ['hand_left_thumb_joint',
                                  'hand_left_index_joint',
                                  'hand_left_middle_joint']

        jtp = JointTrajectoryPoint()
        for j_name in jt.joint_names:
            if j_name == joint_name:
                jtp.positions.append(new_value)
            else:
                jtp.positions.append(self.get_joint_position(j_name))

        # TODO: maybe tune time for joint groups too, fingers may be too slow
        # Goals will take 1.0 seconds + a part relative on how much it should
        # move
        time_for_goal = 1.0 + \
            abs(self.get_joint_position(joint_name) - new_value) * 3.0
        jtp.time_from_start = rospy.Duration(time_for_goal)

        jt.points.append(jtp)

        return jt

    def send_goal(self, joint_name, new_value):
        if self.last_js is None:
            return
        goal = self.create_goal_for(joint_name, new_value)
        pub = self.get_pub_for(joint_name)
        pub.publish(goal)

    def add_cb_to_class_by_joint_name(self, joint_name, type_cb):
        # Create dynamically a method to be called
        def make_method(joint_name):
            def _method():
                # print "Cb for joint '" + parameter_name + "' called."
                # print "We got value: " + str(new_value)
                if type_cb == 'slider':
                    new_value = self.__getattribute__(
                        joint_name + "_slider").value()
                    new_value = float(new_value / 100.0)
                elif type_cb == 'spinbox':
                    new_value = self.__getattribute__(
                        joint_name + "_spinbox").value()

                # TODO: update slider and spinbox with new value
                self.send_goal(joint_name, new_value)

            return _method

        method_name = "callback_method_for_" + joint_name + "_" + type_cb
        cb_method = make_method(joint_name)
        setattr(self, method_name, cb_method)
        return method_name

if __name__ == '__main__':
    rospy.init_node('joint_sliders')
    app = QtGui.QApplication(["CHIP Joint sliders"])
    myWidget = Main()
    myWidget.show()
    app.exec_()
