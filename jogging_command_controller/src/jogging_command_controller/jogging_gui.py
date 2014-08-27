import os
import numpy as np
from functools import partial


import rospy
import rosservice
import rosparam

from controller_manager_msgs.srv import *

from qt_gui.plugin import Plugin

import python_qt_binding.QtCore as QtCore
import python_qt_binding.QtGui as QtGui

from rqt_py_common.plugin_container_widget import PluginContainerWidget
from rqt_robot_dashboard.util import IconHelper

from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState

from ur_joint_ctrl_gui.joint_vel_ctrl import JointVelocityController

from urdf_parser_py.urdf import Robot

JOGGING_SUFFIX = 'JoggingCommandController'

JOINT_NAMES = ['Shoulder Pan', 'Shoulder Lift', 'Elbow',
               'Wrist 1', 'Wrist 2', 'Wrist 3']
JOINT_SHORT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
ICON_NAMES = [['rotate_y_pos_on.png'], ['rotate_y_neg_on.png'],
              ['rotate_z_neg_on.png'], ['rotate_z_pos_on.png']]
BUTTON_ICONS = [(2,3), (0,1), (0,1), (0,1), (2,3), (0,1)]
VEL_MULTS = [-1.0, 1.0] 
MONITOR_RATE = 1000./100.
METERS_PER_FOOT = 0.3048

class JoggingControlGui(Plugin):

    def __init__(self, context):
        super(JoggingControlGui, self).__init__(context)
        self.setObjectName('jogging_control_gui')

        self.widget = JoggingControlWidget(self)

        self.main_widget = PluginContainerWidget(self.widget, True, False)

        self.widget.start()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        self.widget.shutdown_plugin()

class JoggingControlWidget(QtGui.QWidget):

    # signals
    state_changed = QtCore.Signal()
    start_moving = QtCore.Signal(int, float)
    stop_moving = QtCore.Signal(int)
    sig_sysmsg = QtCore.Signal(str)

    def __init__(self, plugin):
        super(JoggingControlWidget, self).__init__()

        self._plugin = plugin
        self.setWindowTitle('Jogging Control GUI')

        self.clear_internal_state()

        # # setup IconHelper
        # import rospkg
        # rp = rospkg.RosPack()
        # paths = [os.path.join(rp.get_path('jogging_command_controller'), 'images')]
        # self.icon_helper = IconHelper(paths, self.name)
        # converted_icons = self.icon_helper.set_icon_lists(ICON_NAMES, None, False)
        # self._icons = converted_icons[0]
        # self._clicked_icons = converted_icons[1]

        ########################################################################
        # setup gui
        vlayout_outer = QtGui.QVBoxLayout(self)
        vlayout_outer.setObjectName('vert_layout_outer')
        hlayout_top = QtGui.QHBoxLayout(self)
        hlayout_top.setObjectName('hori_layout_top')
        vlayout_outer.addLayout(hlayout_top)
        vlayout_joints = QtGui.QVBoxLayout(self)
        vlayout_joints.setObjectName('vert_layout_joints')
        vlayout_outer.addLayout(vlayout_joints)
        self.vlayout_joints = vlayout_joints

        # create top bar
        # controller manager namespace combo box & label
        cm_ns_label = QtGui.QLabel(self)
        cm_ns_label.setObjectName('cm_ns_label')
        cm_ns_label.setText('CM Namespace:')
        fixed_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed)
        cm_ns_label.setSizePolicy(fixed_policy)
        hlayout_top.addWidget(cm_ns_label)
        cm_namespace_combo = QtGui.QComboBox(self)
        cm_namespace_combo.setObjectName('cm_namespace_combo')
        hlayout_top.addWidget(cm_namespace_combo)
        self.cm_namespace_combo = cm_namespace_combo

        # load controller combo box & label
        load_ctrl_label = QtGui.QLabel(self)
        load_ctrl_label.setObjectName('load_ctrl_label')
        load_ctrl_label.setText('Jogging Controller:')
        fixed_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed)
        load_ctrl_label.setSizePolicy(fixed_policy)
        hlayout_top.addWidget(load_ctrl_label)
        load_ctrl_combo = QtGui.QComboBox(self)
        load_ctrl_combo.setObjectName('load_ctrl_combo')
        load_ctrl_size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        load_ctrl_combo.setSizePolicy(load_ctrl_size_policy)
        hlayout_top.addWidget(load_ctrl_combo)
        self.load_ctrl_combo = load_ctrl_combo

        # load control button
        load_ctrl_button = QtGui.QPushButton(self)
        load_ctrl_button.setObjectName('load_ctrl_button')
        button_size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        button_size_policy.setHorizontalStretch(0)
        button_size_policy.setVerticalStretch(0)
        button_size_policy.setHeightForWidth(load_ctrl_button.sizePolicy().hasHeightForWidth())
        load_ctrl_button.setSizePolicy(button_size_policy)
        load_ctrl_button.setBaseSize(QtCore.QSize(30, 30))
        load_ctrl_button.setIcon(QtGui.QIcon.fromTheme('add'))
        load_ctrl_button.setIconSize(QtCore.QSize(20,20))
        load_ctrl_button.clicked.connect(self.load_jog_ctrl)
        hlayout_top.addWidget(load_ctrl_button)

        # # unload control button
        # unload_ctrl_button = QtGui.QPushButton(self)
        # unload_ctrl_button.setObjectName('unload_ctrl_button')
        # button_size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        # button_size_policy.setHorizontalStretch(0)
        # button_size_policy.setVerticalStretch(0)
        # button_size_policy.setHeightForWidth(unload_ctrl_button.sizePolicy().hasHeightForWidth())
        # unload_ctrl_button.setSizePolicy(button_size_policy)
        # unload_ctrl_button.setBaseSize(QtCore.QSize(30, 30))
        # unload_ctrl_button.setIcon(QtGui.QIcon.fromTheme('cancel'))
        # unload_ctrl_button.setIconSize(QtCore.QSize(20,20))
        # unload_ctrl_button.clicked.connect(self.unload_jog_ctrl)
        # hlayout_top.addWidget(unload_ctrl_button)

        ########################################################################

        # controller manager services
        self.ctrlman_ns_cur = '/'
        self.loadable_params = {}
        self.ctrlman_list_types = {}

        # self.joint_vel_ctrl = JointVelocityController()

        # setup update timers
        self._timer_update_commands = QtCore.QTimer(self)
        self._timer_update_commands.timeout.connect(self._update_commmands)
        self._timer_refresh_ctrlers = QtCore.QTimer(self)
        self._timer_refresh_ctrlers.timeout.connect(self._refresh_ctrlers_cb)

        # create ros subscribers
        # self._diag_agg_sub = rospy.Subscriber('diagnostics_agg', DiagnosticArray, 
        #                                       self._diagnostics_cb)
        # self._joint_states_sub = rospy.Subscriber('joint_states', JointState, 
        #                                           self._joint_states_cb)

    def clear_internal_state(self):
        # interactive components
        self.buttons      = []
        self.speed_boxes  = []
        # self.joint_modes  = []
        self.q_left_nums  = []
        self.q_right_nums = []
        self.q_sliders    = []

        # internal state
        # self._joint_mode_text = None
        self._q_pos           = None
        self._q_order         = None
        self._joint_names     = []
        self._velocity_limits = None
        self._joint_types     = None
        self._limits_lower    = None
        self._limits_upper    = None
        self._jnt_cmd_pubs    = None
        self._pub_timers      = None
        self._joint_state_sub = None

    @QtCore.Slot()
    def _update_commmands(self):
        pass

    def _refresh_ctrlers_cb(self):
        try:
            # refresh the list of controller managers we can find
            srv_list = rosservice.get_service_list()
            ctrlman_ns_list_cur = []
            for srv_name in srv_list:
                if 'controller_manager/list_controllers' in srv_name:
                    srv_type = rosservice.get_service_type(srv_name)
                    if srv_type == 'controller_manager_msgs/ListControllers':
                        ctrlman_ns = srv_name.split('/controller_manager/list_controllers')[0]
                        if ctrlman_ns == '':
                            ctrlman_ns = '/'
                        # ctrlman_ns is a Controller Manager namespace
                        if ctrlman_ns not in self.ctrlman_list_types:
                            # we haven't connected to it yet, create the service proxies
                            self.controller_manager_connect(ctrlman_ns)
                        ctrlman_ns_list_cur.append(ctrlman_ns)

            # remove every controller manager which isn't up anymore
            for ctrlman_ns_old in self.ctrlman_list_types.keys():
                if ctrlman_ns_old not in ctrlman_ns_list_cur:
                    self.controller_manager_disconnect(ctrlman_ns_old)

            # refresh the controller list for the current controller manager
            self.refresh_loadable_ctrlers()
        except Exception as e:
            self.sig_sysmsg.emit(e.message)

    def refresh_loadable_ctrlers(self):
        if self.cm_namespace_combo.count() == 0:
            # no controller managers found so there are no loadable controllers
            # remove old loadables
            for old_loadable_text in self.loadable_params.keys():
                self.remove_loadable_from_list(old_loadable_text)
            return

        ctrlman_ns = self.cm_namespace_combo.currentText()

        if self.ctrlman_ns_cur != ctrlman_ns:
            # new controller manager selected
            # remove old loadables from list from last CM
            for old_loadable_text in self.loadable_params.keys():
                self.remove_loadable_from_list(old_loadable_text)
            self.ctrlman_ns_cur = ctrlman_ns

        rospy.wait_for_service(ctrlman_ns + '/controller_manager/list_controller_types', 0.2)
        try:
            resp = self.ctrlman_list_types[ctrlman_ns].call(ListControllerTypesRequest())
        except rospy.ServiceException as e:
            # TODO: display warning somehow
            return 
        ctrler_types = resp.types
        loadable_params_cur = []
        all_params = rosparam.list_params('/')
        # for every parameter
        for pname in all_params:
            # remove the controller manager namespace
            if ctrlman_ns == '/':
                pname_sub = pname
            else:
                pname_sub = pname[len(ctrlman_ns):]
            psplit = pname_sub.split('/')
            if len(psplit) > 2 and psplit[2] == 'type':
                loadable_type = rosparam.get_param(pname)
                if loadable_type in ctrler_types and JOGGING_SUFFIX in loadable_type:
                    load_text = pname[:-5] + '  -  ' + loadable_type
                    loadable_params_cur.append(load_text)
                    if load_text not in self.loadable_params:
                        self.loadable_params[load_text] = psplit[1]
                        self.load_ctrl_combo.addItem(load_text)

        # remove loadable parameters no longer in the parameter server
        for load_text_old in self.loadable_params.keys():
            if load_text_old not in loadable_params_cur:
                self.remove_loadable_from_list(load_text_old)

    def start(self):
        self._timer_update_commands.start(1000)
        self._timer_refresh_ctrlers.start(1000)

    def controller_manager_connect(self, ctrlman_ns):
        self.ctrlman_list_types[ctrlman_ns] = rospy.ServiceProxy(
                ctrlman_ns + '/controller_manager/list_controller_types', ListControllerTypes)
        self.cm_namespace_combo.addItem(ctrlman_ns)

    def controller_manager_disconnect(self, ctrlman_ns):
        self.ctrlman_list_types[ctrlman_ns].close()
        del self.ctrlman_list_types[ctrlman_ns]
        combo_ind = self.cm_namespace_combo.findText(ctrlman_ns)
        self.cm_namespace_combo.removeItem(combo_ind)

    def remove_loadable_from_list(self, load_text):
        load_ctrl_ind = self.load_ctrl_combo.findText(load_text)
        self.load_ctrl_combo.removeItem(load_ctrl_ind)
        del self.loadable_params[load_text]

    def add_joints_to_layout(self, joint_names, velocity_limits, joint_types,
                                   limits_lower, limits_upper):

        # first destroy any existing joint layouts
        self.clear_joints_from_layout()

        # now add all the joint layouts 
        for jnt_idx, jnt_name in enumerate(joint_names):
            hlayout = QtGui.QHBoxLayout()
            hlayout.setObjectName('hlayout_%d' % jnt_idx)

            ############################################################################

            vlayout1 = QtGui.QVBoxLayout()
            vlayout1.setObjectName('vlayout1_%d' % jnt_idx)

            joint_label = QtGui.QLabel(jnt_name)
            joint_label.setObjectName('joint_label_%d' % jnt_idx)
            joint_label.setAlignment(QtCore.Qt.AlignCenter)
            vlayout1.addWidget(joint_label)

            hlayout11 = QtGui.QHBoxLayout()
            hlayout11.setObjectName('hlayout11_%d' % jnt_idx)
            cur_buttons = []
            for bside_idx, bside in enumerate(['down', 'up']):
                button = QtGui.QPushButton(self)
                button.setObjectName('button_%s_%d' % (bside, jnt_idx))
                size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, 
                                                QtGui.QSizePolicy.Expanding)
                size_policy.setHorizontalStretch(0)
                size_policy.setVerticalStretch(0)
                size_policy.setHeightForWidth(button.sizePolicy().hasHeightForWidth())
                button.setSizePolicy(size_policy)
                button.setBaseSize(QtCore.QSize(80, 80))
                button.setIcon(QtGui.QIcon.fromTheme(bside))
                button.setIconSize(QtCore.QSize(40,40))
                hlayout11.addWidget(button)
                cur_buttons.append(button)

                button.pressed.connect(partial(self._start_move, jnt_idx, bside))
                button.released.connect(partial(self._stop_move, jnt_idx))

            vlayout1.addLayout(hlayout11)

            hlayout.addLayout(vlayout1)

            ############################################################################

            vlayout2 = QtGui.QVBoxLayout()
            vlayout2.setObjectName('vlayout2_%d' % jnt_idx)

            speed_label = QtGui.QLabel('Speed')
            speed_label.setObjectName('speed_label_%d' % jnt_idx)
            speed_label.setAlignment(QtCore.Qt.AlignCenter)
            vlayout2.addWidget(speed_label)

            speed_box = QtGui.QDoubleSpinBox(self)
            speed_box.setObjectName('speed_box_%d' % jnt_idx)
            speed_box.setMaximum(velocity_limits[jnt_idx])
            speed_box.setMinimum(0.0)
            speed_box.setSingleStep(0.05)
            speed_box.setProperty('value', 0.1)
            vlayout2.addWidget(speed_box)

            hlayout.addLayout(vlayout2)

            ############################################################################

            # vlayout3 = QtGui.QVBoxLayout()
            # vlayout3.setObjectName('vlayout3_%d' % jnt_idx)

            # state_label = QtGui.QLabel('State')
            # state_label.setObjectName('state_label_%d' % jnt_idx)
            # state_label.setAlignment(QtCore.Qt.AlignCenter)
            # vlayout3.addWidget(state_label)

            # joint_mode = QtGui.QTextBrowser(self)
            # joint_mode.setObjectName('q_state_%d' % jnt_idx)
            # size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
            # size_policy.setHorizontalStretch(0)
            # size_policy.setVerticalStretch(0)
            # size_policy.setHeightForWidth(joint_mode.sizePolicy().hasHeightForWidth())
            # joint_mode.setSizePolicy(size_policy)
            # joint_mode.setMinimumSize(QtCore.QSize(100, 31))
            # joint_mode.setMaximumSize(QtCore.QSize(180, 31))
            # joint_mode.setBaseSize(QtCore.QSize(100, 31))
            # joint_mode.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            # joint_mode.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            # joint_mode.setLineWrapMode(QtGui.QTextEdit.NoWrap)
            # joint_mode.setText('STALE')
            # vlayout3.addWidget(joint_mode)

            # hlayout.addLayout(vlayout3)

            ############################################################################

            vlayout4 = QtGui.QVBoxLayout()
            vlayout4.setObjectName('vlayout4_%d' % jnt_idx)

            hlayout41 = QtGui.QHBoxLayout()
            hlayout41.setObjectName('hlayout41_%d' % jnt_idx)

            if joint_types[jnt_idx] == 'prismatic':
                q_left_label = QtGui.QLabel('q (m)')
            else:
                q_left_label = QtGui.QLabel('q (rad)')
            q_left_label.setObjectName('q_left_label_%d' % jnt_idx)
            q_left_label.setAlignment(QtCore.Qt.AlignCenter)
            hlayout41.addWidget(q_left_label)

            q_left_num = QtGui.QTextBrowser(self)
            q_left_num.setObjectName('q_left_num_%d' % jnt_idx)
            size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
            size_policy.setHorizontalStretch(0)
            size_policy.setVerticalStretch(0)
            size_policy.setHeightForWidth(q_left_num.sizePolicy().hasHeightForWidth())
            q_left_num.setSizePolicy(size_policy)
            q_left_num.setMinimumSize(QtCore.QSize(60, 31))
            q_left_num.setMaximumSize(QtCore.QSize(100, 31))
            q_left_num.setBaseSize(QtCore.QSize(80, 31))
            q_left_num.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_left_num.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_left_num.setLineWrapMode(QtGui.QTextEdit.NoWrap)
            hlayout41.addWidget(q_left_num)

            if joint_types[jnt_idx] == 'prismatic':
                q_right_label = QtGui.QLabel('q (ft)')
            else:
                q_right_label = QtGui.QLabel('q (deg)')
            q_right_label.setObjectName('q_right_label_%d' % jnt_idx)
            q_right_label.setAlignment(QtCore.Qt.AlignCenter)
            hlayout41.addWidget(q_right_label)

            q_right_num = QtGui.QTextBrowser(self)
            q_right_num.setObjectName('q_right_num_%d' % jnt_idx)
            size_policy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
            size_policy.setHorizontalStretch(0)
            size_policy.setVerticalStretch(0)
            size_policy.setHeightForWidth(q_right_num.sizePolicy().hasHeightForWidth())
            q_right_num.setSizePolicy(size_policy)
            q_right_num.setMinimumSize(QtCore.QSize(60, 31))
            q_right_num.setMaximumSize(QtCore.QSize(100, 31))
            q_right_num.setBaseSize(QtCore.QSize(80, 31))
            q_right_num.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_right_num.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
            q_right_num.setLineWrapMode(QtGui.QTextEdit.NoWrap)
            hlayout41.addWidget(q_right_num)

            vlayout4.addLayout(hlayout41)

            hlayout42 = QtGui.QHBoxLayout()
            hlayout42.setObjectName('hlayout42_%d' % jnt_idx)
            q_slider = QtGui.QSlider(self)
            q_slider.setObjectName('q_slider_%d' % jnt_idx)
            q_slider.setTickPosition(QtGui.QSlider.TicksBelow)
            q_slider.setMinimum(int(limits_lower[jnt_idx]*1000.0))
            q_slider.setMaximum(int(limits_upper[jnt_idx]*1000.0))
            if joint_types[jnt_idx] == 'prismatic':
                q_slider.setTickInterval(int(0.25*1000.0))
            else:
                q_slider.setTickInterval(int(np.pi/4.0*1000.0))
            q_slider.setValue(0)
            q_slider.setEnabled(False)
            q_slider.setOrientation(QtCore.Qt.Horizontal)
            hlayout42.addWidget(q_slider)

            vlayout4.addLayout(hlayout42)

            hlayout.addLayout(vlayout4)

            self.vlayout_joints.addLayout(hlayout)

            # collect the updatable components
            self.buttons.append(cur_buttons)
            self.speed_boxes.append(speed_box)
            # self.joint_modes.append(joint_mode)
            self.q_left_nums.append(q_left_num)
            self.q_right_nums.append(q_right_num)
            self.q_sliders.append(q_slider)

        # connect signals
        # self.state_changed.connect(self._update_state)
        # for i in range(6):
        #     for j in range(2):
        #         self.buttons[i][j].pressed.connect(partial(self._start_move, i, j))
        #         self.buttons[i][j].released.connect(self._stop_move)
        self.state_changed.connect(self._update_state)
        self.start_moving.connect(self.start_moving_call)
        self.stop_moving.connect(self.stop_moving_call)

    def clear_joints_from_layout(self):
        if self._joint_state_sub is not None:
            self._joint_state_sub.unregister()
            self._joint_state_sub = None

        while True:
            item = self.vlayout_joints.takeAt(0)
            if item is None:
                break
            item_widget = item.widget()
            del item_widget
            del item

        self.clear_internal_state()

    def load_jog_ctrl(self):
        ctrlman_ns = self.cm_namespace_combo.currentText()
        load_text = self.load_ctrl_combo.currentText()
        ctrl_param = self.loadable_params[load_text]

        joints_param = ctrl_param + '/joints'
        if rospy.has_param(joints_param):
            joint_names = rospy.get_param(joints_param)
        else:
            self.sig_sysmsg.emit("No /joints parameter for this controller")
            return

        heartbeat_param = ctrl_param + '/heartbeat_timeout'
        if rospy.has_param(heartbeat_param):
            self._heartbeat_timeout = rospy.get_param(heartbeat_param)
        else:
            self._heartbeat_timeout = 0.3

        joint_types = []
        velocity_limits = []
        limits_lower = []
        limits_upper = []
        jnt_cmd_pubs = []

        robot_desc_param = self.ctrlman_ns_cur + 'robot_description'
        if rospy.has_param(robot_desc_param):
            robot = Robot.from_parameter_server(robot_desc_param)
            for joint_name in joint_names:
                if joint_name not in robot.joint_map:
                    self.sig_sysmsg.emit("Joint" + joint_name + "not in URDF")
                    return
                joint = robot.joint_map[joint_name]
                joint_types.append(joint.type)
                if joint.type == 'continuous' or joint.limit is None:
                    limits_lower.append(0.0)
                    limits_upper.append(2.0*np.pi)
                else:
                    limits_lower.append(joint.limit.lower)
                    limits_upper.append(joint.limit.upper)

                joint_param = ctrl_param + '/' + joint_name
                if rospy.has_param(joint_param + "/velocity_limit"):
                    vel_limit = rospy.get_param(joint_param + "/velocity_limit")
                elif joint.limit.velocity is not None:
                    vel_limit = joint.limit.velocity
                else:
                    vel_limit = 1.0
                velocity_limits.append(vel_limit)

                jnt_cmd_pub = rospy.Publisher(ctrl_param + '/' + joint_name + '/command', Float64)
                jnt_cmd_pubs.append(jnt_cmd_pub)

        else:
            self.sig_sysmsg.emit("No robot_description found.")
            return

        self.add_joints_to_layout(joint_names, velocity_limits, joint_types, 
                                  limits_lower, limits_upper)

        self._joint_state_sub = rospy.Subscriber(self.ctrlman_ns_cur + 'joint_states', JointState, 
                                                  self._joint_states_cb)

        # self._joint_mode_text = ['STALE']*len(joint_names)
        self._q_pos           = [0.0]*len(joint_names)
        self._q_order         = None
        self._joint_names     = joint_names
        self._velocity_limits = velocity_limits
        self._joint_types     = joint_types
        self._limits_lower    = limits_lower
        self._limits_upper    = limits_upper
        self._jnt_cmd_pubs    = jnt_cmd_pubs
        self._pub_timers      = [None]*len(joint_names)

    def unload_jog_ctrl(self):
        self.clear_joints_from_layout()

    def _update_state(self):
        for i, jnt_name in enumerate(self._joint_names):
            # self.joint_modes[i].setText(self._joint_mode_text[i])
            self.q_left_nums[i].setText('%.4f' % self._q_pos[i])
            if self._joint_types[i] == 'prismatic':
                q_right = self._q_pos[i] / METERS_PER_FOOT
            else:
                q_right = np.rad2deg(self._q_pos[i])
            self.q_right_nums[i].setText('%.2f' % q_right)
            self.q_sliders[i].setValue(int(self._q_pos[i] * 1000.0))

    def _start_move(self, jnt_idx, direction):
        velocity = self.speed_boxes[jnt_idx].value()
        if direction == 'down':
            velocity = -velocity
        self.start_moving.emit(jnt_idx, velocity)

    def _stop_move(self, jnt_idx):
        self.stop_moving.emit(jnt_idx)

    def start_moving_call(self, jnt_idx, velocity):
        def pub_vel_loop(te):
            self._jnt_cmd_pubs[jnt_idx].publish(Float64(velocity))
        pub_timer = rospy.Timer(rospy.Duration(self._heartbeat_timeout/2.0), pub_vel_loop)
        self._pub_timers[jnt_idx] = pub_timer

    def stop_moving_call(self, jnt_idx):
        self._pub_timers[jnt_idx].shutdown()
        self._pub_timers[jnt_idx].join()
        self._pub_timers[jnt_idx] = None
        self._jnt_cmd_pubs[jnt_idx].publish(Float64(0.0))

    # def _diagnostics_cb(self, msg):
    #     if rospy.is_shutdown():
    #         return
    #     for status in msg.status:
    #         if "UR arm joint" in status.name:
    #             joint_mode = status.values[0].value
    #             for i, jnt_name in enumerate(JOINT_SHORT_NAMES):
    #                 if jnt_name in status.name:
    #                     self._joint_mode_text[i] = joint_mode
    #                     break
    #     self.state_changed.emit()

    def _joint_states_cb(self, msg):
        if rospy.is_shutdown():
            return
        if self._q_order is None:
            self._q_order = []
            for jnt_name in self._joint_names:
                self._q_order.append(msg.name.index(jnt_name))

        for i, q_idx in enumerate(self._q_order):
            self._q_pos[i] = msg.position[q_idx]
        self.state_changed.emit()

    def shutdown_plugin(self):
        # self._diag_agg_sub.unregister()
        if self._joint_state_sub is not None:
            self._joint_state_sub.unregister()
        self._timer_update_commands.stop()
