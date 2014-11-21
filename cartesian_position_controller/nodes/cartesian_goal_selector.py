#!/usr/bin/python

import numpy as np
import copy

import roslib

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer 
from interactive_markers.interactive_marker_server import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl, Marker

from geometry_msgs.msg import PoseStamped

from interactive_markers.menu_handler import MenuHandler

# from hrl_geom.pose_converter import PoseConv

class PoseFinder(object):
    def __init__(self):
        test_prefix = rospy.get_param("~test_prefix", "")

        self.imkr_srv = InteractiveMarkerServer('pose_finder')

        pose_imkr = InteractiveMarker()
        base_frame = rospy.get_param("~base_frame", "/table_link")
        pose_imkr.header.frame_id = base_frame
        pose_imkr.name = 'pose_finder'
        pose_imkr.scale = 0.2

        mkr_cyl = Marker()
        mkr_cyl.color.r = 0.5
        mkr_cyl.color.g = 0.5
        mkr_cyl.color.b = 0.5
        mkr_cyl.color.a = 1.0
        mkr_arr = copy.deepcopy(mkr_cyl)

        mkr_cyl.type = Marker.CYLINDER
        mkr_cyl.scale.x = 0.09
        mkr_cyl.scale.y = 0.09
        mkr_cyl.scale.z = 0.03
        mkr_cyl.pose.orientation.y = 1
        mkr_cyl.pose.orientation.w = 1

        mkr_arr.type = Marker.ARROW
        mkr_arr.scale.x = 0.08
        mkr_arr.scale.y = 0.03
        mkr_arr.scale.z = 0.03
        mkr_arr.pose.orientation.x = 1
        mkr_arr.pose.orientation.w = 1

        imkr_ctrl = InteractiveMarkerControl()
        imkr_ctrl.name = 'wrist'
        imkr_ctrl.always_visible = True
        imkr_ctrl.markers.append(mkr_cyl)
        imkr_ctrl.markers.append(mkr_arr)
        pose_imkr.controls.append(imkr_ctrl)

        names = {'x' : [1, 0, 0, 1], 'y' : [0, 1, 0, 1], 'z' : [0, 0, 1, 1]}
        ctrl_types = {'move_%s' : InteractiveMarkerControl.MOVE_AXIS, 
                      'rotate_%s' : InteractiveMarkerControl.ROTATE_AXIS}
        for name in names:
            for ctrl_type in ctrl_types:
                ctrl = InteractiveMarkerControl()
                ctrl.name = ctrl_type % name
                ctrl.interaction_mode = ctrl_types[ctrl_type]
                q = names[name]
                ctrl.orientation.x = q[0]
                ctrl.orientation.y = q[1]
                ctrl.orientation.z = q[2]
                ctrl.orientation.w = q[3]
                pose_imkr.controls.append(ctrl)

        self.imkr_srv.insert(pose_imkr, self.proc_fb)
        self.imkr_srv.applyChanges()
        # self.imkr_srv.setPose('pose_finder', PoseConv.to_pose_msg([0.]*3, [0.]*3))

        sideways_hand = MenuHandler()
        sideways_hand.insert('Sideways point', callback=self.sideways_press)
        sideways_hand.insert('Vertical point', callback=self.vertical_press)
        sideways_hand.insert('Go to pose', callback=self.goto_press)
        sideways_hand.apply(self.imkr_srv, 'pose_finder')

        self.imkr_srv.applyChanges()

        command_topic = rospy.get_param("~command_topic", "/vel_cart_pos_ctrl/command_pose")
        self.pose_pub = rospy.Publisher(command_topic, PoseStamped)

    def proc_fb(self, fb):
        pass
        # print "proc_fb"
        # pose = PoseConv.to_homo_mat(fb.pose)

    def goto_press(self, fb):
        print "goto_press"
        ps = PoseStamped()
        ps.pose = fb.pose
        self.pose_pub.publish(ps)
        # pose = PoseConv.to_homo_mat(fb.pose)

    def sideways_press(self, fb):
        print "sideways_press"
        # pos, euler = PoseConv.to_pos_euler(fb.pose)
        # euler = [0., 0., 0.]
        # self.imkr_srv.setPose('pose_finder', PoseConv.to_pose_msg(pos, euler))
        # self.imkr_srv.applyChanges()
        # self.proc_fb(fb)

    def vertical_press(self, fb):
        print "vertical_press"
        # pos, euler = PoseConv.to_pos_euler(fb.pose)
        # euler = [0., np.pi/2., 0.]
        # self.imkr_srv.setPose('pose_finder', PoseConv.to_pose_msg(pos, euler))
        # self.imkr_srv.applyChanges()
        # self.proc_fb(fb)

def main():
    rospy.init_node('pose_finder')
    pf = PoseFinder()
    rospy.spin()

if __name__ == '__main__':
    main()
