#!/usr/bin/python
import pdb
#############################################################
#                                                           #
# Jon Weisz - General base class for clickable,             #
# interactive markers in python                             #
#                                                           #
#############################################################

import rospy
import math
from copy import deepcopy

from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from hubo_robot_msgs.msg import *
import actionlib
import tf
from actionlib import *
from transformation_helper import *
import numpy

class MoveableButtonMarker(object):
    def __init__(self, marker_namespace, frame='/world', marker_tran = numpy.eye(4)):        
        self.marker_pose_stamped = PoseStamped()
        self.marker_pose_stamped.pose =  PoseFromMatrix(marker_tran)
        self.marker_pose_stamped.header.frame_id = frame


        self.marker_namespace = marker_namespace

        self.gripper_options = []
        self.menu_options = []
        self.menu_handler = []
        self.int_marker = InteractiveMarker()
        self.object_marker = []
        pose_topic = marker_namespace + '/' + 'set_relative_pose'
        stamped_pose_topic = marker_namespace + '/' + 'set_absolute_pose'
        
        
        self.set_pose_subscriber = rospy.Subscriber(pose_topic, Pose, self.set_pose)
        self.set_absolute_pose_subscriber = rospy.Subscriber(stamped_pose_topic, PoseStamped, self.set_pose)

        

    def update_menu(self, item_text, item_callback, arg_list):
        self.menu_options += [{'label': item_text, 'action':item_callback, 'args':arg_list}]
        self.populate_menu()
        

    def populate_menu(self):
        #Pull the options out the easy way (list comprehension on the menu list)
        self.options = [option['label'] for option in self.menu_options]
        self.menu_handler = MenuHandler()
        i = 1
        for menu_option in self.options:
            print("Option ID: " + str(i) + " option: " + menu_option)
            self.menu_handler.insert(menu_option, callback=self.int_marker_feedback_cb)
            i += 1
    
    def int_marker_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            self.set_pose(feedback.pose)
        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.options[feedback.menu_entry_id - 1])
            self.menu_callback(feedback.menu_entry_id)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))


    def menu_callback(self, menu_entry_id):
        if menu_entry_id <= len(self.menu_options):
            option = self.menu_options[menu_entry_id-1]
            option['action'](*(option['args']))
        else:
            rospy.logerr("Unrecognized menu entry %i"%(menu_entry_id))


    def set_pose(self, msg):
        if isinstance(msg,Pose):
            self.marker_pose_stamped.pose = msg
        else:
            self.marker_pose_stamped = msg
        self.int_marker.header.frame_id = self.marker_pose_stamped.header.frame_id
        self.int_marker.pose = self.marker_pose_stamped.pose


    def create_marker(self):
        
        self.int_marker.name = "button_marker"
        self.int_marker.scale = 1.0
        # Make the default control for the marker itself
        
        self.make_object_marker()
        self.make_alignment_marker()
        self.set_pose(self.marker_pose_stamped)


    def make_object_marker(self):
        base_control = InteractiveMarkerControl()
        base_control.orientation_mode = InteractiveMarkerControl.FIXED
        base_control.interaction_mode = InteractiveMarkerControl.MENU
        base_control.always_visible = True
        self.object_marker = Marker()
        self.object_marker.id = 1
        self.object_marker.type = Marker.SPHERE
        self.object_marker.lifetime = rospy.Duration(0.5)
        self.object_marker.header.frame_id = self.int_marker.header.frame_id
        self.object_marker.scale.x = 1.0
        self.object_marker.scale.y = 1.0
        self.object_marker.scale.z = 1.0
        self.object_marker.color.r = 1.0
        self.object_marker.color.g = 1.0
        self.object_marker.color.b = 1.0
        self.object_marker.color.a = 1.0

        #object_marker.name=self.int_marker.name + '_objectmarker'
        base_control.name = self.int_marker.name + 'base_control'
        base_control.markers.append(self.object_marker)
        self.int_marker.controls.append(base_control)

    def make_alignment_marker(self):
        def create_axis_control(axis, interaction_mode, base_name):
            control = InteractiveMarkerControl()
            control.interaction_mode = interaction_mode
            control.orientation_mode = InteractiveMarkerControl.INHERIT
            control.always_visible = False
            control.name = base_name + axis
            control.orientation.w = 1.0
            control.orientation.x = 0.0
            control.orientation.y = 0.0
            control.orientation.z = 0.0
            
            setattr(control.orientation, axis, 1.0)
            self.int_marker.controls.append(control)
            return control
        
        axes=['x','y','z']
        for axis in axes:
            translation_control = create_axis_control(axis, InteractiveMarkerControl.MOVE_AXIS,'translate')
            rotation_control = create_axis_control(axis, InteractiveMarkerControl.ROTATE_AXIS,'rotate')
            
        return 

    def update(self):
        if not self.object_marker:
            return
        self.object_marker.header.frame_id = self.int_marker.header.frame_id
        self.object_marker.pose = self.int_marker.pose
        

    
