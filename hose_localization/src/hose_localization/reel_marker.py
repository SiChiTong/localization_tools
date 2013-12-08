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
from moveable_button_marker import MoveableButtonMarker
import ipdb
import stable_pointcloud_snapshotter
import copy
class ObjectStatus:

    CW = "CW"
    CCW = "CCW"
    PREGRASP = "PREGRASP"
    MOVEFORWARD = "MOVEFORWARD"
    LEFT = 'LH'
    RIGHT = 'RH'
    PREVIEW = "PREVIEW"
    EXECUTE = "EXECUTE"

    def __init__(self):
        self.default_thickness = 0.02
        self.default_radius = 0.2286
        self.default_height = 0.0984
        
        self.radius = self.default_radius
        self.height = self.default_height
        self.default_pose_stamped = PoseStamped()
        self.default_pose_stamped.header.frame_id = "/Body_TSY"
        self.default_pose_stamped.pose.position.x = 1.0
        self.default_pose_stamped.pose.position.y = 0.0
        self.default_pose_stamped.pose.position.z = 0.0
        self.default_pose_stamped.pose.orientation.x = 0.0
        self.default_pose_stamped.pose.orientation.y = 0.0
        self.default_pose_stamped.pose.orientation.z = 0.0
        self.default_pose_stamped.pose.orientation.w = 1.0
        self.session_pose_stamped = deepcopy(self.default_pose_stamped)
        self.pose_stamped = deepcopy(self.default_pose_stamped)
        self.hands = self.LEFT
        self.turning_direction = self.CW        
        self.last_plan = []



class GraspableReelMarker(MoveableButtonMarker):
    def __init__(self, marker_namespace, frame='/world', marker_tran = numpy.eye(4)):        
        self.status = ObjectStatus()        
        MoveableButtonMarker.__init__(self, marker_namespace, frame, marker_tran)

        self.left_pose_topic = "/left_gripper_marker/set_absolute_pose"
        self.right_pose_topic = "/right_gripper_marker/set_absolute_pose"

        self.hand_pose_publishers = { self.status.LEFT: rospy.Publisher(self.left_pose_topic, PoseStamped) , 
                               self.status.RIGHT: rospy.Publisher(self.right_pose_topic, PoseStamped)       
                               }
        self.point_sub = rospy.Subscriber('/clicked_point',PointStamped, self.set_position)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()


    def send_hand_pose(self, pose_stamped_msg):
        self.hand_pose_publishers[self.status.hands].publish(pose_stamped_msg)

    
    def create_marker(self):
        MoveableButtonMarker.create_marker(self)
        self.int_marker.name += "_graspable_reel"

    
        
    def set_position(self, point_stamped_msg):
        pose = PoseStamped()
        pose.header = point_stamped_msg.header
        pose.pose.position = point_stamped_msg.point
        self.set_pose(pose)
        
    def make_object_marker(self):
        base_control = InteractiveMarkerControl()
        base_control.orientation_mode = InteractiveMarkerControl.FIXED
        base_control.interaction_mode = InteractiveMarkerControl.MENU
        base_control.always_visible = True
        
        self.object_marker = Marker()
        self.object_marker.id = 1
        self.object_marker.type = Marker.CYLINDER
        self.object_marker.lifetime = rospy.Duration(0.5)
        self.object_marker.header.frame_id = self.int_marker.header.frame_id
        
        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        self.object_marker.scale.x = (self.status.radius * 2.0)
        self.object_marker.scale.y = (self.status.radius * 2.0)
        self.object_marker.scale.z = self.status.height
        pose_offset = Pose()
        q1 = quaternion_about_axis(math.pi / 2.0, (1,0,0))
        pose_offset.orientation.x = q1[3]
        pose_offset.orientation.y = q1[2]
        pose_offset.orientation.z = q1[1]
        pose_offset.orientation.w = q1[0]
        self.object_marker.color.r = 1.0
        self.object_marker.color.g = 0.0
        self.object_marker.color.b = 0.0
        self.object_marker.color.a = 0.5

        
        

        rotated_pose = ComposePoses(pose_offset, self.marker_pose_stamped.pose)
        self.object_marker.pose = rotated_pose

        self.inner_cylinder_marker = copy.deepcopy(self.object_marker)
        self.inner_cylinder_marker.color.r = 0.0
        self.inner_cylinder_marker.color.g = 1.0
        self.inner_cylinder_marker.color.b = 1.0
        self.inner_cylinder_marker.color.a = 1.0
        self.inner_cylinder_marker.scale.x -= .047625
        self.inner_cylinder_marker.scale.y -= .047625
        self.inner_cylinder_marker.pose = Pose()
        self.inner_cylinder_marker.header.frame_id = self.int_marker.header.frame_id
        
        
        
        base_control.name = self.int_marker.name + 'base_control'
        base_control.markers.append(self.object_marker)
        base_control.markers.append(self.inner_cylinder_marker)
        self.int_marker.controls.append(base_control)
        

    
    def mod_property(self, mod_object,mod_attr,mod_value):
        '''Modify state machine attribute by a certain amount '''
        current_value = getattr(mod_object,mod_attr)
        self.create_marker()
        setattr(mod_object,mod_attr,current_value+mod_value)
        

    def switch_property(self, mod_object,mod_attr,new_mod_attr):
        '''Replace one state machine attribute with another attribute'''
        current_value = getattr(mod_object,new_mod_attr)
        if new_mod_attr in ['default_pose_stamped','session_pose_stamped','pose_stamped']:
            current_value = deepcopy(current_value)
        setattr(mod_object,mod_attr,current_value)


    def send_pregrasp_transform(self):
        #send the pregrasp transform in the reel's frame
        pregrasp_pose_stamped = PoseStamped()
        pregrasp_pose_stamped.header.frame_id = self.int_marker.name
        pregrasp_pose_stamped.pose = PoseFromTransform(TransformFromComponents([-0.220, -0.272, -0.000], [-0.000, -0.000, 0.965, -0.263]))
        self.send_hand_pose(pregrasp_pose_stamped)

    def enter_pdb(self):
        ipdb.set_trace()


        
    
    def initialize_menu(self):
        
        self.menu_options = [        
            {'label':"send pregrasp transform", 'action':self.send_pregrasp_transform, 'args':[]},


            {'label':"Increase radius by 1.0cm", 'action': self.mod_property, 'args':[self.status,'radius',0.01]},
            {'label':"Decrease radius by 1.0cm", 'action': self.mod_property, 'args':[self.status,'radius',-0.01]},
            {'label':"Increase height by 1.0cm", 'action': self.mod_property, 'args':[self.status,'height',0.01]},
            {'label':"Decrease height by 1.0cm", 'action': self.mod_property, 'args':[self.status,'height',-0.01]},
            {'label':"Reset to default radius", 'action': self.switch_property, 'args':[self.status,'radius','default_radius']},
            {'label':"Reset to default height", 'action': self.switch_property, 'args':[self.status,'height','default_height']},
            {'label':"Reset to default pose", 'action': self.switch_property, 'args':[self.status,'pose_stamped','default_pose_stamped']},
            {'label':"Reset to session default pose", 'action': self.switch_property, 'args':[self.status,'pose_stamped','session_pose_stamped']},
            {'label':"Set session default pose", 'action': self.switch_property, 'args':[self.status,'session_pose_stamped','pose_stamped']},
            {'label':"Use LEFT hand", 'action': self.switch_property, 'args':[self.status,'hands','LEFT']},
            {'label':"Use RIGHT hand", 'action': self.switch_property, 'args':[self.status,'hands','RIGHT']},

            {'label':"Enter Debugger", 'action': self.enter_pdb, 'args' :[]}
        ]

        self.populate_menu()
            

    def update(self):
        
        MoveableButtonMarker.update(self)
        tfp = ComponentsFromTransform(PoseToTransform(self.int_marker.pose))
        self.inner_cylinder_marker.pose = copy.deepcopy(self.object_marker.pose)
        self.inner_cylinder_marker.pose.position = Pose().position
        self.tf_broadcaster.sendTransform(tfp[0],tfp[1], rospy.Time().now(), self.int_marker.name, self.int_marker.header.frame_id )



if __name__ == '__main__':
    rospy.init_node('reel_marker_test')
    marker_tran = numpy.eye(4)
    marker_tran[0,3] = 1
    
    reel_marker = GraspableReelMarker('graspable_reel_test','/leftFoot', marker_tran)
    reel_marker.initialize_menu()
    loop = rospy.Rate(20)

    server = InteractiveMarkerServer(reel_marker.marker_namespace)
    reel_marker.create_marker()

    while not rospy.is_shutdown():        
        reel_marker.update()
        server.clear()
        server.insert(reel_marker.int_marker, reel_marker.int_marker_feedback_cb)
        reel_marker.menu_handler.apply(server, reel_marker.int_marker.name)
        reel_marker.update()
        server.applyChanges()
        
        loop.sleep()
