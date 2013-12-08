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
        self.base_radius = .076/2
        self.base_height = .0571
        self.branch_radius= .0571/2
        self.branch_height=.0571
        self.wye_angle = math.pi/4
        self.height_overlap_fraction = .2
        self.pin_height = .003715

class WyeMarker(MoveableButtonMarker):
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
        self.cylinder_pose_pub = rospy.Publisher('/graspable_cylinder_test/set_absolute_pose', PoseStamped)


    def send_hand_pose(self, pose_stamped_msg):
        self.hand_pose_publishers[self.status.hands].publish(pose_stamped_msg)

    
    def create_marker(self):
        MoveableButtonMarker.create_marker(self)


    
        
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
        
               
        base_control.name = self.int_marker.name + 'base_control'
        self.int_marker.controls.append(base_control)
        self.build_wye()
        

    
    def mod_property(self, mod_object,mod_attr,mod_value):
        '''Modify state machine attribute by a certain amount '''
        current_value = getattr(mod_object,mod_attr)        
        setattr(mod_object,mod_attr,current_value+mod_value)
        self.make_object_marker()

    def switch_property(self, mod_object,mod_attr,new_mod_attr):
        '''Replace one state machine attribute with another attribute'''
        current_value = getattr(mod_object,new_mod_attr)
        if new_mod_attr in ['default_pose_stamped','session_pose_stamped','pose_stamped']:
            current_value = deepcopy(current_value)
        setattr(mod_object,mod_attr,current_value)


    def send_pregrasp_transform(self):
        #send the pregrasp transform in the wye's frame
        pregrasp_pose_stamped = PoseStamped()
        pregrasp_pose_stamped.header.frame_id = self.int_marker.name
        pregrasp_pose_stamped.pose = PoseFromTransform(TransformFromComponents([-0.220, -0.272, -0.000], [-0.000, -0.000, 0.965, -0.263]))
        self.send_hand_pose(pregrasp_pose_stamped)

    def enter_pdb(self):
        ipdb.set_trace()


    def build_wye(self):
        if self.int_marker.name.find('wye') < 0:
            self.int_marker.name += "_wye"
        for marker in self.int_marker.controls[0].markers:
            if marker.text.find('wye') >= 0:
                self.int_marker.controls[0].markers.remove(marker)
        base_cylinder = Marker()
        base_cylinder.type=Marker.CYLINDER
        base_cylinder.scale.x = base_cylinder.scale.y = self.status.base_radius*2
        base_cylinder.scale.z = self.status.base_height
        base_cylinder.text = "wye_base"
        base_cylinder.color = std_msgs.msg.ColorRGBA(1.0,1.0,0,1.0)
        
        base_cylinder.header.frame_id = self.int_marker.name
        left_cylinder = Marker()
        left_cylinder.type = Marker.CYLINDER
        left_cylinder.scale.x = left_cylinder.scale.y = self.status.branch_radius*2
        left_cylinder.scale.z = self.status.branch_height
        left_cylinder.text="wye_left"
        
        left_cylinder.header.frame_id = self.int_marker.name
        left_cylinder.color = std_msgs.msg.ColorRGBA(1.0,1.0,0,1.0)
        right_cylinder = deepcopy(left_cylinder)
        right_cylinder.text="wye_right"
        self.update_wye_pose()
        self.int_marker.controls[0].markers.append(base_cylinder)
        self.int_marker.controls[0].markers.append(left_cylinder)
        self.int_marker.controls[0].markers.append(right_cylinder)


    def update_wye_pose(self):
        offset_poses = [Pose(), self.build_branch_offset(self.status.wye_angle), self.build_branch_offset(-self.status.wye_angle)]
        for offset_pose, marker in zip(offset_poses,self.int_marker.controls[0].markers):
            marker.pose = offset_pose
        
        


    def build_branch_offset(self, angle):
        
        offset_z = self.status.height_overlap_fraction * self.status.base_height/2 + self.status.branch_height/2
        rotation_transform = tf.transformations.euler_matrix(angle, 0,0,'sxyz')
        offset_transform = numpy.eye(4)
        offset_transform[2,3] = offset_z
        total_transform = numpy.dot(rotation_transform, offset_transform)
        return PoseFromMatrix(total_transform)
    
    def initialize_menu(self):
        
        self.menu_options = [        
            {'label':"send pregrasp transform", 'action':self.send_pregrasp_transform, 'args':[]},


            {'label':"Increase base radius by 0.2 cm", 'action': self.mod_property, 'args':[self.status,'base_radius',0.002]},
            {'label':"Decrease base radius by 0.02cm", 'action': self.mod_property, 'args':[self.status,'base_radius',-0.002]},
            {'label':"Increase base height by 1.0cm", 'action': self.mod_property, 'args':[self.status,'base_height',0.002]},
            {'label':"Decrease base height by 1.0cm", 'action': self.mod_property, 'args':[self.status,'base_height',-0.002]},
            {'label':"Increase branch radius by 0.2 cm", 'action': self.mod_property, 'args':[self.status,'branch_radius',0.002]},
            {'label':"Decrease branch radius by 0.2cm", 'action': self.mod_property, 'args':[self.status,'branch_radius',-0.002]},
            {'label':"Increase branch height by 0.2cm", 'action': self.mod_property, 'args':[self.status,'branch_height',0.002]},
            {'label':"Decrease branch height by 0.2cm", 'action': self.mod_property, 'args':[self.status,'branch_height',-0.002]},

            {'label':"Increase offset fraction by 1%", 'action': self.mod_property, 'args':[self.status,'height_overlap_fraction',0.1]},
            {'label':"Decrease offset fraction by 1%", 'action': self.mod_property, 'args':[self.status,'height_overlap_fraction',-0.1]},
            {'label':"Increase angle by .02", 'action': self.mod_property, 'args':[self.status,'wye_angle',0.2]},
            {'label':"Decrease angle by .02", 'action': self.mod_property, 'args':[self.status,'wye_angle',-0.02]},

            {'label':"Reset to default pose", 'action': self.switch_property, 'args':[self.status,'pose_stamped','default_pose_stamped']},
            {'label':"Reset to session default pose", 'action': self.switch_property, 'args':[self.status,'pose_stamped','session_pose_stamped']},
            {'label':"Set session default pose", 'action': self.switch_property, 'args':[self.status,'session_pose_stamped','pose_stamped']},
            {'label':"Use LEFT hand", 'action': self.switch_property, 'args':[self.status,'hands','LEFT']},
            {'label':"Use RIGHT hand", 'action': self.switch_property, 'args':[self.status,'hands','RIGHT']},

            {'label':"Enter Debugger", 'action': self.enter_pdb, 'args' :[]},
            {'label':"Align cylinder to right wye", 'action':self.set_cylinder_pose,'args':[[[-0.017, -0.075, 0.074],[-0.069, -0.329, 0.942, -0.020]]]},
            {'label':"Align cylinder to left wye", 'action':self.set_cylinder_pose,'args':[[[-0.016, 0.050, 0.075],[-0.067, 0.324, 0.943, 0.027]]]}
            
           
        ]

        self.populate_menu()
            
    def set_cylinder_pose(self, cylinder_tf):
        pose = PoseStamped()
        pose.pose=PoseFromTransform(TransformFromComponents(*cylinder_tf))
        pose.header.frame_id = self.int_marker.name
        self.cylinder_pose_pub.publish(pose)
                                    

    def update(self):
        
        MoveableButtonMarker.update(self)
        tfp = ComponentsFromTransform(PoseToTransform(self.int_marker.pose))
        self.update_wye_pose()
        
        self.tf_broadcaster.sendTransform(tfp[0],tfp[1], rospy.Time().now(), self.int_marker.name, self.int_marker.header.frame_id )



if __name__ == '__main__':
    rospy.init_node('wye_marker_test')
    marker_tran = TransformToMatrix(TransformFromComponents([0.811, -0.037, 0.423], [0.741, 0.018, -0.671, -0.020]))
    
    wye_marker = WyeMarker('wye_test','/leftFoot', marker_tran)
    wye_marker.initialize_menu()
    loop = rospy.Rate(20)

    server = InteractiveMarkerServer(wye_marker.marker_namespace)
    wye_marker.create_marker()

    while not rospy.is_shutdown():        
        wye_marker.update()
        server.clear()
        server.insert(wye_marker.int_marker, wye_marker.int_marker_feedback_cb)
        wye_marker.menu_handler.apply(server, wye_marker.int_marker.name)
        wye_marker.update()
        server.applyChanges()
        
        loop.sleep()
