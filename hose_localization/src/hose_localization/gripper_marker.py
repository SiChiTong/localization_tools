#!/usr/bin/env python

#############################################################
#                                                           #
#   Calder Phillips-Grafflin -- ARC Lab                     #
#                                                           #
#   Asynchronous cartesian end-effector teleoperation of    #
#   the DRCHubo robot using interactive markers in RVIZ.    #
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

class GripperMarker (MoveableButtonMarker):
    def __init__(self, marker_namespace, is_left_side = True, 
               frame="/world", marker_tran = numpy.eye(4)):
        MoveableButtonMarker.__init__(self, marker_namespace, frame, marker_tran)
        self.marker_mesh_file =  "package://drchubo_v3/meshes/convhull_RWR_merged.stl"
        self.side = 'right'
        if is_left_side:
            self.marker_mesh_file = "package://drchubo_v3/meshes/convhull_LWR_merged.stl"
            self.side = 'left'
        
        self.other_frames = ['/Body_TSY','/left_hand']
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        


    def make_object_marker(self):
        base_control = InteractiveMarkerControl()
        base_control.orientation_mode = InteractiveMarkerControl.FIXED
        base_control.interaction_mode = InteractiveMarkerControl.MENU
        base_control.always_visible = True
        self.object_marker = Marker()
        self.object_marker.id = 1
        self.object_marker.type = Marker.MESH_RESOURCE
        self.object_marker.mesh_resource = self.marker_mesh_file
        self.object_marker.mesh_use_embedded_materials = True
        self.object_marker.lifetime = rospy.Duration(0.5)
        self.object_marker.header.frame_id = self.int_marker.header.frame_id
        self.object_marker.scale.x = 1.0
        self.object_marker.scale.y = 1.0
        self.object_marker.scale.z = 1.0
        self.object_marker.color.a = 1.0
        
        
        base_control.name = self.int_marker.name + 'base_control'
        base_control.markers.append(self.object_marker)
        self.int_marker.controls.append(base_control)


    def create_marker(self):
        MoveableButtonMarker.create_marker(self)         
        self.int_marker.name=self.side + "_gripper_marker"
        
    def update(self):
        MoveableButtonMarker.update(self)
        tfp = ComponentsFromTransform(PoseToTransform(self.int_marker.pose))
        self.tf_broadcaster.sendTransform(tfp[0],tfp[1], rospy.Time.now(), self.int_marker.name, self.int_marker.header.frame_id )
        """
        for tf_name in self.other_frames:
            self.tf_listener.waitForTransform(self.int_marker.name, self.int_marker.header.frame_id, rospy.Time(0), rospy.Duration(.5))
            other_basis_tf = self.tf_listener.lookupTransform(self.int_marker.name, tf_name, rospy.Time(0)) 
            self.tf_broadcaster.BroadcastTransform(other_basis_tf[0], other_basis_tf[1], rospy.Time.now(),tf_name, self.int_marker.name + '_to_' + tf_name)
        """


if __name__ == '__main__':
    def print_pose(marker):
        print "Marker pose"
        print marker.marker_pose_stamped

    rospy.init_node('gripper_marker_test')
    grasper_marker = GripperMarker('gripper_marker_test',True, "/leftFoot")
    grasper_marker.update_menu("output pose", print_pose, [grasper_marker])

    grasper_marker.populate_menu()                
    loop = rospy.Rate(5)
    server = InteractiveMarkerServer(grasper_marker.marker_namespace)
    grasper_marker.create_marker()
    while not rospy.is_shutdown():
        grasper_marker.update()
        server.clear()
        server.insert(grasper_marker.int_marker, grasper_marker.int_marker_feedback_cb)
        grasper_marker.menu_handler.apply(server,grasper_marker.int_marker.name)
        grasper_marker.update()
        server.applyChanges()
        
        loop.sleep()


