#!/usr/bin/python

#############################################################
#                                                           #
#   Calder Phillips-Grafflin -- ARC Lab                     #
#                                                           #
#   Displays interactive markers of valve-like objects and  #
#   allows the user to align the objects against the world  #
#   represented by the robot's sensors.                     #
#                                                           #
#############################################################

import rospy
import math
import time
import threading
import subprocess
from copy import deepcopy

from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from shape_msgs.msg import *
from transformation_helper import *
from icp_server_msgs.msg import *
from icp_server_msgs.srv import *
from valve_planner_msgs.msg import *
from valve_planner_msgs.srv import *
from hubo_manipulation_planner.srv import PosePlanningSrv
import tf_conversions.posemath as pm
import tf

import hubo_robot_msgs.msg

import pdb

import actionlib

class ObjectStatus:

    CW = "CW"
    CCW = "CCW"
    PREGRASP = "PREGRASP"
    MOVEFORWARD = "MOVEFORWARD"
    UNSNAPPED = 'UNSNAPPED'
    SNAPPED = 'SNAPPED'
    SNAPPEDPLUS = 'SNAPPEDPLUS'
    LEFT = 'LH'
    RIGHT = 'RH'
    BOTH = 'BH'
    ROUND = 'W'
    LEFTLEVER = 'LL'
    RIGHTLEVER = 'RL'
    READY = 'READY'
    PLANNING = 'PLANNING'
    PLANNED = 'PLANNED'
    EXECUTING = 'EXECUTING'
    EXECUTED = 'EXECUTED'
    ERROR = 'ERROR'
    IDLE = "IDLE"
    GETREADY = "GETREADY"
    GRASP = "GRASP"
    UNGRASP = "UNGRASP"
    TURN = "TURNVALVE"
    FINISH = "END"
    PREVIEW = "PREVIEW"
    EXECUTE = "EXECUTE"

    def __init__(self):
        self.default_thickness = 0.02
        self.default_radius = 0.05
        self.default_height = 0.1
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
        self.icp_status = self.UNSNAPPED
        self.hands = self.BOTH
        self.object_type = self.ROUND
        self.operating_status = self.READY
        self.turning_direction = self.CW
        self.last_pointcloud = None
        self.planning_status = self.IDLE
        self.last_plan = []

class ObjectLocalizer:

    def __init__(self, marker_namespace, data_dir, planner_service, execution_service):
        self.marker_namespace = marker_namespace
        self.populate_menu()
        self.status = ObjectStatus()
        #Setup the interactive marker server
        self.server = InteractiveMarkerServer(self.marker_namespace)
        rospy.loginfo("Connecting to planner...")
        #self.planner_client = rospy.ServiceProxy(planner_service, PlanTurning)
        #self.planner_client.wait_for_service()
        self.pregrasp_planner_client = rospy.ServiceProxy('/pregrasp_planning',PosePlanningSrv)
        self.pregrasp_planner_client.wait_for_service()
        self.move_straight_planner_client = rospy.ServiceProxy('/move_straight_planning',PosePlanningSrv)
        
        #self.execution_client = rospy.ServiceProxy(execution_service, ExecuteTurning)
        #self.execution_client.wait_for_service()
        rospy.loginfo("...Connected to planner")
        rate = rospy.Rate(20.0)
        self.tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        # This bugfix should have made it into Groovy
        #for marker_name in self.server.marker_contexts.keys():
        #    self.server.erase(marker_name)
        self.server.clear()
        alignment_marker = self.make_alignment_imarker(self.status.pose_stamped)
        self.server.insert(alignment_marker, self.alignment_feedback_cb)
        self.menu_handler.apply(self.server, "object_alignment")
        self.server.applyChanges()

    def populate_menu(self):
        self.options = ["Snap with ICP", "Plan PREGRASP", "Plan MOVEFORWARD", "Plan GRASP", "Plan UNGRASP", "Plan TURNING", "Plan FINISH", "Preview plan", "Execute plan", "Increase radius by 1.0cm", "Decrease radius by 1.0cm", "Increase height by 1.0cm", "Decrease height by 1.0cm", "Reset to default radius","Reset to default height", "Reset to default pose", "Reset to session default pose", "Set session default pose", "Use LEFT hand", "Use RIGHT hand", "Turn object CLOCKWISE", "Turn object COUNTERCLOCKWISE"]
        self.menu_handler = MenuHandler()
        i = 1
        for menu_option in self.options:
            print "Option ID: " + str(i) + " option: " + menu_option
            self.menu_handler.insert(menu_option, callback=self.alignment_feedback_cb)
            i += 1

    def process_menu_select(self, menu_entry_id):
        if (menu_entry_id == 1):
            print "Snapping with ICP!"
        elif (menu_entry_id == 2):
            self.call_planner(self.status.PREGRASP)
        elif (menu_entry_id == 3):
            self.call_planner(self.status.MOVEFORWARD)
        elif (menu_entry_id == 4):
            self.call_planner(self.status.GRASP)
        elif (menu_entry_id == 5):
            self.call_planner(self.status.UNGRASP)
        elif (menu_entry_id == 6):
            self.call_planner(self.status.TURN)
        elif (menu_entry_id == 7):
            self.call_planner(self.status.FINISH)
        elif (menu_entry_id == 8):
            self.call_execute(self.status.PREVIEW)
        elif (menu_entry_id == 9):
            self.call_execute(self.status.EXECUTE)
        elif (menu_entry_id == 10):
            self.status.radius += 0.01
        elif (menu_entry_id == 11):
            self.status.radius += (-0.01)
        elif (menu_entry_id == 12):
            self.status.height += 0.01
        elif (menu_entry_id == 13):
            self.status.height += (-0.01)
        elif (menu_entry_id == 14):
            self.status.radius = self.status.default_radius
        elif (menu_entry_id == 15):
            self.status.height = self.status.default_height
        elif (menu_entry_id == 16):
            self.status.pose_stamped = deepcopy(self.status.default_pose_stamped)
        elif (menu_entry_id == 17):
            self.status.pose_stamped = deepcopy(self.status.session_pose_stamped)
        elif (menu_entry_id == 18):
            self.status.session_pose_stamped = deepcopy(self.status.pose_stamped)
        elif (menu_entry_id == 19):
            self.status.hands = self.status.LEFT
        elif (menu_entry_id == 20):
            self.status.hands = self.status.RIGHT
        elif (menu_entry_id == 21):
            self.status.turning_direction = self.status.CW
        elif (menu_entry_id == 22):
            self.status.turning_direction = self.status.CCW
        else:
            rospy.logerr("Unrecognized menu entry")

    def call_icp(self):
        if (self.status.last_pointcloud == None):
            rospy.logerr("No pointcloud available for using ICP")
            return
        req = RunICPRequest()
        # Set the target pointcloud
        req.Request.TargetPointCloud = self.status.last_pointcloud
        # Set the origin of the bounding box
        req.Request.UseBoundingVolume = True
        req.Request.BoundingVolumeOrigin = object_in_sensor_frame_transform
        # Set the bounding box size
        req.Request.BoundingVolumeDimensions.x = self.status.radius * 3.0
        req.Request.BoundingVolumeDimensions.y = self.status.radius * 3.0
        req.Request.BoundingVolumeDimensions.z = self.status.radius * 3.0
        # Set the input pointcloud
        req.Request.InputType = req.Request.SHAPE
        req.Request.SolidPrimitive = SolidPrimitive()
        req.Request.SolidPrimitive.type = SolidPrimitive.CYLINDER
        res = None
        try:
            res = self.icp_client.call(req)
        except:
            res = None
            rospy.logerr("Service call failed to connect. Is the ICP server running?")
        if (res != None):
            pose_adjustment = PoseFromTransform(res.Response.AlignmentTransform)
            self.status.pose_stamped = ComposePoses(self.status.pose_stamped.pose,pose_adjustment)

    def call_planner(self, stage):
        self.status.planning_status = stage
        print stage

        if stage == ObjectStatus.PREGRASP:
            
            target_pose = self.status.pose_stamped.pose
            
            pregrasp_distance = .1

            #The marker pose is defined in the torso coordinate system
            target_tran = pm.toMatrix(pm.fromMsg(target_pose))
            hand_tran = target_tran.copy()


            #Align hand perpendicular to cylinder in its canonical pose
            hand_tran[:3,1] = target_tran[:3,2]
            hand_tran[:3,2] = target_tran[:3,0]
            hand_tran[:3,0] = target_tran[:3,1]
            pregrasp_tran = hand_tran.copy()
            #The approach direction of this gripper is -y for some reason
            pregrasp_tran[:3,3] = pregrasp_tran[:3,3] + pregrasp_tran[:3,1]*pregrasp_distance

            hand_tran_pose = pm.toMsg(pm.fromMatrix(hand_tran))
            pregrasp_pose = pm.toMsg(pm.fromMatrix(pregrasp_tran))



            req = PosePlanningSrv._request_class()
            req.TargetPoses = [pregrasp_pose, hand_tran_pose]
            req.ManipulatorID = int(self.status.hands == self.status.RIGHT)
            res = None
            try:
                print req
                res = self.pregrasp_planner_client.call(req)
                self.status.operating_status = self.status.PLANNING
                #print res
                self.last_plan = res.PlannedTrajectory
            except:
                res = None
                rospy.loginfo("Planner failed to pregrasp")
                         
                self.status.operating_status = self.status.ERROR

    def call_execute(self, mode):
        if not self.last_plan:
            pass
        # Creates a SimpleActionClient, passing the type of action to the constructor.
        client = actionlib.SimpleActionClient('/hubo_trajectory_server_joint', hubo_robot_msgs.msg.JointTrajectoryAction )
        print "waiting for server!"
        client.wait_for_server()
        self.last_plan.header.stamp = rospy.Time.now()
        traj_goal = hubo_robot_msgs.msg.JointTrajectoryGoal()
        traj_goal.trajectory = self.last_plan
        traj_goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(1.0)
        client.send_goal( traj_goal )
        print "Wait for result!"
        client.wait_for_result()
        res = client.get_result()
        print res 

    def alignment_feedback_cb(self, feedback):
        cur_pose = feedback.pose
        event_type = feedback.event_type
        if (event_type == feedback.MOUSE_DOWN):
            pass
        elif (event_type == feedback.MOUSE_UP):
            pass
        elif (event_type == feedback.POSE_UPDATE):
            self.status.pose_stamped.pose = feedback.pose
        elif (event_type == feedback.MENU_SELECT):
            rospy.loginfo("Menu feedback selection: " + self.options[feedback.menu_entry_id - 1])
            self.process_menu_select(feedback.menu_entry_id)
        else:
            rospy.logerr("MENU Unrecognized feedback type - " + str(feedback.event_type))

    def make_alignment_imarker(self, marker_pose):
        new_marker = InteractiveMarker()
        new_marker.header.frame_id = marker_pose.header.frame_id
        new_marker.pose = marker_pose.pose
        new_marker.scale = 1.0
        new_marker.name = 'object_alignment'
        # Make the default control for the marker itself
        base_control = InteractiveMarkerControl()
        base_control.orientation_mode = InteractiveMarkerControl.FIXED
        base_control.always_visible = True
        display_marker = self.make_object_marker(marker_pose)
        base_control.markers.append(display_marker)
        new_marker.controls.append(base_control)
        # Make the menu control
        new_control = InteractiveMarkerControl()
        new_control.interaction_mode = InteractiveMarkerControl.MENU
        new_control.always_visible = True
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.markers.append(display_marker)
        new_marker.controls.append(new_control)
        # Make the x-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_x"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 1.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_y"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the z-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "translate_z"
        new_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        new_marker.controls.append(new_control)
        # Make the x-axis rotation control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_x"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 1.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the y-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_y"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 1.0
        new_control.orientation.z = 0.0
        new_marker.controls.append(new_control)
        # Make the z-axis control
        new_control = InteractiveMarkerControl()
        new_control.name = "rotate_z"
        new_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        new_control.always_visible = False
        new_control.orientation_mode = InteractiveMarkerControl.INHERIT
        new_control.orientation.w = 1.0
        new_control.orientation.x = 0.0
        new_control.orientation.y = 0.0
        new_control.orientation.z = 1.0
        new_marker.controls.append(new_control)
        return new_marker

    def make_object_marker(self, marker_pose):
        marker = Marker()
        marker.header.frame_id = marker_pose.header.frame_id
        #Give it a unique ID
        marker.ns = "object_alignment"
        marker.id = 1
        marker.type = Marker.CYLINDER
        pose_offset = Pose()
        q1 = quaternion_about_axis(math.pi / 2.0, (0,1,0))
        pose_offset.orientation.x = q1[0]
        pose_offset.orientation.y = q1[1]
        pose_offset.orientation.z = q1[2]
        pose_offset.orientation.w = q1[3]
        rotated_pose = ComposePoses(marker_pose.pose, pose_offset)
        marker.pose = rotated_pose
        #Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = (self.status.radius * 2.0)
        marker.scale.y = (self.status.radius * 2.0)
        marker.scale.z = self.status.height
        #Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1.0
        marker.color.b = 1.0
        marker.color.g = 0.0
        marker.color.a = 0.75
        marker.lifetime = rospy.Duration(0.5)
        return marker

if __name__ == '__main__':
    rospy.init_node("object_localizer")
    path = subprocess.check_output("rospack find valve_localization", shell=True)
    path = path.strip("\n") + "/data"
    ims_namespace = rospy.get_param("~ims_namespace", "object_localizer")
    data_dir = rospy.get_param("~data_dir", path)
    planner_service = rospy.get_param("~planner_service", "object_planner/drchubo_planner/PlanningQuery")
    execution_service = rospy.get_param("~execution_service", "object_planner/drchubo_planner/ExecutionQuery")
    ObjectLocalizer(ims_namespace, data_dir, planner_service, execution_service)
