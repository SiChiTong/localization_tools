#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import roslib
import rospy
from transformation_helper import *
import tf
import IPython
import pdb
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from pointcloud2 import *

class StablePointcloudSnapshotter(object):

    def add_pointcloud_to_menu(self, snapshot_number):
        local_top = self.cloud_menu_handler.insert( "Snapshot %d"%(snapshot_number) )
        self.cloud_menu[local_top] = snapshot_number

        #Show/Hide the pointcloud
        local_hide = self.cloud_menu_handler.insert( "Hide", parent = local_top, callback= self.hide_entry_cb)
        self.cloud_menu_handler.setCheckState(local_hide, MenuHandler.UNCHECKED )
        self.cloud_menu[local_hide] = local_top

        local_highlight = self.cloud_menu_handler.insert( "Highlight", parent = local_top, callback= self.hide_entry_cb)
        self.cloud_menu_handler.setCheckState(local_highlight, MenuHandler.UNCHECKED)
        self.cloud_menu[local_highlight] = local_top

        local_trans = self.cloud_menu_handler.insert( "Set Transparency", parent = local_top, callback= self.hide_entry_cb)
        self.cloud_menu_handler.setCheckState(local_trans, MenuHandler.UNCHECKED)
        self.cloud_menu[local_trans] = local_top

        local_move = self.cloud_menu_handler.insert( "Move", parent = local_top, callback= self.hide_entry_cb)
        self.cloud_menu_handler.setCheckState(local_trans, MenuHandler.UNCHECKED)
        self.cloud_menu[local_move] = local_top

        local_delete = self.cloud_menu_handler.insert( "Delete Points", parent = local_top, callback= self.hide_entry_cb)
        self.cloud_menu_handler.setCheckState(local_delete, MenuHandler.UNCHECKED)
        self.cloud_menu[local_delete] = local_top

        local_kill = self.cloud_menu_handler.insert( "Delete Cloud", parent = local_top, callback= self.hide_entry_cb)
        #We'll need this for the callback
        self.cloud_menu[local_kill] = local_top
        return local_top
        

    def hide_entry_cb(self, menu_entry_feedback):
        def get_marker_name_from_entry(menu_entry_id):
            return self.cloud_menu_handler.getTitle(self.cloud_menu[menu_entry_id])


        def hide_marker(menu_entry_feedback):
            marker_name = get_marker_name_from_entry(menu_entry_feedback.menu_entry_id)
            
            markers = self.remove_marker(marker_name)
            self.hidden_markers += markers
            self.cloud_menu_handler.setCheckState(menu_entry_feedback.menu_entry_id, MenuHandler.CHECKED)


        def unhide_marker(menu_entry_feedback):
            marker_name = get_marker_name_from_entry(menu_entry_feedback.menu_entry_id)
            marker = [marker for marker in self.hidden_markers if marker.text == marker_name]
            if marker:
                self.control_marker.controls[0].markers += marker                
                self.cloud_menu_handler.setCheckState(menu_entry_feedback.menu_entry_id, MenuHandler.UNCHECKED)
            
            
        if self.cloud_menu_handler.getCheckState(menu_entry_feedback.menu_entry_id) == MenuHandler.UNCHECKED:
            hide_marker(menu_entry_feedback)
        else:
            unhide_marker(menu_entry_feedback)
        self.reinitialize_server()  


    def __init__(self, stable_frame = 'leftFoot', max_clouds = 1, cloud_topic_name = '', tf_listener = [], tf_broadcaster = []):
        self.tf_listener = tf_listener
        self.tf_broadcaster = tf_broadcaster


        self.cloud_topic_name = cloud_topic_name
        self.stable_frame = stable_frame
        self.max_clouds = max_clouds
        self.snapshot_request_subscriber = rospy.Subscriber('snapshot_cloud', std_msgs.msg.Empty, self.snapshot_callback)
        self.snapshot_delete_subscriber = rospy.Subscriber('delete_snapshot', std_msgs.msg.Int32, self.delete_snapshot_callback)
        self.pointloud_subscriber = rospy.Subscriber(cloud_topic_name, PointCloud2, self.pointcloud_callback)

        self.last_pointcloud = []

        self.cloud_menu_handler = MenuHandler()
        self.cloud_menu_handler.insert("snapshot_server_menu")
        self.cloud_menu_handler.insert("Add Snapshot", callback= self.snapshot_callback )
        self.cloud_menu = dict()
        self.interactive_marker_server = InteractiveMarkerServer("snapshot_int_marker_server")
        self.control_marker = self.make_control_marker()
        self.interactive_marker_server.insert(self.control_marker, self.feedback)
        self.cloud_menu_handler.apply(self.interactive_marker_server, "snapshot_int_marker")
        self.interactive_marker_server.applyChanges()
        self.snapshot_num = 0
        self.hidden_markers = []


    def feedback(self, msg):
        pass

    def make_control_marker(self):
        
        control_mark = InteractiveMarker()
        control_mark.header.frame_id = "/leftFoot"
        control_mark.scale = 1
        control_mark.name="snapshot_int_marker"
        control_mark.pose.position.z = 2
        
        test_int_mark_cont = InteractiveMarkerControl()
        test_int_mark_cont.always_visible = True
        test_int_mark_cont.interaction_mode = InteractiveMarkerControl.MENU

        test_mark = Marker()
        test_mark.type = Marker.SPHERE
        test_mark.scale.x = test_mark.scale.y = test_mark.scale.z = .5
        test_mark.color.a = 1
        
        test_int_mark_cont.markers.append(test_mark)
        control_mark.controls.append(test_int_mark_cont)
        return control_mark


    def convert_pointcloud2_to_marker(self, pointcloud_msg, trans= numpy.eye(4)):
        print "In convert pointcloud"
        
        point_generator = read_points(pointcloud_msg,None, True)        
        point_list = [point for point in point_generator]
        print "read generator"
        color_list = [[point[3]]*3 + [1] for point in point_list]
        print "made color list"
        point_mat = numpy.mat(point_list).transpose()
        # Make points transformable
        point_mat[3,:] = 1
        point_mat = trans*point_mat
        point_marker = Marker()
        point_marker.type= Marker.POINTS
        point_marker.scale.x = point_marker.scale.y = point_marker.scale.z = .01
        point_marker.colors = [std_msgs.msg.ColorRGBA(*color) for color in color_list]
        print "finished setting color array"
        point_marker.points = [geometry_msgs.msg.Point(*point[0,:3].tolist()[0] ) for point in point_mat.transpose()]
        print "finished setting up point array"
        point_marker.header.frame_id = self.stable_frame
        return point_marker
        


    def pointcloud_callback(self, msg):
        self.last_pointcloud =  msg
        self.tf_listener.waitForTransform(self.stable_frame, msg.header.frame_id, rospy.Time(), rospy.Duration(4.0))
        self.tf_listener.waitForTransform(self.stable_frame, msg.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
        self.last_pointcloud_tran = self.tf_listener.asMatrix(self.stable_frame, msg.header)

    def snapshot_callback(self, msg):
        self.snapshot()
    
    def snapshot(self):
        if not self.last_pointcloud:
            print 'No Pointcloud found'
            ipdb.set_trace()
            return
        marker = self.convert_pointcloud2_to_marker(self.last_pointcloud, self.last_pointcloud_tran)
        self.add_pointcloud_to_menu(self.snapshot_num)
        print "made point cloud marker"
        self.last_pointcloud = []
        self.control_marker.controls[0].markers.append(marker)        
        
        marker.text="Snapshot %d"%(self.snapshot_num)
        self.snapshot_num += 1
        self.reinitialize_server()

    def remove_marker(self, marker_name):        
        found_markers = []
        found_marker_inds = []
        for i in xrange(len(self.control_marker.controls[0].markers)):
            marker = self.control_marker.controls[0].markers[i]
            if marker.text==marker_name:
                found_markers.append(marker)
                found_marker_inds.append(i)
        found_marker_inds.reverse()
        for i in found_marker_inds:
            self.control_marker.controls[0].markers.pop(i)
        return found_markers
        

    def reinitialize_server(self):
        self.interactive_marker_server.clear()
        
        self.interactive_marker_server.insert(self.control_marker, self.feedback)
        self.cloud_menu_handler.apply(self.interactive_marker_server, "snapshot_int_marker")
        self.interactive_marker_server.applyChanges()
        
    def delete_snapshot_callback(self, snapshot_ind_msg):
        self.delete_snapshot(snapshot_ind_msg.data)

    def delete_snapshot(self, snapshot_ind):
        pass
        
    def update(self):
        self.interactive_marker_server.applyChanges()

if __name__ == '__main__':
    
    rospy.init_node('stable_pointcloud_snapshotter')
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()

    loop = rospy.Rate(5)
    snapper = StablePointcloudSnapshotter('/leftFoot', 1, '/rgbd_longrange/depth/points_xyzrgb', tf_listener, tf_broadcaster)
    #    IPython.embed()
    while not rospy.is_shutdown():
        snapper.update()
        
        loop.sleep()
