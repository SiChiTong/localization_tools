#!/usr/bin/env python

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import roslib
import rospy
from transformation_helper import *
import tf
import IPython


class StablePointcloudSnapshotter(object):
    class StableCloud(object):
        def __init__(self, stable_frame, shot_number, tf_listener, tf_broadcaster):
            self.stable_frame = stable_frame
            self.pointcloud = []
            self.saved_transform = []
            self.snapshot_frame = 'snapshot_%d'%(shot_number)
            self.cloud_publisher = []
            self.cloud_name = "pointcloud_%d"%(shot_number)
            self.tf_listener = tf_listener
            self.tf_broadcaster = tf_broadcaster

        def rebroadcast(self):
            if self.cloud_publisher:
                self.tf_broadcaster.sendTransform(self.saved_transform[0], self.saved_transform[1], rospy.Time.now(),  self.snapshot_frame, self.stable_frame) 
                self.cloud_publisher.publish(self.pointcloud)

            
        def set_cloud(self, pointcloud):
            self.pointcloud = pointcloud
            self.saved_transform = tf_listener.lookupTransform(self.stable_frame, self.pointcloud.header.frame_id, rospy.Time(0))
            self.pointcloud.header.frame_id = self.snapshot_frame
            self.cloud_publisher = rospy.Publisher(self.cloud_name, PointCloud2)
        
        def stop_publisher(self):
            self.cloud_publisher.unregister()
            self.cloud_publisher = []

        


    def __init__(self, stable_frame = 'leftFoot', max_clouds = 1, cloud_topic_name = '', tf_listener = [], tf_broadcaster = []):
        self.tf_listener = tf_listener
        self.tf_broadcaster = tf_broadcaster

        self.stable_frame = stable_frame
        self.max_clouds = max_clouds
        self.snapshot_request_subscriber = rospy.Subscriber('snapshot_cloud', std_msgs.msg.Empty, self.snapshot_callback)
        self.snapshot_delete_subscriber = rospy.Subscriber('delete_snapshot', std_msgs.msg.Int32, self.delete_snapshot_callback)
        self.snapshot_list = [self.StableCloud(stable_frame, cloud_ind, self.tf_listener, self.tf_broadcaster) for cloud_ind in xrange(max_clouds)]
        self.cloud_topic_name = cloud_topic_name


    def snapshot_callback(self, msg):
        self.snapshot()
    
    def snapshot(self):
        last_unset = None
        for i in xrange(self.max_clouds):
            if self.snapshot_list[i].cloud_publisher:
                continue
            last_unset = i
            break
        
        if last_unset is None:
            self.delete_snapshot(0)
            i = -1
            
        try:
            pointcloud_msg = rospy.wait_for_message(self.cloud_topic_name, PointCloud2, 11)
        except:
            print "failed to get pointcloud on %s"%(self.cloud_topic_name)
            return

        self.snapshot_list[i].set_cloud(pointcloud_msg)


    def delete_snapshot_callback(self, snapshot_ind_msg):
        self.delete_snapshot(snapshot_ind_msg.data)

    def delete_snapshot(self, snapshot_ind):
        if snapshot_ind > self.max_clouds:
            return

        last_moved = snapshot_ind

        for move_ind in xrange(snapshot_ind+1, self.max_clouds):
            
            moving_shot = self.snapshot_list[move_ind]
            if moving_shot.cloud_publisher == []:
                break
            
            last_moved = move_ind
            target_shot = self.snapshot_list[move_ind - 1]
            target_shot.pointcloud = moving_shot.pointcloud
            target_shot.saved_transform = moving_shot.saved_transform
            
        if self.snapshot_list[last_moved].cloud_publisher:
            self.snapshot_list[last_moved].cloud_publisher.unregister()
            self.snapshot_list[last_moved].cloud_publisher = []
        
    def update(self):
        [snapshot.rebroadcast() for snapshot in self.snapshot_list]

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
