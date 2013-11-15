#!/usr/bin/env python

import roslib
import rospy


from moveable_button_marker import *


def print_pose(marker):
    print "Marker pose"
    print marker.marker_pose_stamped

if __name__ == '__main__':
    rospy.init_node('button_marker_test')
    button_marker = MoveableButtonMarker("button_marker")
    button_marker.update_menu("output pose", print_pose, [button_marker])

    button_marker.populate_menu()                
    loop = rospy.Rate(5)
    server = InteractiveMarkerServer(button_marker.marker_namespace)
    button_marker.create_marker()
    while not rospy.is_shutdown():
        button_marker.update()
        server.clear()
        server.insert(button_marker.int_marker, button_marker.int_marker_feedback_cb)
        button_marker.menu_handler.apply(server,"button_marker")
        button_marker.update()
        server.applyChanges()
        
        loop.sleep()
        
    
    
