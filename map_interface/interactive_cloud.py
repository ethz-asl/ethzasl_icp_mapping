#!/usr/bin/env python

import rospy
import copy
import tf

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.broadcaster import TransformBroadcaster
from tf import transformations as t
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os
from std_msgs.msg import String as st
import std_msgs

server = None
menu_handler = MenuHandler()
br = None
counter = 0
int_marker = InteractiveMarker()
control = InteractiveMarkerControl()
map_frame = rospy.get_param('/map_frame')
marker_frame = rospy.get_param('/marker_frame')
marker_frame2 = rospy.get_param('/marker_frame2')
world_frame = 'world'
transform_set = False
transform_pub = rospy.Publisher('/T_map_marker', Transform, queue_size=10)


def frameCallback(msg):
    global int_marker, br, map_frame, marker_frame2, world_frame
    time = rospy.Time.now()
    br.sendTransform((int_marker.pose.position.x,
                      int_marker.pose.position.y,
                      int_marker.pose.position.z),
                     (int_marker.pose.orientation.x,
                      int_marker.pose.orientation.y,
                      int_marker.pose.orientation.z,
                      int_marker.pose.orientation.w),
                     time, marker_frame2, map_frame)


def processFeedback(feedback):
    server.applyChanges()


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control

# Marker Creation
def make6DofMarker(fixed, interaction_mode, position, show_6dof=False):
    global int_marker, control, map_frame, marker_frame2
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = map_frame
    int_marker.pose.position = position
    int_marker.scale = 1
    int_marker.pose.orientation.x = 0
    int_marker.pose.orientation.y = 0
    int_marker.pose.orientation.z = 0
    int_marker.pose.orientation.w = 1
    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # Insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if not interaction_mode == InteractiveMarkerControl.NONE:
        control_modes_dict = {
            InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
            InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
            InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D"}
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
            int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(makeBox(int_marker))
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)

def loadMAPCb(feedback):
    global transform_pub, int_marker
    rospy.loginfo("Loading MAP.")

    while True:
        try:
            load_map_proxy = rospy.ServiceProxy('/mapper/load_published_map',
                                                Empty)
            success = load_map_proxy(EmptyRequest())
            if not success:
                print "Loading MAP did not work."
            else:
                print "Loading MAP successful."
                transform = Transform()
                transform.translation = Vector3(int_marker.pose.position.x,int_marker.pose.position.y,int_marker.pose.position.z)
                transform.rotation = Quaternion(int_marker.pose.orientation.x,int_marker.pose.orientation.y,int_marker.pose.orientation.z,int_marker.pose.orientation.w)
                transform_pub.publish(transform)
                break
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e



def initMenu():
    global h_mode_last, map_frame, marker_frame, transform_pub
    h_fifth_entry = menu_handler.insert("Load MAP",
                                        callback=loadMAPCb)


if __name__ == "__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()

    # Create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("interactive_cloud_marker_controls")

    position = Point(0, 0, 0)
    make6DofMarker(False, InteractiveMarkerControl.MOVE_ROTATE_3D, position,
                   True)

    # Add menu to marker.
    initMenu()
    menu_handler.apply(server, int_marker.name)

    server.applyChanges()

    rospy.spin()
