#!/usr/bin/env python

import rospy
import tf
from tf import TransformBroadcaster
from rospy import Time
from std_msgs.msg import Float32MultiArray
import numpy as np
from visualization_msgs.msg import Marker
from robot_module_msgs.srv import ToolChanger
import time
import collections
import os
import subprocess
import signal


receivedCoords = [0.0, 0.0, 0.0]
receivedEulers = [0.0, 0.0, 0.0]
scale = 0.01
toolZOffset = 0
b = TransformBroadcaster()
# default tool @panda_link8
tool_name = 'screwdriver'
group_name = 'screwdriver'
attach_to = 'panda_2/panda_link8'
hold = True
infos = []
change_trigger = 0
child = None
markerPub = None

def changer(tool_args):
    # rosservice call /tool_change ...
    global tool_name
    global group_name
    global attach_to
    global infos
    global change_trigger
    global receivedCoords
    global receivedEulers
    global hold

    # no changes if inputs are empty
    if tool_args.toolname == '':
        pass
    else:
        tool_name = tool_args.toolname

    if tool_args.groupname == '':
        pass
    else:
        group_name = tool_args.groupname

    if tool_args.transform_to == '':
        pass
    else:
        attach_to = tool_args.transform_to
    
    if tool_args.hold == '':
        pass
    else:
        hold = tool_args.hold
    
    change_trigger = 1

    # delete service reply list to prevent stacking
    del infos[:]
    infos.append(True)
    infos.append('No Error')
    return infos


def tool_change():    
    marker = Marker()
    global tool_name
    global group_name
    global attach_to
    global hold
    global change_trigger
    global child
    global receivedCoords
    global receivedEulers
    global markerPub

    try:         
    # stop static node when tool is assigned to robot
        if hold:
            if change_trigger == 1:                     
                change_trigger = 0
                s_proc()
            translation = (receivedCoords[0], receivedCoords[1], receivedCoords[2] + toolZOffset * scale)       
            rotation = receivedEulers
            rotation = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
            b.sendTransform(translation, rotation, Time.now(), str(tool_name), str(attach_to))
            
            # Prepare Marker message
            marker.header.stamp = Time.now()
            marker.header.frame_id = str(tool_name)
            marker.ns = str(group_name)
            marker.id = 0
            marker.type = 9
            marker.pose.position.z = 0
            marker.pose.orientation.z = 1.5
            marker.scale.z = 0.05
            marker.color.b = 1.0
            marker.color.a = 1.0
            markerPub.publish(marker)
        else:
            pass

    except KeyboardInterrupt:
        rospy.loginfo("Exiting service")     

def s_proc():
    global tool_name
    global child
    # launch static nodes
    child.send_signal(signal.SIGINT)
    
    if tool_name == 'screwdriver':
        child = subprocess.Popen(["roslaunch","reconcycle_simulation", "static_tools.launch", "selector:=1"])
    elif tool_name == 'vacuumgripper':
        child = subprocess.Popen(["roslaunch","reconcycle_simulation", "static_tools.launch", "selector:=2"])
    elif tool_name == 'parallelgripper':
        child = subprocess.Popen(["roslaunch","reconcycle_simulation", "static_tools.launch", "selector:=3"])
    

if __name__ == '__main__':
    # initialize node
    rospy.init_node("change_rviz_tool_server")
    # set while loop rate
    r = rospy.Rate(100)
    # print service call node
    rospy.loginfo("Created /tool_change service node!")
    # start service
    service = rospy.Service("/tool_change", ToolChanger, changer)
    rospy.loginfo("Service server has been started!")
    # start marker publisher
    markerPub = rospy.Publisher('/visualization_marker', Marker, queue_size = 10)
    # initialize tool positions
    child = subprocess.Popen(["roslaunch","reconcycle_simulation", "static_tools.launch", "selector:=1"])
    # start tool_chagne function in loop with rospy.Rate 100
    while not rospy.is_shutdown():
        tool_change()
        r.sleep()