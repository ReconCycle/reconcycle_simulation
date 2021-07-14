#!/usr/bin/python

import rospy
from rospy import Time
import numpy as np
from robot_module_msgs.srv import rviz_tools
# Because of transformations
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import time
import argparse
import sys


class FixedTFBroadcaster:
    def __init__(self, tool, frame):
        self.tool = tool
        self.frame = frame

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    def tf_loop(self, tool, frame):
        self.tool = tool
        self.frame = frame
        while not rospy.is_shutdown():
            # Run this loop at about 100Hz
            rospy.sleep(0.01)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = str(self.frame)
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = str(self.tool)
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = tf2_msgs.msg.TFMessage([t])
            self.pub_tf.publish(tfm)

    def service_call(self, tool_data):
        if self.frame == tool_data.frame:
            rospy.loginfo("Tool is already on this position!")
            return False, "Tool is already on this position!"
        else:
            self.frame = tool_data.frame
            return True, "Tool moved"


if __name__ == '__main__':
    rospy.init_node('tool_viz_node', anonymous=True)    
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool', default=None, type=str)
    parser.add_argument('--frame', default=None, type=str)

    try:
        args, unknown = parser.parse_known_args(rospy.myargv()[:])
        rospy.loginfo(args)
        tfb = FixedTFBroadcaster("", "")
        rospy.loginfo("Tool change service has started...")
        service_path = "/change_tool_frame/" + str(args.tool)

        service = rospy.Service(service_path, rviz_tools, tfb.service_call)
        time.sleep(1)
        tfb.tf_loop(args.tool, args.frame)
    except:
        rospy.loginfo("Error")

    rospy.spin()

