#!/usr/bin/env python3

import rospy
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CorrectMarkerPoseNode:
    def __init__(self):
        rospy.init_node('correct_marker_pose_node', anonymous=True)

        # Subscriber to get the marker pose
        self.marker_pose_sub = rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_pose_callback)
        
        # Publisher to publish the corrected marker pose
        self.corrected_pose_pub = rospy.Publisher('/corrected_marker_pose', geometry_msgs.msg.PoseStamped, queue_size=10)
        
        rospy.loginfo("CorrectMarkerPoseNode initialized, waiting for marker pose...")

    def marker_pose_callback(self, pose_msg):
        rospy.loginfo("Received marker pose: {}".format(pose_msg))
        corrected_pose = self.correct_orientation(pose_msg)
        self.corrected_pose_pub.publish(corrected_pose)
        rospy.loginfo("Published corrected marker pose: {}".format(corrected_pose))

    def correct_orientation(self, pose_msg):
        pose = pose_msg.pose
        # Convert quaternion to Euler angles
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        rospy.loginfo("Original orientation (rpy): {}, {}, {}".format(roll, pitch, yaw))

        # Assuming the marker should be flat on the table and pointing upwards
        roll = 0
        pitch = -1.57  # 90 degrees rotation around x-axis to point upwards
        yaw = 0

        # Convert back to quaternion
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        rospy.loginfo("Corrected orientation (quaternion): {}, {}, {}, {}".format(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

        pose_msg.pose = pose
        return pose_msg

if __name__ == '__main__':
    try:
        correct_marker_pose = CorrectMarkerPoseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
