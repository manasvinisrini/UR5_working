#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker
import math

class MoveToMarker:
    """Move the robot to an ArUco marker, inching towards the target to position for picking it up."""

    def __init__(self):
        rospy.init_node('move_to_marker', anonymous=True)
        self.rate = rospy.Rate(1)  # ROS Rate at 1Hz

        # Initialize MoveIt! commander and RobotCommander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator')
        self.group.set_max_acceleration_scaling_factor(1)
        self.group.set_max_velocity_scaling_factor(1)

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_pose_callback)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_pose = None
        self.transformed_pose = None

        # Offsets for the gripper position
        self.x_offset = 0.200  # Adjust this value as needed
        self.y_offset = -0.002  # Adjust this value as needed
        self.z_offset = 0.00  # Adjust this value as needed

        # Define home position joint values for UR5
        self.home_joint_values = [-3.751, -0.995, 1.344, 2.721, 4.481, -0.051]

        # Incremental step size for inching
        self.step_size = 0.05  # 5 cm per step

        # Distance threshold for stopping
        self.stop_threshold = 0.10  # 10 cm from target

        # Task completion flag
        self.task_completed = False

    def marker_pose_callback(self, msg):
        if self.task_completed:
            return

        self.marker_pose = msg.pose
        rospy.loginfo(f"Received marker pose: {self.marker_pose}")

        try:
            # Wait for the transform to be available
            self.tf_listener.waitForTransform("base_link", msg.header.frame_id, rospy.Time(), rospy.Duration(4.0))
            # Transform the marker pose to the base frame
            marker_pose_stamped = geometry_msgs.msg.PoseStamped()
            marker_pose_stamped.header = msg.header
            marker_pose_stamped.pose = self.marker_pose

            self.transformed_pose = self.tf_listener.transformPose("base_link", marker_pose_stamped)
            rospy.loginfo(f"Transformed marker pose: {self.transformed_pose.pose}")
            self.publish_marker(self.transformed_pose.pose)
            self.inch_towards_pose(self.transformed_pose.pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: {}".format(e))

    def publish_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker_pose"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def inch_towards_pose(self, target_pose):
        # Adjust the pose with the specified offsets
        target_pose.position.x += self.x_offset
        target_pose.position.y += self.y_offset
        target_pose.position.z += self.z_offset

        # Set the orientation to the current orientation of the robot
        current_pose = self.group.get_current_pose().pose
        # target_pose.orientation = current_pose.orientation

        # Compute the distance to the target
        distance = self.calculate_distance(current_pose, target_pose)
        rospy.loginfo(f"Distance to target: {distance}")

        while distance > self.stop_threshold:
            # Calculate the next incremental step towards the target
            next_pose = self.calculate_next_pose(current_pose, target_pose)

            # Set the target pose for the robot
            self.group.set_pose_target(next_pose)

            # Plan and execute the motion
            plan = self.group.go(wait=True)

            # Stop and clear pose targets after planning
            self.group.stop()
            self.group.clear_pose_targets()

            # Log the result of the plan
            if plan:
                rospy.loginfo("Inching towards marker...")
                current_pose = self.group.get_current_pose().pose
                distance = self.calculate_distance(current_pose, target_pose)
                rospy.loginfo(f"Updated distance to target: {distance}")
            else:
                rospy.logwarn("Move to marker failed, stopping inching")
                break

        rospy.loginfo("Reached the desired position or stopped inching.")
        self.task_completed = True

    def calculate_distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2 +
            (pose1.position.z - pose2.position.z) ** 2
        )

    def calculate_next_pose(self, current_pose, target_pose):
        next_pose = geometry_msgs.msg.Pose()
        distance = self.calculate_distance(current_pose, target_pose)
        step_fraction = self.step_size / distance if distance > 0 else 1.0

        next_pose.position.x = current_pose.position.x + step_fraction * (target_pose.position.x - current_pose.position.x)
        next_pose.position.y = current_pose.position.y + step_fraction * (target_pose.position.y - current_pose.position.y)
        next_pose.position.z = current_pose.position.z + step_fraction * (target_pose.position.z - current_pose.position.z)
        next_pose.orientation = current_pose.orientation  # Keep orientation constant

        return next_pose

    def go_to_home_position(self):
        self.group.go(self.home_joint_values, wait=True)
        self.group.stop()
        rospy.loginfo("Returned to home position")

    def run(self):
        while not rospy.is_shutdown():
            if self.transformed_pose and not self.task_completed:
                self.publish_marker(self.transformed_pose.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        move_to_marker = MoveToMarker()
        move_to_marker.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
