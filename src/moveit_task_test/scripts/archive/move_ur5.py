#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker
# import robotiq_gripper  # Make sure this module is installed and accessible

class MoveToMarker:
    """Move the robot to an ArUco marker, control the gripper, and return to home position."""

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

        # Initialize Robotiq gripper
        # self.gripper = robotiq_gripper.RobotiqGripper()
        # self.gripper_ip = "192.168.12.165"
        # self.connect_gripper()

        rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_pose_callback)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_pose = None
        self.transformed_pose = None

        # Offsets for the gripper position
        self.x_offset = 0.00# Adjust this value as needed
        self.y_offset = 0.00 # Adjust this value as needed
        self.z_offset = 0.00 # Adjust this value as needed

        # Define home position joint values for UR5
        self.home_joint_values = [-3.751, -0.995, 1.344, 2.721, 4.481, -0.051]

        # Task completion flag
        self.task_completed = False

    def connect_gripper(self):
        print("Creating gripper...")
        self.gripper.connect(self.gripper_ip, 63352)
        # Uncomment the following line if you need to activate the gripper
        # print("Activating gripper...")
        # self.gripper.activate()

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
            self.move_to_pose(self.transformed_pose.pose)
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

    def move_to_pose(self, pose):
        # Adjust the pose with the specified offsets
        pose.position.x += self.x_offset
        pose.position.y += self.y_offset
        pose.position.z += self.z_offset

        # Set the orientation to the current orientation of the robot
        current_pose = self.group.get_current_pose().pose
        # pose.orientation = current_pose.orientation

        # Log the target pose
        rospy.loginfo("Moving to pose: {}".format(pose))

        # Set the target pose for the robot
        self.group.set_pose_target(pose)

        # Increase planning attempts and time
        self.group.set_planning_time(20)
        self.group.set_num_planning_attempts(50)

        # Plan and execute the motion
        plan = self.group.go(wait=True)

        # Stop and clear pose targets after planning
        self.group.stop()
        self.group.clear_pose_targets()

        # Log the result of the plan
        if plan:
            rospy.loginfo("Move to marker successful")
            # Close the gripper after reaching the marker
            # Go to home position
            self.task_completed = True
        else:
            rospy.logwarn("Move to marker failed")
            current_pose = self.group.get_current_pose().pose
            rospy.logwarn("Current Robot Pose: {}".format(current_pose))

    def go_to_home_position(self):
        self.group.go(self.home_joint_values, wait=True)
        self.group.stop()
        rospy.loginfo("Returned to home position")
        # Open the gripper after reaching home position

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
