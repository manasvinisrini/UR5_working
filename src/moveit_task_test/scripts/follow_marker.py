#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker
import math
import time

class MoveToMarkerWithOMPL:
    """Move the robot to an ArUco marker using OMPL planner, keeping the orientation constant."""

    def __init__(self):
        rospy.init_node('move_to_marker_with_ompl', anonymous=True)
        self.rate = rospy.Rate(10)  # ROS Rate at 10Hz

        # Initialize MoveIt! commander and RobotCommander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator')
        self.group.set_max_acceleration_scaling_factor(1.0)
        self.group.set_max_velocity_scaling_factor(1.0)  # Maximum velocity

        # Set the planner ID to OMPL's RRTConnect
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_planning_time(5)  # Reduced from 10s to 5s
        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_pose_callback)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_pose = None
        self.transformed_pose = None
        self.x_offset = 0.220 # Adjust this value as needed
        self.y_offset = 0.00   # Adjust this value as needed
        self.z_offset = 0.00   # Adjust this value as needed
        # Home joint positions
        self.home_joint_positions = [-0.027286354695455373, -0.269754711781637, -1.7435172239886683, -1.1397526899920862, 1.6361838579177856, -0.05274373689760381]

        # Fallback joint positions (define this based on your robot setup)
        self.fallback_joint_positions = [-0.12354165712465459,-0.6483519713031214,-1.7097061316119593, -0.8243325392352503,  1.7764321565628052, -0.019865338002340138]  # Adjust these values accordingly

        # Set fallback time (e.g., 20 seconds)
        self.fallback_time = 15 # seconds

        # Move the robot to the home position at startup
        self.move_to_home_position()

    def marker_pose_callback(self, msg):
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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transform error: {}".format(e))

    def move_to_pose(self, target_pose):
        # Adjust the pose with the specified offsets
        target_pose.position.x += self.x_offset
        target_pose.position.y += self.y_offset
        target_pose.position.z += self.z_offset

        # Set the orientation to the current orientation of the robot
        current_pose = self.group.get_current_pose().pose
        target_pose.orientation = current_pose.orientation

        # Calculate the Euclidean distance between the current and target pose
        distance = math.sqrt(
            (current_pose.position.x - target_pose.position.x) ** 2 +
            (current_pose.position.y - target_pose.position.y) ** 2 +
            (current_pose.position.z - target_pose.position.z) ** 2
        )

        # If the distance is small, try to move incrementally
        if distance < 0.30:  # Adjust the threshold as needed
            rospy.loginfo("Attempting incremental movement...")
            if not self.move_incrementally(current_pose, target_pose):
                rospy.logwarn("Failed to execute incremental movement. Moving to fallback position.")
                self.move_to_fallback_position()
        else:
            # Set goal tolerances
            self.group.set_goal_position_tolerance(0.01)
            self.group.set_goal_orientation_tolerance(0.05)  # Slightly relaxed for better performance

            # Set the target pose
            self.group.set_pose_target(target_pose)

            # Start the timer
            start_time = time.time()

            while True:
                # Plan the motion using OMPL
                plan = self.group.plan()

                # Check if the plan is valid
                if plan and plan[0]:
                    rospy.loginfo("Executing plan...")
                    self.group.execute(plan[1], wait=True)
                    rospy.loginfo("Plan executed successfully.")
                    break
                else:
                    rospy.logwarn("Failed to create a valid plan. Retrying...")

                # Check if fallback time has been exceeded
                if time.time() - start_time > self.fallback_time:
                    rospy.logwarn("Planning time exceeded. Moving to fallback position.")
                    self.move_to_fallback_position()
                    break

    def move_incrementally(self, current_pose, target_pose):
        waypoints = [current_pose, target_pose]
        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)  # Smaller steps
        if fraction > 0.9:  # If the path is mostly valid
            self.group.execute(plan, wait=True)
            rospy.loginfo("Incremental movement executed successfully.")
            return True
        else:
            return False

    def move_to_home_position(self):
        rospy.loginfo("Moving to home position...")
        self.group.set_joint_value_target(self.home_joint_positions)
        plan = self.group.plan()
        if plan and plan[0]:
            self.group.execute(plan[1], wait=True)
            rospy.loginfo("Returned to home position.")
        else:
            rospy.logwarn("Failed to return to home position.")

    def move_to_fallback_position(self):
        rospy.loginfo("Moving to fallback position...")
        self.group.set_joint_value_target(self.fallback_joint_positions)
        plan = self.group.plan()
        if plan and plan[0]:
            self.group.execute(plan[1], wait=True)
            rospy.loginfo("Moved to fallback position successfully.")
        else:
            rospy.logwarn("Failed to move to fallback position.")

    def run(self):
        while not rospy.is_shutdown():
            if self.transformed_pose:
                self.move_to_pose(self.transformed_pose.pose)
            self.rate.sleep()

    def publish_marker(self, pose):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        move_to_marker = MoveToMarkerWithOMPL()
        move_to_marker.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
