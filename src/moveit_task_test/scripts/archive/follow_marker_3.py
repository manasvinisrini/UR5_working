#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker

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
        self.group.set_max_velocity_scaling_factor(1.0)  # Increased for faster execution

        # Set the planner ID to OMPL's RRTConnect
        self.group.set_planner_id("RRTConnectkConfigDefault")
        self.group.set_planning_time(3)  # Reduced from 10s to 3s  # Adjusted goal bias for faster convergence

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_pose_callback)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_pose = None
        self.transformed_pose = None

        # Offsets for the gripper position
        self.x_offset = 0.200  # Adjust this value as needed
        self.y_offset = 0.00   # Adjust this value as needed
        self.z_offset = 0.00   # Adjust this value as needed

        # Distance threshold for stopping
        self.stop_threshold = 0.05  # 5 cm from target

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

        # Set the orientation to the current orientation of the robot (to maintain orientation)
        current_pose = self.group.get_current_pose().pose
        target_pose.orientation = current_pose.orientation

        # Set goal tolerances
        self.group.set_goal_position_tolerance(0.01)
        self.group.set_goal_orientation_tolerance(0.1)  # Slightly relaxed for better performance

        # Set the target pose
        self.group.set_pose_target(target_pose)

        # Plan the motion using OMPL
        plan = self.group.plan()

        # Simplify the plan if needed
        simplified_plan = self.group.simplify_plan(plan)

        # Execute the motion if a valid plan is returned
        if simplified_plan[0]:
            rospy.loginfo("Executing plan...")
            self.group.execute(simplified_plan[1], wait=True)
            rospy.loginfo("Plan executed successfully.")
        else:
            rospy.logwarn("Failed to create a valid plan.")

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

    def run(self):
        while not rospy.is_shutdown():
            if self.transformed_pose:
                self.move_to_pose(self.transformed_pose.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        move_to_marker = MoveToMarkerWithOMPL()
        move_to_marker.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
