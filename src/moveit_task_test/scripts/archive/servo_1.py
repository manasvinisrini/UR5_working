#!/usr/bin/env python3

import rospy
import threading
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.planning_scene_monitor import PlanningSceneMonitor
from moveit_servo.pose_tracking import PoseTracking
from moveit_servo.servo import Servo

LOGNAME = "pose_tracking_example"

class PoseTrackingExample:
    def __init__(self):
        rospy.init_node(LOGNAME, anonymous=True)
        rospy.loginfo("Initializing Pose Tracking Example")

        # Initialize MoveIt Commander
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("manipulator")

        # Load the planning scene monitor
        self.planning_scene_monitor = PlanningSceneMonitor("robot_description")
        if not self.planning_scene_monitor.getPlanningScene():
            rospy.logerr("Error in setting up the PlanningSceneMonitor.")
            exit(EXIT_FAILURE)

        self.planning_scene_monitor.startSceneMonitor()
        self.planning_scene_monitor.startWorldGeometryMonitor(
            PlanningSceneMonitor.DEFAULT_COLLISION_OBJECT_TOPIC,
            PlanningSceneMonitor.DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
            False
        )
        self.planning_scene_monitor.startStateMonitor()

        # Create the pose tracker
        self.tracker = PoseTracking(rospy.get_param("~move_group_name", "manipulator"), self.planning_scene_monitor)

        # Publisher for sending pose commands
        self.target_pose_pub = rospy.Publisher("target_pose", PoseStamped, queue_size=1, latch=True)

        # Subscribe to the ArUco marker pose topic
        self.aruco_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, self.aruco_pose_callback)

        # Run the pose tracking in a new thread
        self.lin_tol = np.array([0.01, 0.01, 0.01])
        self.rot_tol = 0.1
        self.move_to_pose_thread = threading.Thread(target=self.run_pose_tracking)
        self.move_to_pose_thread.start()

    def aruco_pose_callback(self, msg):
        target_pose = msg

        # Apply an offset to the target pose
        target_pose.pose.position.x += 0.1

        # Publish the target pose
        target_pose.header.stamp = rospy.Time.now()
        self.target_pose_pub.publish(target_pose)

        # Reset the target pose in the tracker
        self.tracker.reset_target_pose()

    def run_pose_tracking(self):
        while not rospy.is_shutdown():
            self.tracker.move_to_pose(self.lin_tol, self.rot_tol, 0.1)  # 0.1 seconds timeout
            rospy.sleep(0.01)  # sleep to reduce CPU usage

    def stop(self):
        # Make sure the tracker is stopped and clean up
        self.tracker.stop_motion()
        self.move_to_pose_thread.join()

if __name__ == '__main__':
    try:
        pose_tracking_example = PoseTrackingExample()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        pose_tracking_example.stop()
