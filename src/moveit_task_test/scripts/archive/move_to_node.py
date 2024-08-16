#!/usr/bin/env python3
import sys
print(sys.path)
import rospy
import moveit_commander
from robotiq_gripper import RobotiqGripper

class MoveRobotNode():
    """MoveRobotNode"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_robot_node", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Initialize the gripper
        self.gripper = RobotiqGripper()
        self.gripper.connect("192.168.1.10", 63352)  # Replace with your gripper's IP and port
        self.gripper.activate()

    def control_gripper(self):
        # Check if the gripper is open, and close it if it is
        if self.gripper.is_open():
            rospy.loginfo("Gripper is open. Closing it now.")
            self.gripper.move_and_wait_for_pos(self.gripper.get_closed_position(), speed=100, force=100)
        else:
            rospy.loginfo("Gripper is already closed.")

    def open_gripper(self):
        # Open the gripper
        rospy.loginfo("Opening the gripper.")
        self.gripper.move_and_wait_for_pos(self.gripper.get_open_position(), speed=100, force=100)

    def go_to_joint_state(self):
        move_group = self.move_group

        # Get the current joint values
        current_joint_state = move_group.get_current_joint_values()
        rospy.loginfo(f"Current Joint State: {current_joint_state}")

        # Define home position joint values for UR5
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0.4766159951686859 #shoulder_pan_joint
        joint_goal[1] = -1.965820614491598 #shoulder_lift_joint
        joint_goal[2] = -1.4923704306231897 #elbow_joint
        joint_goal[3] = -0.0902331511126917 #wrist_1
        joint_goal[4] = 1.1299151182174683 #wrist_2
        joint_goal[5] = 0.07848618924617767 #wrist_3

        # Move to the joint goal
        success = move_group.go(joint_goal, wait=True)
        rospy.loginfo(f"Motion Success: {success}")

        # Stop the robot
        move_group.stop()

        # Verify the final joint state
        final_joint_state = move_group.get_current_joint_values()
        rospy.loginfo(f"Final Joint State: {final_joint_state}")

    def execute(self):
        try:
            self.control_gripper()
            self.go_to_joint_state()
            self.open_gripper()
        finally:
            # Disconnect the gripper
            self.gripper.disconnect()
            moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    try:
        robot_control = MoveRobotNode()
        robot_control.execute()
    except rospy.ROSInterruptException:
        pass
