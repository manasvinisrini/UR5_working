#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

class MoveRobotNode():
    """MoveRobotNode"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_robot_node", anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

    def go_to_joint_state(self):
        move_group = self.move_group

        # Get the current joint values
        current_joint_state = move_group.get_current_joint_values()
        rospy.loginfo(f"Current Joint State: {current_joint_state}")

        # Define home position joint values for UR5
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] =   0.2601296007633209 #shoulder_pan_joint
        joint_goal[1] =  -2.0604880491839808 #shoulder_lift_joint
        joint_goal[2] = -1.2188199202166956 #elbow_joint
        joint_goal[3] =  -0.5688918272601526#wrist_1
        joint_goal[4] =  1.3435795307159424 #wrist_2
        joint_goal[5] = 0.18534615635871887 

        # Move to the joint goal
        success = move_group.go(joint_goal, wait=True)
        rospy.loginfo(f"Motion Success: {success}")

        # Stop the robot
        move_group.stop()

        # Verify the final joint state
        final_joint_state = move_group.get_current_joint_values()
        rospy.loginfo(f"Final Joint State: {final_joint_state}")

if __name__ == "__main__":
    try:
        robot_control = MoveRobotNode()
        robot_control.go_to_joint_state()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
