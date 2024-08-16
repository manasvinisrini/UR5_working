import rospy
import moveit_commander
import sys
# Initialize the MoveIt! commander and ROS node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('get_current_position', anonymous=True)

# Initialize the MoveGroupCommander for your robot's manipulator
group = moveit_commander.MoveGroupCommander('manipulator')

# Get the current pose of the end-effector
current_pose = group.get_current_pose().pose

# Print the current position and orientation
print("Current Position:")
print(f"x: {current_pose.position.x}")
print(f"y: {current_pose.position.y}")
print(f"z: {current_pose.position.z}")

print("Current Orientation:")
print(f"x: {current_pose.orientation.x}")
print(f"y: {current_pose.orientation.y}")
print(f"z: {current_pose.orientation.z}")
print(f"w: {current_pose.orientation.w}")

# Shut down MoveIt! commander
moveit_commander.roscpp_shutdown()
