import sys
import rospy
import moveit_commander
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker
import math
import time  # For timing the execution

class MoveToMarker:
    """Move the robot to an ArUco marker, inching towards the target to position for picking it up."""

    def __init__(self):
        rospy.init_node('move_to_marker', anonymous=True)
        self.rate = rospy.Rate(10)  # ROS Rate at 10Hz

        # Initialize MoveIt! commander and RobotCommander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('manipulator')
        self.group.set_max_acceleration_scaling_factor(1.0)
        self.group.set_max_velocity_scaling_factor(1.0)

        self.tf_listener = tf.TransformListener()

        rospy.Subscriber('/aruco_single/pose', geometry_msgs.msg.PoseStamped, self.marker_pose_callback)

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        self.marker_pose = None
        self.transformed_pose = None

        # Offsets for the gripper position
        self.x_offset = 0.215  # Adjust this value as needed
        self.y_offset = 0.00  # Adjust this value as needed
        self.z_offset = 0.00  # Adjust this value as needed

        # Define home position joint values for UR5
        self.home_joint_values = [-3.751, -0.995, 1.344, 2.721, 4.481, -0.051]

        # Incremental step size for inching
        self.step_size = 0.5  # 10 cm per step

        # Distance threshold for stopping
        self.stop_threshold = 0.05  # 5 cm from target

        # Variables for metrics
        self.current_pose = None
        self.target_pose = None
        self.path_length = 0.0
        self.execution_time = 0.0
        self.success_rate = 0.0

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

    def move_to_pose(self, target_pose):
        # Adjust the pose with the specified offsets
        target_pose.position.x += self.x_offset
        target_pose.position.y += self.y_offset
        target_pose.position.z += self.z_offset

        # Store current and target pose
        self.current_pose = self.group.get_current_pose().pose
        self.target_pose = target_pose

        # Set the orientation to the current orientation of the robot
        target_pose.orientation = self.current_pose.orientation

        # Plan the Cartesian path to the target pose
        waypoints = [self.current_pose, target_pose]

        # Start timing the path execution
        start_time = time.time()

        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.05,        # eef_step, resolution of 1 cm
            2)         # jump_threshold

        # Calculate execution time
        self.execution_time = time.time() - start_time

        # Calculate path length
        self.path_length = self.calculate_path_length(waypoints)

        # Update success rate (fraction of the path that was planned successfully)
        self.success_rate = fraction

        # Execute the motion
        self.group.execute(plan, wait=True)

        rospy.loginfo("Reached the desired position or stopped inching.")

    def calculate_path_length(self, waypoints):
        length = 0.0
        for i in range(1, len(waypoints)):
            pose1 = waypoints[i-1]
            pose2 = waypoints[i]
            length += math.sqrt(
                (pose2.position.x - pose1.position.x)**2 +
                (pose2.position.y - pose1.position.y)**2 +
                (pose2.position.z - pose1.position.z)**2
            )
        return length

    def go_to_home_position(self):
        self.group.go(self.home_joint_values, wait=True)
        self.group.stop()
        rospy.loginfo("Returned to home position")

    def run(self):
        while not rospy.is_shutdown():
            if self.transformed_pose:
                self.move_to_pose(self.transformed_pose.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        move_to_marker = MoveToMarker()
        move_to_marker.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
