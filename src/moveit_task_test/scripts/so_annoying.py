import tf
import rospy
import geometry_msgs.msg
import math

def check_orientation_alignment(aruco_marker_frame, wrist_3_link, alignment_threshold=0.01):
    listener = tf.TransformListener()
    
    try:
        listener.waitForTransform(wrist_3_link, aruco_marker_frame, rospy.Time(), rospy.Duration(4.0))
        
        # Get the transform between the end-effector and the marker
        (trans, rot) = listener.lookupTransform(wrist_3_link, aruco_marker_frame, rospy.Time(0))
        
        # Calculate the relative quaternion
        relative_quaternion = rot
        
        # Identity quaternion represents no rotation (aligned)
        identity_quaternion = [0, 0, 0, 1]  # Equivalent to [roll=0, pitch=0, yaw=0]
        
        # Calculate the angle between the quaternions
        angle = 2 * math.acos(abs(tf.transformations.quaternion_dot(relative_quaternion, identity_quaternion)))
        
        rospy.loginfo(f"Angle between the quaternions: {math.degrees(angle)} degrees")
        
        if angle < alignment_threshold:
            rospy.loginfo("The robot's end-effector is aligned with the marker.")
            return True
        else:
            rospy.loginfo("The robot's end-effector is NOT aligned with the marker.")
            return False
        
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF Exception: {}".format(e))
        return False

# Example usage
if __name__ == '__main__':
    rospy.init_node('check_orientation')
    if check_orientation_alignment('aruco_marker_frame', 'wrist_3_link'):
        rospy.loginfo("Aligned!")
    else:
        rospy.loginfo("Not aligned.")
