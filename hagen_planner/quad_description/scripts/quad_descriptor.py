#!/usr/bin/env python
import roslib
roslib.load_manifest('quad_description')
import rospy
import geometry_msgs
from nav_msgs.msg import Odometry
import math
import rospy
import tf
import tf2_ros


def handle_odometry(msg):
    orientation_q = msg.pose.pose.orientation
    euler_drone = tf.transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
    roll_drone = euler_drone[0]
    pitch_drone = euler_drone[1]
    yaw_drone = euler_drone[2]
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     # roll, pitch and yaw angles
                     # tf.transformations.quaternion_from_euler(math.pi*gimbal_ang.x/180.0, math.pi*gimbal_ang.y/180.0, math.pi*gimbal_ang.z/180.0),
                     tf.transformations.quaternion_from_euler(roll_drone, pitch_drone, yaw_drone),
                     rospy.Time.now(),
                     "base_link",
                     "map")



if __name__ == '__main__':
    rospy.init_node('quad_descriptor')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/mavros/local_position/odom', Odometry, handle_odometry)
    rospy.spin()
