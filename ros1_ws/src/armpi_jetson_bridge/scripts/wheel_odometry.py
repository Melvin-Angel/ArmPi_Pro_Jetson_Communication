#!/usr/bin/env python3
"""Publish calibrated wheel odometry without broadcasting TF."""
import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
# Catkin devel-space wrappers execute the source script from another directory.
import os
import sys
import rospkg
sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('armpi_jetson_bridge'), 'scripts'))
from encoder_reader import read_counts
from wheel_math import WheelOdometry


def main():
    rospy.init_node('wheel_odometry')
    if not rospy.get_param('~calibration_verified', False):
        rospy.logfatal('Set calibration_verified only after hardware calibration')
        return
    order = rospy.get_param('~channel_order', [])
    if sorted(order) != [1, 2, 3, 4]:
        raise ValueError('channel_order must map FL, FR, RL, RR to channels 1..4')
    odom = WheelOdometry(float(rospy.get_param('~wheel_radius_m')),
                         float(rospy.get_param('~ticks_per_revolution')),
                         float(rospy.get_param('~wheelbase_sum_m')),
                         rospy.get_param('~wheel_signs'),
                         float(rospy.get_param('~max_ticks_per_second')))
    pub = rospy.Publisher('/wheel/odometry', Odometry, queue_size=1)
    joints_pub = rospy.Publisher('/wheel/joint_states', JointState, queue_size=1)
    rate = rospy.Rate(float(rospy.get_param('~rate_hz', 20.0)))
    while not rospy.is_shutdown():
        try:
            counts = read_counts(int(rospy.get_param('~i2c_bus', 1)))
            stamp = rospy.Time.now()
            result = odom.update([counts[i - 1] for i in order], stamp.to_sec())
            if result:
                x, y, yaw, vx, vy, wz = result
                msg = Odometry()
                msg.header.stamp = stamp
                msg.header.frame_id = rospy.get_param('~odom_frame', 'wheel_odom')
                msg.child_frame_id = rospy.get_param('~base_frame', 'base_link')
                msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z = vx, vy, wz
                # Conservative placeholders; replace with measurements before fusion.
                msg.pose.covariance = [1000.0 if i in (14, 21, 28) else 0.1 if i in (0, 7, 35) else 0.0 for i in range(36)]
                msg.twist.covariance = list(msg.pose.covariance)
                pub.publish(msg)
            # Encoder-relative wheel angles; zero is defined at node startup.
            joints = JointState()
            joints.header.stamp = stamp
            joints.name = ['wheel_lf_Joint', 'wheel_rf_Joint', 'wheel_lb_Joint', 'wheel_rb_Joint']
            joints.position = list(odom.angles)
            joints.velocity = list(odom.angular_velocities)
            joints_pub.publish(joints)
        except (OSError, ValueError) as exc:
            rospy.logerr('Wheel odometry stopped: %s; restart and relocalize', exc)
            return
        rate.sleep()


if __name__ == '__main__':
    main()
