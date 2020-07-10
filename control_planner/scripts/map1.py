#!/usr/bin/env python
#-*- coding utf8-*-
import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class simple kinematics:
    def __init__(self):
        rospy.init_node('simple kinematics',anonymous=True)

        rospy.Subscriber("/sensors/core",VescStateStamped, self.status_callback)
        rospy.Subscriber("/sensors/servo_position_command",Float64, self.servo_command_callback)

        self.is_speed=False
        self.is_servo=False
        self.servo_msg=Float64()

        self.odom_pub=rospy.Publisher('/odom',Odometry,queue_size=1)
        self.odom_msg=Odometry()
        self.odom_msg.header.frmae_id='/odom'

        self.rpm_gain=4614
        self.steering_angle_to_servo_gain=1.2135
        self