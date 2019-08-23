#!/usr/bin/env python
import math
import rospy
from clever import srv
from mavros_msgs.srv import CommandBool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_multiply
import tf2_ros

rospy.init_node('cow_despasturer_helper')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)
aruco_frame = 'aruco_0'
frame = 'map'

destination_pose = rospy.Publisher('~/destination_pose', PoseStamped, queue_size=40)
pose = PoseStamped()
pose.header.frame_id = "map"
trans = TransformStamped()
trans2 = TransformStamped()
trans3 = TransformStamped()
#trans.header.frame_id = "map"
broadcaster = tf2_ros.TransformBroadcaster()
static_bloadcaster = tf2_ros.StaticTransformBroadcaster()

while not rospy.is_shutdown():
    try:
        trans = tfBuffer.lookup_transform(frame, aruco_frame, rospy.Time(), timeout=rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue
    #pose.pose.position = trans.transform.translation
    roll, pitch, yaw = euler_from_quaternion([trans.transform.rotation.x,
                                                trans.transform.rotation.y,
                                                trans.transform.rotation.z,
                                                trans.transform.rotation.w])
    trans.transform.rotation = Quaternion(*quaternion_from_euler(0,0,yaw+math.pi/2))
    trans.child_frame_id = "destination_frame_1"
    #trans.header.stamp = rospy.Time.now()
    trans2.transform.translation = Point(-1, 0, 0)
    trans2.transform.rotation = Quaternion(0, 0, 0, 1)
    trans2.header.frame_id = trans.child_frame_id
    trans2.child_frame_id = "destination_frame_2"
    trans2.header.stamp = trans.header.stamp
    #print (trans)
    #destination_pose.publish(pose)
    broadcaster.sendTransform(trans)
    static_bloadcaster.sendTransform(trans2)
    try:
        trans3 = tfBuffer.lookup_transform(frame, "destination_frame_2", rospy.Time(), timeout=rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue
    pose.pose.position = trans3.transform.translation
    pose.pose.orientation = trans3.transform.rotation
    pose.header.stamp = rospy.Time.now()
    destination_pose.publish(pose)

