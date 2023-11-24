#!/usr/bin/python

#
# Similar to static_transform_broadcaster, this node constantly publishes
# static odometry information (Odometry msg and tf). This can be used
# with fake_localization to evaluate planning algorithms without running
# an actual robot with odometry or localization
#
# Author: Armin Hornung
# License: BSD

import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point, PoseWithCovarianceStamped


def publishOdom():
    rospy.init_node('fake_odom')
    base_frame_id = rospy.get_param("~base_frame_id", "/odom")
    odom_frame_id = rospy.get_param("~odom_frame_id", "/map")
    publish_frequency = rospy.get_param("~publish_frequency", 10000.0)
    pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped,queue_size=10)
    tf_pub = tf.TransformBroadcaster()

    #TODO: static pose could be made configurable (cmd.line or parameters)
    quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    odom = PoseWithCovarianceStamped()
    odom.header.frame_id = odom_frame_id
    odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(*quat))

    rospy.loginfo("Publishing static odometry from \"%s\" to \"%s\"", odom_frame_id, base_frame_id)
    r = rospy.Rate(publish_frequency)
    while not rospy.is_shutdown():
        odom.header.stamp = rospy.Time.now()
        tf_pub.sendTransform((0, 0, 0), quat,
                        odom.header.stamp, base_frame_id, odom_frame_id)
        pub.publish(odom)
        r.sleep()

if __name__ == '__main__':
    try:
        publishOdom()
    except rospy.ROSInterruptException:
        pass
