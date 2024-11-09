#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped

def get_position_of_ins():
    # Initialize the ROS node
    rospy.init_node('ins_position_calculator')

    # Create a TF listener
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # Set the rate to check the transformation periodically
    while not rospy.is_shutdown():
        try:
            # Listen for the transform from 'world' to '/girona/ins'
            (trans, rot) = listener.lookupTransform('/world', '/girona1000/ins', rospy.Time(0))

            # Output the translation as the position of /girona/ins wrt /world
            rospy.loginfo("Position of /girona1000/ins w.r.t /world:")
            rospy.loginfo("x: %.2f, y: %.2f, z: %.2f" % (trans[0], trans[1], trans[2]))

            # Optional: Create a PointStamped message for further use
            ins_position = PointStamped()
            ins_position.header.frame_id = 'world'
            ins_position.header.stamp = rospy.Time.now()
            ins_position.point.x = trans[0]
            ins_position.point.y = trans[1]
            ins_position.point.z = trans[2]

            # Here you can publish or further process the ins_position as needed

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Waiting for TF between /world and /girona/ins...")

        rate.sleep()

if __name__ == '__main__':
    try:
        get_position_of_ins()
    except rospy.ROSInterruptException:
        pass
