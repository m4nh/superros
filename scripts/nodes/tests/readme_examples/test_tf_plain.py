#!/usr/bin/env python

import rospy
import tf
import numpy as np

rospy.init_node('tf_manipulator')
rate = rospy.Rate(rospy.get_param("hz", 30))

listener = tf.TransformListener()
br = tf.TransformBroadcaster()

while not rospy.is_shutdown():
    try:
        (trans1, rot1) = listener.lookupTransform(
            '/odom', '/base_link',
            rospy.Time(0)
        )
    except (tf.LookupException):
        continue

    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    t = tf.transformations.rotation_matrix(1.57, [0, 0, 1])
    t[:3, 3] = np.array([0, 0, 0.5]).T

    mat2 = np.dot(mat1, t)
    trans2 = tf.transformations.translation_from_matrix(mat2)
    rot2 = tf.transformations.quaternion_from_matrix(mat2)

    br.sendTransform(
        trans2,  rot2, rospy.get_rostime(),
        "base_link2", "odom"
    )

    rate.sleep()
