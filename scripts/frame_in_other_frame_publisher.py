#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import tf
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg

def publisher(args):
    pub = rospy.Publisher(args.topic, tf2_msgs.msg.TFMessage, queue_size=10)
    rospy.init_node('frame_in_other_frame_publisher', anonymous=True)
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            correction_transform_x = rospy.get_param("offset_translation_x")
            correction_transform_y = rospy.get_param("offset_translation_y")
            correction_transform_z = rospy.get_param("offset_translation_z")

            correction_rotation_r = rospy.get_param("offset_rotation_r")
            correction_rotation_p = rospy.get_param("offset_rotation_p")
            correction_rotation_y = rospy.get_param("offset_rotation_y")

            apply_offset = rospy.get_param("apply_offset")

            (trans,rot) = listener.lookupTransform(args.target_frame, 
                    args.src_frame, rospy.Time(0))
            # print(".")
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()

            t.header.frame_id = args.target_frame
            t.child_frame_id = args.src_frame

            if apply_offset:
                t.transform.translation.x = trans[0] + correction_transform_x
                t.transform.translation.y = trans[1] + correction_transform_y
                t.transform.translation.z = trans[2] + correction_transform_z

                rotation_q = (rot[0], rot[1], rot[2], rot[3])
                rotation_correction_q = tf.transformations.quaternion_from_euler(correction_rotation_r, correction_rotation_p, correction_rotation_y)
                rotation_corrected_q = tf.transformations.quaternion_multiply(rotation_q,rotation_correction_q)

                t.transform.rotation.x = rotation_corrected_q[0]
                t.transform.rotation.y = rotation_corrected_q[1]
                t.transform.rotation.z = rotation_corrected_q[2]
                t.transform.rotation.w = rotation_corrected_q[3]
            else:
                t.transform.translation.x = trans[0]
                t.transform.translation.y = trans[1]
                t.transform.translation.z = trans[2]

                t.transform.rotation.x = rot[0]
                t.transform.rotation.y = rot[1]
                t.transform.rotation.z = rot[2]
                t.transform.rotation.w = rot[3]


            # correction_rotation = tf.transformations.quaternion_from_euler(correction_rotation_r, correction_rotation_p, correction_rotation_y)
            # correction_rotation = tf.transformations.quaternion_from_euler(correction_rotation_r, correction_rotation_p, correction_rotation_y)
            # # # t.transform.rotation = t.transform.rotation * correction_rotation
            # t.transform.rotation = tf.transformations.quaternion_multiply(t.transform.rotation,correction_rotation)

            # tfm = tf2_msgs.msg.TFMessage([t])
            # # print(tfm)
            # pub.publish(tfm)
            rate.sleep()


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument("-sf", "--src", dest="src_frame", required=True,
                        help="the src frame of the transform")
    parser.add_argument("-tf", "--target", dest="target_frame", required=True,
                        help="the target frame of the transform")
    parser.add_argument("-t", "--topic", dest="topic", required=True,
                        help="The topic the new transform will be published on")
    
    args = parser.parse_args()
    print("Source frame: " + args.src_frame)
    print("Target frame: " + args.target_frame)
    print("Topic: " + args.topic)

    rospy.set_param("offset_translation_x",0)
    rospy.set_param("offset_translation_y",0)
    rospy.set_param("offset_translation_z",0)

    rospy.set_param("offset_rotation_r",0)
    rospy.set_param("offset_rotation_p",0)
    rospy.set_param("offset_rotation_y",0)

    rospy.set_param("apply_offset", True)

    try:
        publisher(args)
    except rospy.ROSInterruptException:
        pass

