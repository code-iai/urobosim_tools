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
            (trans,rot) = listener.lookupTransform(args.target_frame, 
                    args.src_frame, rospy.Time(0))
            # print(".")
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()

            t.header.frame_id = args.target_frame
            t.child_frame_id = args.src_frame

            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]

            t.transform.rotation.x = rot[0]
            t.transform.rotation.y = rot[1]
            t.transform.rotation.z = rot[2]
            t.transform.rotation.w = rot[3]

            tfm = tf2_msgs.msg.TFMessage([t])
            # print(tfm)
            pub.publish(tfm)


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


    try:
        publisher(args)
    except rospy.ROSInterruptException:
        pass

