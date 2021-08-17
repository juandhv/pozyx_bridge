#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import tf2_ros

from geometry_msgs.msg import TransformStamped


class Listener(object):

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(tfBuffer)

    def callback(data):
        transform_msg = TransformStamped()
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

    def listener():


        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)

    while not rospy.is_shutdown():
        try:

            # while not rospy.is_shutdown():
            listener = Listener()
            pozyx_bridge.setup_client()
            listener.run()

        except rospy.ROSInterruptException:
            pass
