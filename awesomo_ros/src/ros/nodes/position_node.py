#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


class ROSNode(object):
    def __init__(self):
        self.pubs = {}
        self.subs = {}

    def init_pub(self, topic, msg_type, queue_size=1):
        self.pubs[topic] = rospy.Publisher(topic,
                                           msg_type,
                                           queue_size=queue_size)

    def init_sub(self, topic, msg_type, callback):
        self.subs[topic] = rospy.Subscriber(topic, msg_type, callback)


class PositionNode(ROSNode):
    def __init__(self, mode):
        super(PositionNode, self).__init__()
        self.mode = mode
        self.position_topic = "/awesomo/position/"
        self.mavros_topic = "/mavros/local_position/pose"

        rospy.init_node("position_node", anonymous=True)
        self.init_pub(self.position_topic, PoseStamped)
        self.init_sub(self.mavros_topic, PoseStamped, self.mavros_cb)

    def mavros_cb(self, msg):
        # obtain copy of position in NED
        pos_ned = [
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z)
        ]

        # transform from mavros NED to ENU when in GPS mode
        if self.mode == "gps":
            pos_enu = [pos_ned[1], pos_ned[0], -pos_ned[2]]
            msg.pose.position.x = pos_enu[0]
            msg.pose.position.y = pos_enu[1]
            msg.pose.position.z = pos_enu[2]

        # publish
        self.pubs[self.position_topic].publish(msg)

if __name__ == '__main__':
    PositionNode(ros.get_param("/position_mode"))
    rospy.spin()
