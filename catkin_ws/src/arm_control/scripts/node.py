#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

pub = rospy.Publisher('rbt_topic', Point, queue_size=1000)


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "X: %f", data.x)
    rospy.loginfo(rospy.get_caller_id() + "Y: %f", data.y)
    rospy.loginfo(rospy.get_caller_id() + "Z: %f", data.z)
    print("\n")
    pub.publish(data)
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rbt_node', anonymous=True)

    rospy.Subscriber("target_pos", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()