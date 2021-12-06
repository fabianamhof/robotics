#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String

pub = rospy.Publisher('target_pos', Point, queue_size=1000)
sent = False


def calculate_target_pos(cube_pos):
    max_vel = [0.1,0.1,0.1,0.1]
    max_accel = [0.02,0.02,0.02,0.02]
    max_jerk = [0.01,0.01,0.01,0.01]

    intermediate_point = cube_pos
    intermediate_point.x += 0.1
    intermediate_point.y += 0.1

    return intermediate_point

def callback(data):
    
    # rospy.loginfo(rospy.get_caller_id() + "X: %f", data.x)
    # rospy.loginfo(rospy.get_caller_id() + "Y: %f", data.y)
    # rospy.loginfo(rospy.get_caller_id() + "Z: %f", data.z)
    # print("\n")
    global sent
    if data.y > -0.5 and data.y < 0.5 and data.z < 0.6 and data.z > 0.1:
        if sent == False:
            print("test\n")
            pub.publish(calculate_target_pos(data))

            rospy.sleep(1)

            pub.publish(data)

            sent = True
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rbt_node', anonymous=True)

    rospy.Subscriber("cube_pos", Point, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    sent = False
    listener()
    

