#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Int8

pub = rospy.Publisher('target_pos', Point, queue_size=1000)
sent = False
step = 0

def printState(position, robot_state):
    print("Cube Position")
    print("X: "+ str(position.x))
    print("Y: "+ str(position.y))
    print("Z: "+ str(position.z))
    print("Robot State " + str(robot_state))
    print("\n")

def calculate_target_pos(cube_pos):
    intermediate_point = cube_pos
    intermediate_point.x += 0.1
    intermediate_point.y += 0.1

    return intermediate_point

def callback(data):
    global robot_state
    global step
    if step == 0:
        if data.y > -0.5 and data.y < 0.5 and data.z < 0.6 and data.z > 0.1 and robot_state.data == 1:
            #print("\n MOVING ROBOT \n")
            print("Step %d, Moving Robot", step)
            printState(data, robot_state)
            pub.publish(calculate_target_pos(data))
            step += 1
    if step == 1:
        if data.y > -0.5 and data.y < 0.5 and data.z < 0.6 and data.z > 0.1 and robot_state.data == 1:
            #print("\n MOVING ROBOT \n")
            rospy.loginfo("Step %d, Moving Robot", step)
            print("Cube Position")
            printState(data, robot_state)
            print("\n")
            pub.publish(data)
            step += 1
    if step == 2:
        if not (data.y > -0.5 and data.y < 0.5 and data.z < 0.6 and data.z > 0.1):
            step = 0
        


robot_state = Int8(1)

def robotState_callback(data):
    global robot_state
    robot_state = data
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rbt_node', anonymous=True)

    rospy.Subscriber("cube_pos", Point, callback)
    rospy.Subscriber("robot_state", Int8, robotState_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    sent = False
    listener()
    

