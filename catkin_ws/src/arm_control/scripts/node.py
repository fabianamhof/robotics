#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from rospy.rostime import get_time
from std_msgs.msg import Int8
from std_msgs.msg import Bool

pubTargetPos = rospy.Publisher('target_pos', Point, queue_size=1000)
pubGripperPos = rospy.Publisher('gripper_pos', Bool, queue_size=1000)
sent = False
closing = False
gripperWaitingTime = 0
step = 0

def printState(position, robot_state):
    print("Cube Position")
    print("X: "+ str(position.x))
    print("Y: "+ str(position.y))
    print("Z: "+ str(position.z))
    print("Robot State " + str(robot_state))
    print("\n")

def calculate_target_pos(cube_pos):
    return Point(cube_pos.x + 0.1, cube_pos.y + 0.1, cube_pos.z)

def close_gripper():
    global closing
    global gripperWaitingTime
    if(gripper_state.data):
        closing = False
        return 1
    if(not gripper_state.data and not closing):
        pubGripperPos.publish(Bool(True))
        closing = True
        gripperWaitingTime = get_time()
        return 2
    if(not gripper_state.data and closing):
        if(gripperWaitingTime + 5000 < get_time()):
            closing = False
            return 0

def loop():
    global robot_state
    global step
    global closing
    global gripperWaitingTime
    global cube_pos
    printState(cube_pos, robot_state)
    if step == 0:
        if cube_pos.y > -0.5 and cube_pos.y < 0.5 and cube_pos.z < 0.6 and cube_pos.z > 0.1 and robot_state.data == 1:
            rospy.loginfo("Step %d, Moving Robot", step)
            pubTargetPos.publish(calculate_target_pos(cube_pos))
            step += 1
    elif step == 1:
        if cube_pos.y > -0.5 and cube_pos.y < 0.5 and cube_pos.z < 0.6 and cube_pos.z > 0.1 and robot_state.data == 1:
            rospy.loginfo("Step %d, Moving Robot", step)
            pubTargetPos.publish(cube_pos)
            step += 1
    elif step == 2:
        if(robot_state.data == 1):
            rospy.loginfo("Step %d, Grabbing Cube", step)
            if(close_gripper() == 1):
                step += 1
            elif(close_gripper() == 0):
                step = 100
    elif step == 3:
        if(robot_state.data == 1):
            rospy.loginfo("Step %d, Moving Robot", step)
            pubTargetPos.publish(Point(0.4,0.0,0.4))
            step += 1
    elif step == 4:
        if(robot_state.data == 1):
            rospy.loginfo("Step %d, Moving Robot", step)
            pubTargetPos.publish(Point(0.1,0.2,0.6))
            step += 1
    elif step == 5:
        if(robot_state.data == 1):
            rospy.loginfo("Step %d, Moving Robot", step)
            pubGripperPos.publish(Bool(False))
            step += 1
    elif step == 6:
        if(gripper_state.data == 0):
            rospy.loginfo("Step %d, Moving Robot", step)
            pubTargetPos.publish(Point(0.2,0.2,0.6))
            step = 100


    
    #Reset
    elif step == 100:
        if not (cube_pos.y > -0.5 and cube_pos.y < 0.5 and cube_pos.z < 0.6 and cube_pos.z > 0.1):
            print("Step = 0, restarting")
            pubGripperPos.publish(Bool(False))
            pubTargetPos.publish(Point(0.4,0.0,0.4))
            step = 0
            gripperWaitingTime = get_time()


        


robot_state = Int8(1)
gripper_state = Int8(0)
cube_pos = Point()

def robotState_callback(data):
    global robot_state
    robot_state = data
    print("Robot State: " + str(robot_state.data))


def gripperState_callback(data):
    global gripper_state
    gripper_state = data

def cubePos_callback(data):
    global cube_pos
    cube_pos = data


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rbt_node', anonymous=True)
    r = rospy.Rate(10)
    rospy.Subscriber("cube_pos", Point, cubePos_callback)
    rospy.Subscriber("robot_state", Int8, robotState_callback)
    rospy.Subscriber("gripper_state", Int8, gripperState_callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        loop()
        r.sleep()
        



if __name__ == '__main__':
    sent = False
    listener()
    

