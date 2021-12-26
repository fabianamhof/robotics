#!/usr/bin/env python
#from inspect import GEN_RUNNING
import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3
from rospy.rostime import get_time
from std_msgs.msg import Int8
from std_msgs.msg import Bool

table_center = Point(x=1.125, y=0.125, z=0.925)
pubTargetPos = rospy.Publisher('target_pos', Point, queue_size=1000)
pubGripperPos = rospy.Publisher('gripper_pos', Bool, queue_size=1000)
step = 100

def printState(position, robot_state):
    print("Cube Position")
    print("X: "+ str(position.x))
    print("Y: "+ str(position.y))
    print("Z: "+ str(position.z))
    print("Robot State " + str(robot_state))
    print("\n")

def calculate_target_pos(cube_pos):
    return Point(cube_pos.x + 0.1, cube_pos.y + 0.1, cube_pos.z)

# returns the cubes speed in rad/s given two points and the time taken in seconds
# best to only call it when object on table 
def calculate_rotation_speed(start_pos, end_pos, time_taken):
    start = np.array([start_pos.x,start_pos.y])
    center = np.array([table_center.x,table_center.y])
    end = np.array([end_pos.x,end_pos.y])

    center_start = start - center
    center_end = end - center

    cosine_angle = np.dot(center_start, center_end) / (np.linalg.norm(center_start) * np.linalg.norm(center_end))
    angle = np.arccos(cosine_angle)

    return (angle/time_taken)

# predicts orientation of cube at a time in the future
def predict_rotation(cube_ori, rot_speed, expected_time):

    return None


def close_gripper():
    gripperWaitingTime = get_time()
    global gripper_state
    if(gripper_state.data == 1):
        return True
    else:
        pubGripperPos.publish(Bool(True))
        gripperWaitingTime = get_time()
        while(gripper_state.data == 0):
            if(gripperWaitingTime + 5 < get_time()):
                return False
        return True

def loop():
    global robot_state
    global step
    global cube_pos
    #printState(cube_pos, robot_state)
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
            grabbed = close_gripper()
            print(grabbed)
            if(grabbed):
                step += 1
            else:
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
            step += 1
    elif step == 7:
        if not (cube_pos.y > -0.5 and cube_pos.y < 0.5 and cube_pos.z < 0.6 and cube_pos.z > 0.1) and robot_state.data == 1:
            rospy.loginfo("Step %d, Resetting Robot", step)
            step = 100

    
    #Reset
    elif step == 100:
        pubTargetPos.publish(Point(0.4,0.0,0.4))
        pubGripperPos.publish(Bool(False))
        print("Step = 0, restarting")
        step = 0


        


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
    print("Grippe State: " + str(gripper_state.data))


def cubePos_callback(data):
    global cube_pos
    cube_pos = data

def cubeOri_callback(data):
    global cube_ori
    cube_ori = data


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('rbt_node', anonymous=True)
    r = rospy.Rate(10)
    rospy.Subscriber("cube_pos", Point, cubePos_callback)
    rospy.Subscriber("cube_ori", Vector3, cubePos_callback)
    rospy.Subscriber("robot_state", Int8, robotState_callback)
    rospy.Subscriber("gripper_state", Int8, gripperState_callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    while not rospy.is_shutdown():
        loop()
        r.sleep()
        



if __name__ == '__main__':
    listener()
    

