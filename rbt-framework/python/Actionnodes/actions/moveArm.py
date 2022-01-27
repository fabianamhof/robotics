import py_trees
import rospy
import action
import numpy as np
from rospy.rostime import get_time

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Int8

def pos_equals(pos1, pos2, threshold):
    return abs(pos1.x - pos2.x) < threshold and abs(pos1.y - pos2.y) < threshold and abs(pos1.z - pos2.z) < threshold

def get_quaternion_from_euler(euler):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param euler.x: The euler.x (rotation around x-axis) angle in radians.
      :param euler.y: The euler.y (rotation around y-axis) angle in radians.
      :param euler.z: The euler.z (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(euler.x / 2) * np.cos(euler.y / 2) * np.cos(euler.z / 2) - np.cos(euler.x / 2) * np.sin(euler.y / 2) * np.sin(euler.z / 2)
    qy = np.cos(euler.x / 2) * np.sin(euler.y / 2) * np.cos(euler.z / 2) + np.sin(euler.x / 2) * np.cos(euler.y / 2) * np.sin(euler.z / 2)
    qz = np.cos(euler.x / 2) * np.cos(euler.y / 2) * np.sin(euler.z / 2) - np.sin(euler.x / 2) * np.sin(euler.y / 2) * np.cos(euler.z / 2)
    qw = np.cos(euler.x / 2) * np.cos(euler.y / 2) * np.cos(euler.z / 2) + np.sin(euler.x / 2) * np.sin(euler.y / 2) * np.sin(euler.z / 2)

    return Quaternion(qx, qy, qz, qw)


class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.pub = None
        self.blackboard = None
        self.name = name
        self.executed = False
        self.published = False
        self.publishedTime = get_time()

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="moveArm")
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="targetOri", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_state", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="endeffector_pos", access=py_trees.common.Access.READ)
        self.pub = rospy.Publisher('target_pos', Pose, queue_size=1000)
        return

    def initialise(self):
        return

    def update(self):
        if self.published and self.publishedTime + 5 < get_time():
            self.published = False
            return py_trees.common.Status.FAILURE

        if self.published and self.blackboard.robot_state == Int8(1) and pos_equals(self.blackboard.endeffector_pos, self.blackboard.targetPosition, 0.01):
            self.published = False
            return py_trees.common.Status.SUCCESS
        if not self.published:
            if self.name == "moveArmUp":
                self.blackboard.targetPosition = Point(self.blackboard.targetPosition.x + 0.1, self.blackboard.targetPosition.y, self.blackboard.targetPosition.z)
            self.blackboard.set("robot_state", Int8(0))
            target_pos = Pose()
            target_pos.position = self.blackboard.targetPosition
            target_pos.orientation = get_quaternion_from_euler(self.blackboard.targetOri)
            self.pub.publish(target_pos)
            self.published = True
            self.publishedTime = get_time()
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return
