import py_trees
import rospy
from rospy.rostime import get_time
import action

from std_msgs.msg import Bool
from std_msgs.msg import Int8


class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.pub = None
        self.blackboard = None
        self.name = name
        self.executed = False
        self.published = False
        self.publishedTime = get_time()
        self.publishedCommand = None

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="controlGripper")
        self.blackboard.register_key(key="gripperPosition", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="gripperPosition", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_state", access=py_trees.common.Access.WRITE)
        self.pub = rospy.Publisher('gripper_pos', Bool, queue_size=1000)
        return

    def initialise(self):
        return

    def update(self):
        if self.published:
            if (self.blackboard.gripper_state == Int8(1)) == (self.publishedCommand == Bool(True)):
                self.published = False
                return py_trees.common.Status.SUCCESS
            else:
                if self.publishedTime + 2 < get_time():
                    self.published = False
                    return py_trees.common.Status.FAILURE
                else:
                    return py_trees.common.Status.RUNNING
        if not self.published:
            if self.name == "openGripper":
                self.blackboard.set("gripperPosition", Bool(False))
            elif self.name == "closeGripper":
                self.blackboard.set("gripperPosition", Bool(True))
            p = self.blackboard.gripperPosition
            self.pub.publish(p)
            self.published = True
            self.publishedTime = get_time()
            self.publishedCommand = p

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return
