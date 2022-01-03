import py_trees
import time
import condition

from std_msgs.msg import Int8


class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.name = name
        self.executed = False
        self.blackboard = None

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="goalReached")
        self.blackboard.register_key(key="robot_state", access=py_trees.common.Access.READ)
        return

    def initialise(self):
        return

    def update(self):
        try:
            if self.blackboard.robot_state == Int8(2):
                print("Stopping execution of tree, IK Fail")
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except Exception:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        return
