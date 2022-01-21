import condition
import py_trees
import json
from std_msgs.msg import Int8


class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.blackboard = None
        self.name = name
        self.executed = False

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="armReady")
        self.blackboard.register_key(key="robot_state", access=py_trees.common.Access.READ)

    def initialise(self):
        return

    def update(self):
        if self.blackboard.robot_state == Int8(1):
            return py_trees.common.Status.SUCCESS
        else:
            print("Robot not Ready")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        return
