import condition
import py_trees
import json
from std_msgs.msg import Int8


class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.blackboard = None
        self.previous = None
        self.name = name
        self.executed = False

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="priorityChanged")
        self.blackboard.register_key(key="priorityChanged", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="priorityChanged", access=py_trees.common.Access.WRITE)
        self.previous = None
        print("priorityChanged; Setup");


    def initialise(self):
        return

    def update(self):
        if self.blackboard.priorityChanged:
            self.blackboard.priorityChanged = False
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        return
