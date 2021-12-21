import py_trees
import time
import condition


class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.name = name
        self.executed = False

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="blackboardInitialized")
        self.blackboard.register_key(key="blackboardInitialized", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="blackboardInitialized", access=py_trees.common.Access.WRITE)
        self.blackboard.blackboardInitialized = False
        return

    def initialise(self):
        return

    def update(self):
        if self.blackboard.blackboardInitialized:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        return
