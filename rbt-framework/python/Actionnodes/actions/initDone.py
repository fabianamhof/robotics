import time
import action
import py_trees

class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.name = name
        self.executed = False


    def setup(self):
        return

    def initialise(self):
        self.blackboard = py_trees.blackboard.Client(name="initDone")
        self.blackboard.register_key(key="initDone", access=py_trees.common.Access.WRITE)
        return

    def update(self):
        self.blackboard.initDone = True
        return py_trees.common.Status.RUNNING



    def terminate(self, new_status):
        return