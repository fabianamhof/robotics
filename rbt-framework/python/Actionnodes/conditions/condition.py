import py_trees
import time

class Condition(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.name = name
        self.executed = False


    def setup(self):
        return

    def initialise(self):
        return

    def update(self):
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        return
