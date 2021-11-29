import py_trees
import time
import condition

class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.name = name
        self.executed = False


    def setup(self):
        return

    def initialise(self):

        return

    def update(self):
        time.sleep(2)
        return py_trees.common.Status.FAILURE


    def terminate(self, new_status):
        return
