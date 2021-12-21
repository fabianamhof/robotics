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
        return

    def update(self):
        time.sleep(2)
        print("Idle waiting!")
        return py_trees.common.Status.FAILURE



    def terminate(self, new_status):
        return