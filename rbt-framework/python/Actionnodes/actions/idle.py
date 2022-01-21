import time
import action
import py_trees

'''
Idle is an action that is not terminating.
It solves the problem that the RBT finishes when a subtree has finished.
'''


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
        time.sleep(3)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return
