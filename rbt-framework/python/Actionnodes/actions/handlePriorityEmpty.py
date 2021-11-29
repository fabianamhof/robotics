import json
import py_trees
from rosTopicsReader import RTReader
import action


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
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return
