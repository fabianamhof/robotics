import json
import time
import action
import py_trees

from rosTopicsReader import RTReader


class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.topicsReader = None
        self.topiclist = None
        self.name = name
        self.executed = False
        self.blackboard = None


    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="blackboardInitializer")
        self.blackboard.register_key(key="priorityChanged", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="blackboardInitialized", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="initDone", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="armVelocity", access=py_trees.common.Access.WRITE)
        return

    def initialise(self):
        return

    def update(self):
        with open("../json/topics.json") as topicfile:
            self.topiclist = json.load(topicfile)
        self.topicsReader = RTReader(self.topiclist)
        self.blackboard.set("priorityChanged", False)
        self.blackboard.set("blackboardInitialized", True)
        self.blackboard.set("initDone", False)
        self.blackboard.set("armVelocity", 100.0)
        return py_trees.common.Status.SUCCESS



    def terminate(self, new_status):
        return