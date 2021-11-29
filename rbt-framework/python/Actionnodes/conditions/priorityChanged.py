import time
import condition
import py_trees
import algorithm
import json


class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.name = name
        self.executed = False
        self.blackboard = py_trees.blackboard.Client(name="priorityChanged")


    def setup(self):
        with open("./../json/topics.json") as topicfile:
            self.topiclist = json.load(topicfile)
        for topic in self.topiclist:
            self.blackboard.register_key(key=topic, access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="subtree", access=py_trees.common.Access.WRITE)
        self.previous = None

    def initialise(self):
        return

    def update(self):
        # has to be implemented depending on needs
        score = self.blackboard.get("test1")
        if self.previous != ("tree1" if (score == None or float(score) < 0.5) else "tree2"):
            if score == None or float(score) < 0.5:
                self.blackboard.set("subtree", "tree1")
                self.previous = "tree1"
            else:
                self.blackboard.set("subtree", "tree2")
                self.previous = "tree2"
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE






    def terminate(self, new_status):
        return