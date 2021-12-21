import json
import py_trees
from rosTopicsReader import RTReader
import action
from std_msgs.msg import Int8


class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.blackboard = None
        self.topicsReader = None
        self.name = name
        self.executed = False
        self.reset = True

    def setup(self):
        with open("../json/topics.json") as topicfile:
            self.topiclist = json.load(topicfile)
        self.blackboard = py_trees.blackboard.Client(name="handlePriority")
        for topic in self.topiclist:
            self.blackboard.register_key(key=topic["name"], access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="subtree", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="priorityChanged", access=py_trees.common.Access.WRITE)
        self.previous = None
        print("handlePriority; Setup");

    def initialise(self):
        return

    def update(self):
        # has to be implemented depending on needs
        cube_pos = self.blackboard.get("cube_pos")
        gripper_state = self.blackboard.get("gripper_state")

        newTree = self.previous
        if -0.5 < cube_pos.y < 0.5 and 0.6 > cube_pos.z > 0.1 and self.reset and gripper_state == Int8(0):
            newTree = "pickUpTree"
        elif self.reset and gripper_state == Int8(1):
            newTree = "placeDownTree"
            self.reset = False

        if not (-0.5 < cube_pos.y < 0.5 and 0.6 > cube_pos.z > 0.1):
            newTree = "waitTree"
            if not self.reset:
                self.reset = True

        if newTree != self.previous:
            self.previous = newTree
            self.blackboard.set("subtree", newTree)
            self.blackboard.set("priorityChanged", True)
            print("priorityChanged; Change Tree to " + newTree)

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return
