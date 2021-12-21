import condition
import py_trees
import json
from std_msgs.msg import Int8


class Condition(condition.Condition):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.previous = None
        self.name = name
        self.executed = False

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="priorityChanged")
        with open("./../json/topics.json") as topicfile:
            self.topiclist = json.load(topicfile)
        for topic in self.topiclist:
            self.blackboard.register_key(key=topic["name"], access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="subtree", access=py_trees.common.Access.WRITE)
        self.previous = None
        print("priorityChanged; Setup");


    def initialise(self):
        return

    def update(self):
        print("priorityChanged; update")
        # has to be implemented depending on needs
        cube_pos = self.blackboard.get("cube_pos")
        gripper_state = self.blackboard.get("gripper_state")

        newTree = None
        if -0.5 < cube_pos.y < 0.5 and 0.6 > cube_pos.z > 0.1:
            if gripper_state == Int8(0):
                newTree = "pickUpTree"
            else:
                newTree = "placeDownTree"
        else:
            newTree = "waitTree"

        if newTree != self.previous:
            self.previous = newTree
            self.blackboard.set("subtree", newTree)
            print("priorityChanged; Change Tree to " + newTree)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        return
