import time
import action
import py_trees
import algorithm
import traceback
import json


class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.name = name
        self.executed = False

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="changeSubtree")
        self.blackboard.register_key(key="subtree", access=py_trees.common.Access.READ)
        self.prev_subtree = None
        return

    def initialise(self):
        return

    def update(self):
        try:
            changingNode = self.parent.parent
            subtreeName = self.blackboard.get("subtree")
            with open('./../json/' + subtreeName + '.json') as json_file:
                treeJson = json.load(json_file)
            tree = algorithm.instantiateSubtree(treeJson)
            tree.setup_with_descendants()
            if not self.prev_subtree:
                changingNode.add_child(tree)
            else:
                changingNode.remove_child_by_id(self.prev_subtree)
                changingNode.add_child(tree)
            self.prev_subtree = changingNode.children[1].id

            return py_trees.common.Status.FAILURE  # needs to be reevaluted all the time
        except Exception as e:
            traceback.print_exception(Exception, e, e.__traceback__)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        return
