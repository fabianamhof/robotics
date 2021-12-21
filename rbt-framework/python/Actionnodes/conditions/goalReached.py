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
        try:
            board = py_trees.blackboard.Client(name="Board")
            board.register_key(key="goalReached", access=py_trees.common.Access.READ)
            value = board.goalReached
            if value == "true":
                return py_trees.common.Status.SUCCESS
            return py_trees.common.Status.FAILURE
        except Exception:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        return
