# libraries
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import threading

import json
import py_trees
import os

import graphviz
import sys
from datetime import datetime

# modules
import algorithm as algorithm
import treeviewer as tv
import display_treeviewer as dtv

class Worker (threading.Thread):

    def print_tree(self, tree):
        root = self.tree.root
        dot = dtv.render_dot_tree(root, target_directory=os.path.abspath(os.path.join(os.getcwd(), os.pardir)))

    def __init__(self, tree:py_trees.trees.BehaviourTree):
        self.tree = tree
        threading.Thread.__init__(self)
    def run(self):
        try:
            self.tree.setup()
            self.tree.tick_tock(
                period_ms=20,
                number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
                pre_tick_handler=None,
                post_tick_handler=self.print_tree
            )
        except KeyboardInterrupt:
            behaviourTree.interrupt()

