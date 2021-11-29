import json

import py_trees
from PyQt5.QtWidgets import *

import algorithm
from main import Worker
from node_scene import Scene
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from node_node import Node
from node_graphics_view import QDMGraphicsView
from node_edge import Edge


class NodeEditor(QWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = False

        self.initUI()

    def initUI(self):
        self.setGeometry(200, 200, 800, 600)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 10, 0, 10)
        self.setLayout(self.layout)

        # create graphics scene
        self.scene = Scene(self)
        self.node = None

        # create graphics view
        self.view = QDMGraphicsView(self.scene.grScene, self)
        self.layout.addWidget(self.view)
        self.save_button = QPushButton()
        self.save_button.setText("Save Tree")
        self.new_tree_button = QPushButton()
        self.new_tree_button.setText("New Tree")
        self.load_tree_button = QPushButton()
        self.load_tree_button.setText("Load Tree")
        self.start_stop_button = QPushButton()
        self.start_stop_button.setText("Start Execution")
        self.start_stop_button.clicked.connect(self.execute_tree)
        self.new_tree_button.clicked.connect(self.new_tree)
        self.save_button.clicked.connect(self.save_tree)
        self.load_tree_button.clicked.connect(self.load_tree)

        self.layout.addWidget(self.save_button)
        self.layout.addWidget(self.new_tree_button)
        self.layout.addWidget(self.load_tree_button)
        self.layout.addWidget(self.start_stop_button)
        self.view.centerOn(self.scene.grScene.scene_width // 8, 0)

        self.setWindowTitle("RBT-Lab")
        self.show()

    def save_tree(self):
        fileName, _ = QFileDialog.getSaveFileName(self, "Save tree", "../json/", "JSONs (*.json)")
        with open(fileName, 'w') as file:
            tree = algorithm.node_to_schema(self.node.bt_node, self.node, [])
            json.dump(tree, file)
            file.close()

    def new_tree(self):
        if self.node != None:
            self.scene.removeNode(self.node)
        self.node = Node(self.scene, "", root=True, posX=self.scene.grScene.scene_width // 8, posY=40)
        self.view.centerOn(self.scene.grScene.scene_width // 8, 0)

    def load_tree(self):
        fileName, _ = QFileDialog.getOpenFileName(self, "Load tree", "../json/", "JSONs (*.json)")
        with open(fileName) as json_file:
            if self.node != None:
                self.scene.removeNode(self.node)
            self.node = algorithm.instantiateSubtree(json.load(json_file), self.scene)

    def execute_tree(self):
        if not self.running:
            self.behaviourTree = py_trees.trees.BehaviourTree(root=self.node.bt_node)
            self.worker = Worker(self.behaviourTree)
            self.worker.start()
            self.running = True
            self.start_stop_button.setText("Stop Execution")
        else:
            self.behaviourTree.interrupt()
            self.running = False
            self.start_stop_button.setText("Start Execution")




    def addDebugContent(self):

        greenBrush = QBrush(Qt.green)
        outlinePen = QPen(Qt.black)
        outlinePen.setWidth(2)

        rect = self.grScene.addRect(-100, -100, 300, 100, outlinePen, greenBrush)
        rect2 = self.grScene.addRect(100, 100, 300, 100, outlinePen, greenBrush)
        rect.setFlag(QGraphicsItem.ItemIsMovable)
        rect.setFlag(QGraphicsItem.ItemIsSelectable)
        rect2.setFlag(QGraphicsItem.ItemIsMovable)
        rect2.setFlag(QGraphicsItem.ItemIsSelectable)

        widget1 = QPushButton("Save")
        proxy1 = self.grScene.addWidget(widget1)
        proxy1.setFlag(QGraphicsItem.ItemIsMovable)
        proxy1.setPos(0, 30)
        widget2 = QTextEdit()
        proxy2 = self.grScene.addWidget(widget2)
        proxy2.setFlag(QGraphicsItem.ItemIsSelectable)
        proxy2.setPos(0, 60)
