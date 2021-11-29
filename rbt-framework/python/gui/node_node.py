import traceback

from node_graphics_node import QDMGraphicsNode
from node_socket import Socket, INPUT, OUTPUT
from node_edge import Edge
from PyQt5.QtWidgets import *
import nodes as Bt_nodes
import Actionnodes
import importlib
import json
import py_trees
import inspect
import os
import importlib

class Node():
    def __init__(self, scene, title:str="undefined node", root=False, posX=40, posY=40, kind=None):
        self.scene = scene
        self.title = title
        self.kind = kind

        self.grNode =QDMGraphicsNode(self)
        self.grNode.setPos(posX, posY)

        self.scene.addNode(self)
        self.scene.grScene.addItem(self.grNode)

        self.output = [Socket(OUTPUT, self)]
        self.set_positions()
        self.root = root

        if not root:
            self.input = Socket(INPUT, self)
            self.input.set_position(1,0)
        else:
            self.input = None
            self.initBTNode(self)

        self.children = []

    def set_positions(self):
        for index in range(len(self.output)):
            self.output[index].set_position(len(self.output), index)

    def setPos(self, x, y):
        self.grNode.setPos(x,y)

    def add_socket(self):
        self.output.append(Socket(OUTPUT, self))
        self.set_positions()
        self.updateConnectedEdges()

    @property
    def pos(self):
        return self.grNode.pos()

    def updateConnectedEdges(self, propagadeUp=False):
        if propagadeUp:
            if self.input is not None and self.input.edge is not None:
                if self.input.edge.start_socket.node.grNode.isSelected():
                    self.input.edge.start_socket.node.updateConnectedEdges(propagadeUp=True)
                    return
        for socket in self.output + [self.input]:
            if socket is not None and socket.hasEdge():
                socket.edge.updatePositions()
        for child in self.children:
            child.updateConnectedEdges()

    def selectAllChildren(self):
        for child in self.children:
            child.grNode.setSelected(1)
            child.selectAllChildren()
        for socket in self.output:
            if socket.edge is not None:
                socket.edge.grEdge.setSelected(1)

    def addChild(self, node, initBtNode=True):
        if initBtNode:
            self.initBTNode(node)
        self.bt_node.add_child(node.bt_node)
        self.children.append(node)
        Edge(self.scene, self.output[len(self.output) - 1], node.input)
        self.add_socket()

    def removeChild(self, node):
        self.children.remove(node)
        self.bt_node.remove_child(node.bt_node)

    def cleanSockets(self):
        for socket in self.output[:-1]:
            if socket.edge == None:
                self.scene.grScene.removeItem(socket.grSocket)
                self.output.remove(socket)
        self.set_positions()
        for socket in self.output:
            if socket.hasEdge():
                socket.edge.updatePositions()


    def createChild(self):
        try:
            child = Node(self.scene, self.title + "_" + str(len(self.children)))
            position = self.grNode.pos()
            position.setY(position.y() + 200)
            child.grNode.setPos(position)
            self.addChild(child)
        except Exception as e:
            traceback.print_exception(Exception, e, e.__traceback__)
            self.scene.grScene.removeItem(child.grNode)


    def initBTNode(self, node):
        kind = None
        if node.kind != None:
            kind = node.kind
            name = node.title
        else:
            selection = ["Sequence", "Fallback", "Parallel"]
            if not node.root:
                for nodekind in os.listdir("../Actionnodes/actions"):
                    if nodekind == "__init__.py" or nodekind == "__pycache__":
                        continue
                    nodestring = "Action " + (nodekind[:-3] if nodekind.endswith(".py") else nodekind)
                    selection.append(nodestring)
                for nodekind in os.listdir("../Actionnodes/conditions"):
                    if nodekind == "__init__.py" or nodekind == "__pycache__":
                        continue
                    nodestring = "Condition " + (nodekind[:-3] if nodekind.endswith(".py") else nodekind)
                    selection.append(nodestring)
            kind, kind_ok = QInputDialog.getItem(self.scene.widget, "Node", "Select a node kind", selection, 0, False)
            if not kind_ok:
                raise Exception("Canceled by user")
            name, name_ok = QInputDialog.getText(self.scene.widget, "Name", "Select a name for the Node",
                                                 QLineEdit.Normal, kind)
            if not name_ok:
                raise Exception("Canceled by user")
        node.bt_node = self.createBTNode(kind, name)
        node.kind = type(node.bt_node).__name__ if type(node.bt_node).__name__ != "Selector" else "Fallback"
        node.title = name
        if node.kind == "Action" or node.kind == "Condition":
            node.scene.grScene.removeItem(node.output[0].grSocket)
            node.output = []
        node.grNode.initType()
        node.grNode.initTitle()


    def createBTNode(self, kind, name):
        if kind == "Sequence":
            return py_trees.composites.Sequence(name)
        elif kind == "Fallback":
            return py_trees.composites.Selector(name)
        elif kind == "Parallel":
            return py_trees.composites.Parallel(name)
        elif kind.startswith("Action"):
            module = importlib.import_module("Actionnodes.actions." + kind[7:])
            class_ = getattr(module, "Action")
            return class_(name)
        elif kind.startswith("Condition"):
            module = importlib.import_module("Actionnodes.conditions." + kind[10:])
            class_ = getattr(module, "Condition")
            return class_(name)
