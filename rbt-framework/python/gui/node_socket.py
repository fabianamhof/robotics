INPUT = 0
OUTPUT = 1

from node_graphics_socket import QDMGraphicsSocket

class Socket():
    def __init__(self, KIND, node):
        self.node = node
        self.grSocket = QDMGraphicsSocket(KIND, self, node.grNode)
        self.KIND = KIND
        self.edge = None

    def getSocketPosition(self):
        return self.grSocket.get_position()

    def setConnectedEdge(self, edge=None):
        self.edge = edge


    def set_position(self, length, index):
        self.grSocket.set_position(self.node.grNode.width / 2.0 - ((length / 2.0) - index - 0.5) * 14)

    def hasEdge(self):
        return self.edge != None
