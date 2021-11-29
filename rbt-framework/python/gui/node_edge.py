from node_graphics_edge import QDMGraphicsEdge

class Edge:
    def __init__(self, scene, start_socket, end_socket):
        self.scene = scene

        self.start_socket = start_socket
        self.start_socket.setConnectedEdge(self)
        self.end_socket = end_socket
        self.end_socket.setConnectedEdge(self)

        self.grEdge = QDMGraphicsEdge(self)

        self.updatePositions()

        self.scene.grScene.addItem(self.grEdge)

    def updatePositions(self):
        if self.start_socket == None or self.end_socket == None:
            raise(ReferenceError("Edge is missing Socket(s)"))
        source = self.start_socket.getSocketPosition()
        source[0] += self.start_socket.node.grNode.pos().x()
        source[1] += self.start_socket.node.grNode.pos().y()
        self.grEdge.setSource(source)
        destination = self.end_socket.getSocketPosition()
        destination[0] += self.end_socket.node.grNode.pos().x()
        destination[1] += self.end_socket.node.grNode.pos().y()
        self.grEdge.setDestination(destination)

    def destroyEdge(self):
        self.start_socket.edge = None
        self.start_socket.node.removeChild(self.end_socket.node)
        self.end_socket.edge = None
        self.scene.grScene.removeItem(self.grEdge)