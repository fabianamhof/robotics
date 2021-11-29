from node_graphics_scene import QDMGraphicsScene

class Scene():
    def __init__(self, widget):
        self.nodes = []
        self.edges = []
        self.widget = widget

        self.initUI()

    def initUI(self):
        self.grScene = QDMGraphicsScene(self)

    def addNode(self, node):
        pass

    def addEdge(self, edge):
        self.edges.append(edge)

    def removeNode(self, node, deletionRoot=False):
        # detach from parent correclty
        if deletionRoot:
            if node.input is None:
                return  # can't delete root element
            parent = node.input.edge.start_socket.node
            node.input.edge.destroyEdge()
            parent.cleanSockets()
        # destroy all children ()
        for child in node.children:
            self.removeNode(child)
        # clean all edges and sockets below, can't lose connection to children because they are already destroyed (because of the recursiv calls)
        for socket in node.output:
            if socket.edge is not None:
                socket.edge.destroyEdge()
            del socket
        self.grScene.removeItem(node.grNode)
        del node

    def removeEdge(self, edge):
        self.edges.remove(edge)