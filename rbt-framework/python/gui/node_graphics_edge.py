from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class QDMGraphicsEdge(QGraphicsPathItem):
    def __init__(self, edge, parent=None):
        super().__init__(parent)

        self._pen = QPen(QColor("#FFFFFFFF"))
        self._pen.setWidth(3.0)

        self.edge = edge

        self.posSource = [0,0]

        self.posDestination = [0, 0]

    def setSource(self, pos):
        self.posSource = pos

    def setDestination(self, pos):
        self.posDestination = pos

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        self.updatePath()

        painter.setPen(self._pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(self.path())

    def updatePath(self):
        path = QPainterPath(QPointF(self.posSource[0], self.posSource[1]))
        path.lineTo(self.posDestination[0], self.posDestination[1])
        self.setPath(path)

    def boundingRect(self):
        return QRectF(
            min(self.posSource[0], self.posDestination[0]),
            min(self.posSource[1], self.posDestination[1]) ,
            abs(self.posSource[0] - self.posDestination[0]),
            abs(self.posSource[1] - self.posDestination[1]),
        )