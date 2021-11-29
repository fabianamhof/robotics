from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


class QDMGraphicsSocket(QGraphicsItem):
    def __init__(self, KIND, socket, parent=None):
        super().__init__(parent)
        self.socket = socket
        self._radius = 6.0
        self._node = parent
        self._brush_default = QBrush(QColor("#FF9e2f8d"))
        self._brush_selected = QBrush(QColor("#FF2f9e40"))
        self._pen_default = QPen(QColor("#FF9e2f8d"))
        self._pen_selected = QPen(QColor("#FF2f9e40"))
        self._pen_empty = QPen(QColor("#FFffffff"))
        self._pen_empty.setWidth(2)
        self.position_x = 0
        self.KIND = KIND

    def paint(self, painter, QStyleOptionGraphicsItem, wdget=None):

        painter.setBrush(self._brush_default if not self._node.isSelected() else self._brush_selected)
        painter.setPen((self._pen_default if not self._node.isSelected() else self._pen_selected) if not self.socket.hasEdge() else self._pen_empty)
        painter.drawEllipse(self.position_x, self.position_y, 2 * self._radius, 2 * self._radius)

    def boundingRect(self):
        return QRectF(
            self.position_x,
            self.position_y,
            2 * self._radius,
            2 * self._radius
        )

    def set_position(self, position_x):
        self.position_x = position_x - self._radius
        self.position_y = -self._radius + self._node.height if self.KIND == 1 else -self._radius

    def get_position(self):
        return[self.position_x + self._radius, self.position_y + self._radius]

    def mousePressEvent(self, event):
        self._node.node.createChild()



