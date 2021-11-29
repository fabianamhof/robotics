from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.Qt import Qt

class QDMGraphicsNode(QGraphicsItem):
    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._title_color = Qt.white
        self._title_font = QFont("Ubuntu", 18)

        self.width = 240
        self.height = 140
        self.edge_size = 10
        self._padding = 5
        self.node = node
        self.setFlag(QGraphicsItem.ItemIsFocusable)

        self._brush_default = QBrush(QColor("#FF2f9e40"))
        self._brush_selected = QBrush(QColor("#FF9e2f8d"))

        self.initUI()

    def boundingRect(self):
        return QRectF(
            0,
            0,
            2 * self.edge_size + self.width,
            2 * self.edge_size + self.height
        )

    def initUI(self):
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsMovable)

    def initTitle(self):
        self.title_item = QGraphicsTextItem(self)
        self.title_item.setDefaultTextColor(self._title_color)
        self.title_item.setFont(self._title_font)
        self.title_item.setPlainText(self.node.title[0:45])
        self.title_item.setPos(self._padding, self._padding + 25)
        self.title_item.setTextWidth(self.width - 2 * self._padding)

    def initType(self):
        self.type_item = QGraphicsTextItem(self)
        self.type_item.setDefaultTextColor(self._title_color)
        self.type_item.setFont(self._title_font)
        self.type_item.setPlainText(self.node.kind)
        self.type_item.adjustSize()
        self.type_item.setPos((self.width - self.type_item.textWidth()) //2  , self._padding)

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        path_outline = QPainterPath()
        path_outline.addRoundedRect(0,0, self.width, self.height, self.edge_size, self.edge_size)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self._brush_default if not self.isSelected() else self._brush_selected)
        painter.drawPath(path_outline.simplified())

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        self.node.updateConnectedEdges(propagadeUp=True)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.node.selectAllChildren()
        self.setFocus()

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        self.node.selectAllChildren()

    def mouseDoubleClickEvent(self, event):
        super().mouseDoubleClickEvent(event)
        if not (self.node.kind == "Action" or self.node.kind == "Condition"):
            self.node.createChild()
