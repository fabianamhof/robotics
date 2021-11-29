from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.Qt import Qt
from node_graphics_node import *
import math

class QDMGraphicsScene(QGraphicsScene):
    def __init__(self, scene, parent=None):
        super().__init__(parent)

        self.scene = scene

        # settings
        self.gridSize = 50

        self._color_background = QColor("#0f0f0f")
        self._color_grid = QColor("#2f9e40")
        self._color_grid_fat = QColor("#9e2f8d")

        self._pen_gird = QPen(self._color_grid)
        self._pen_gird.setWidth(1)

        self._pen_gird_fat = QPen(self._color_grid_fat)
        self._pen_gird_fat.setWidth(3)

        self.scene_width, self.scene_heigth = 32000, 32000
        self.setSceneRect(0, 0, self.scene_width, self.scene_heigth)


        self.setBackgroundBrush(self._color_background)

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)
        left = int(math.floor(rect.left()))
        right = int(math.ceil(rect.right()))
        top = int(math.floor(rect.top()))
        bottom = int(math.ceil(rect.bottom()))

        first_left = left - (left % self.gridSize)
        first_top = top - (top % self.gridSize)


        lines_grid, lines_grid_fat = [], []
        for x in range(first_left, right, self.gridSize):
            if(x % (self.gridSize * 5) == 0):
                lines_grid_fat.append(QLine(x, top, x, bottom))
            else:
                lines_grid.append(QLine(x, top, x, bottom))
        for y in range(first_top, bottom, self.gridSize):
            if (y % (self.gridSize * 5) == 0):
                lines_grid_fat.append(QLine(left, y, right, y))
            else:
                lines_grid.append(QLine(left, y, right, y))



        try:
            painter.setPen(self._pen_gird)
            painter.drawLines(*lines_grid)
            painter.setPen(self._pen_gird_fat)
            painter.drawLines(*lines_grid_fat)
        except TypeError:
            return

    def keyReleaseEvent(self, event):
        if event.key() == Qt.Key_Delete:
            focusItem = self.focusItem()
            if type(focusItem) == QDMGraphicsNode:
                self.scene.removeNode(focusItem.node, deletionRoot=True)



