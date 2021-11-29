import sys
from PyQt5.QtWidgets import *

from node_editor import NodeEditor

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setDoubleClickInterval(200)

    wnd = NodeEditor();

    sys.exit(app.exec_())