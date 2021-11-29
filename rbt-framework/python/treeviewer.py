# This Python file uses the following encoding: utf-8
import sys
from datetime import datetime
from PyQt5 import QtCore, QtGui, QtWidgets
  
'''
Qt Widget, which displays the last time updated and the tree in a vertical box layout
'''
class MyWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self, parent)
        
        #label for showing the time the widget was updatet last
        self.timelabel = QtWidgets.QLabel()
        self.timelabel.setText("Last update:" + str(datetime.now()))

        #Pixmap which loads the current tree into a label
        #If the picture is not found, nothing is showed
        self.pixmap = QtGui.QPixmap('rbt_root.png')
        self.label = QtWidgets.QLabel()
        self.label.setPixmap(self.pixmap) 

        # box layout which stacks the time label and the tree vertically
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.timelabel)
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        #Timer which fires every 100 ms
        #On every event, the reload pic func is called
        timer = QtCore.QTimer(self)
        timer.setInterval(100)
        timer.timeout.connect(self.reloadPic)
        timer.start()

    #Reloads the tree into the pixmap and updates the last time updated
    def reloadPic(self):
        self.pixmap = QtGui.QPixmap('rbt_root.png')
        self.label.setPixmap(self.pixmap)
        self.timelabel.setText("Last update:" + str(datetime.now()))

'''
Shows the QtApp and waits for an exit command 
'''
def main(): 
    app = QtWidgets.QApplication(sys.argv)
    widget = MyWidget()
    widget.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()