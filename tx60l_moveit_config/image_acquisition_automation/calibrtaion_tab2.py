import os.path

from another_Window import *
import src.multical.workspace as ws
from src.pyqt_final2_tab2 import *

# INFO: QtInteractor can not work with PyQt6
# PyQt5
from PyQt5.QtCore import Qt, QRectF, QPoint, QPointF, pyqtSignal, QEvent, QSize, QRect
from PyQt5.QtGui import QImage, QPixmap, QPainterPath, QMouseEvent, QPainter, QPen, QColor
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QFileDialog, QSizePolicy, \
    QGraphicsItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QTableWidget, \
    QTableWidgetItem
from PyQt5.QtWidgets import QApplication, QMainWindow, QSpinBox, QWidget, QPushButton, QTextEdit, QVBoxLayout, \
    QHBoxLayout, QGridLayout, QLineEdit, QLabel, QTabWidget, QScrollArea, QTextBrowser, QCheckBox
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
# numpy is optional: only needed if you want to display numpy 2d arrays as images.
try:
    import numpy as np
except ImportError:
    np = None
from pyqt_final2 import MyTabWidget

class Calibration_tab(QWidget):

    def __init__(self, MyTabWidget, handEyeCamera=None):
        super().__init__()
        ######### Tab2 ########
        self.MyTabWidget = MyTabWidget
        self.handEyeCamera = handEyeCamera
        self.layout = QVBoxLayout(self)

        self.cb = QComboBox(self)
        # MyTabWidget.cb.addItem("No Selection")
        if self.handEyeCamera != None:
            items = len(self.handEyeCamera)
            for k in self.handEyeCamera.keys():
                text = "Group-"+ str(k)
                self.cb.addItem(text)
        self.cb.setGeometry(QRect(0, 0, 93, 28))
        self.cb.currentIndexChanged.connect(partial(selectionchange, MyTabWidget))

        self.master_num = QLabel(self)
        self.master_num.setGeometry(QRect(100, 0, 93, 28))
        self.master_num.setText("Master")

        self.slave_num = QLabel(self)
        self.slave_num.setGeometry(QRect(200, 0, 93, 28))
        self.slave_num.setText("Slave")

        # Grid for images
        self.tab2_gridLayoutWidget1 = QWidget(self)
        self.tab2_gridLayoutWidget1.setGeometry(QRect(0, 50, 1880, 500))
        self.tab2_gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.tab2_gridLayout1 = QGridLayout(self.tab2_gridLayoutWidget1)
        self.tab2_gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.tab2_gridLayout1.setObjectName("gridLayout")

        # Grid for table
        self.tab2_gridLayoutWidget3 = QWidget(self)
        self.tab2_gridLayoutWidget3.setGeometry(QRect(0, 600, 700, 700))
        self.tab2_gridLayoutWidget3.setObjectName("gridLayoutWidget")
        self.tab2_gridLayout3 = QGridLayout(self.tab2_gridLayoutWidget3)
        self.tab2_gridLayout3.setContentsMargins(0, 0, 0, 0)
        self.tab2_gridLayout3.setObjectName("gridLayout")

        self.setLayout(self.layout)
        self.MyTabWidget.tab2.setLayout(self.layout)
        pass


