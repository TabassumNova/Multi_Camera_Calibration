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


def tab2(MyTabWidget):
    ######### Tab2 ########
    MyTabWidget.tab2.layout = QVBoxLayout(MyTabWidget)

    MyTabWidget.cb = QComboBox(MyTabWidget.tab2)
    # MyTabWidget.cb.addItem("No Selection")
    if MyTabWidget.tab1_handEyeCamera != None:
        items = len(MyTabWidget.tab1_handEyeCamera)
        for k in MyTabWidget.tab1_handEyeCamera.keys():
            text = "Group-"+ str(k)
            MyTabWidget.cb.addItem(text)
    MyTabWidget.cb.setGeometry(QRect(0, 0, 93, 28))
    MyTabWidget.cb.currentIndexChanged.connect(partial(selectionchange, MyTabWidget))

    MyTabWidget.master_num = QLabel(MyTabWidget.tab2)
    MyTabWidget.master_num.setGeometry(QRect(100, 0, 93, 28))
    MyTabWidget.master_num.setText("Master")

    MyTabWidget.slave_num = QLabel(MyTabWidget.tab2)
    MyTabWidget.slave_num.setGeometry(QRect(200, 0, 93, 28))
    MyTabWidget.slave_num.setText("Slave")

    # Grid for images
    MyTabWidget.tab2_gridLayoutWidget1 = QWidget(MyTabWidget.tab2)
    MyTabWidget.tab2_gridLayoutWidget1.setGeometry(QRect(0, 50, 1880, 500))
    MyTabWidget.tab2_gridLayoutWidget1.setObjectName("gridLayoutWidget")
    MyTabWidget.tab2_gridLayout1 = QGridLayout(MyTabWidget.tab2_gridLayoutWidget1)
    MyTabWidget.tab2_gridLayout1.setContentsMargins(0, 0, 0, 0)
    MyTabWidget.tab2_gridLayout1.setObjectName("gridLayout")

    # Grid for table
    MyTabWidget.tab2_gridLayoutWidget3 = QWidget(MyTabWidget.tab2)
    MyTabWidget.tab2_gridLayoutWidget3.setGeometry(QRect(0, 600, 700, 700))
    MyTabWidget.tab2_gridLayoutWidget3.setObjectName("gridLayoutWidget")
    MyTabWidget.tab2_gridLayout3 = QGridLayout(MyTabWidget.tab2_gridLayoutWidget3)
    MyTabWidget.tab2_gridLayout3.setContentsMargins(0, 0, 0, 0)
    MyTabWidget.tab2_gridLayout3.setObjectName("gridLayout")

    MyTabWidget.tab2.setLayout(MyTabWidget.tab2.layout)
    pass

def selectionchange(MyTabWidget, i):
    handEye_Cam = MyTabWidget.tab1_handEyeCamera[str(i)]
    master_cam = handEye_Cam["master_cam"]
    master_board = handEye_Cam["master_board"]
    slave_cam = handEye_Cam["slave_cam"]
    slave_board = handEye_Cam["slave_board"]
    image_list = handEye_Cam["image_list"]
    master_path = os.path.join(MyTabWidget.folder_path, master_cam, image_list[0])
    slave_path = os.path.join(MyTabWidget.folder_path, slave_cam, image_list[0])
    imageLabel1 = image_load(master_path)
    imageLabel2 = image_load(slave_path)
    # self.clearLayout(layout)
    MyTabWidget.tab2_gridLayout1.addWidget(imageLabel1,0,0)
    MyTabWidget.tab2_gridLayout1.addWidget(imageLabel2, 0, 1)
    print(i)
    pass

def image_load(path):
    frame = cv2.imread(path)
    h, w, ch = frame.shape
    bytes_per_line = ch * w
    convert_to_Qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
    p = convert_to_Qt_format.scaled(700, 700, Qt.KeepAspectRatio)
    imageLabel = QLabel()
    x = QPixmap.fromImage(p)
    print(h, w, ch)
    imageLabel.setPixmap(x)
    return imageLabel
