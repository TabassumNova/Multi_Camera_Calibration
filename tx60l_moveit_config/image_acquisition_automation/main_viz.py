# Ref
# https://stackoverflow.com/questions/35508711/how-to-enable-pan-and-zoom-in-a-qgraphicsview
# https://github.com/marcel-goldschen-ohm/PyQtImageViewer
# https://www.pythonguis.com/tutorials/pyqt-layouts/
# https://www.geeksforgeeks.org/pyqt5-qtabwidget/

import os.path
from pyqtViz.cameraWindow import *
# from calibrtaion_tab2 import *
# from src.aravis_show_image import *
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
from src.multical_scripts.board_angle import *
from src.multical_scripts.handEye_viz import *
# numpy is optional: only needed if you want to display numpy 2d arrays as images.
try:
    import numpy as np
except ImportError:
    np = None

# qimage2ndarray is optional: useful for displaying numpy 2d arrays as images.
# !!! qimage2ndarray requires PyQt5.
#     Some custom code in the viewer appears to handle the conversion from numpy 2d arrays,
#     so qimage2ndarray probably is not needed anymore. I've left it here just in case.
try:
    import qimage2ndarray
except ImportError:
    qimage2ndarray = None

from pyqtViz.operation_tab import *
from pyqtViz.calibration_tab import *
from pyqtViz.viewPlan_tab import *
import cv2

# Creating the main window
class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = "RAPtOr_GUI"
        self.left = 0
        self.top = 0
        self.width = 1000
        self.height = 800
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.tab_widget = MyTabWidget(self)
        self.setCentralWidget(self.tab_widget)
        self.show()


# Creating tab widgets
class MyTabWidget(QWidget):
    def __init__(self, parent=None):
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout(self)
        # Initialize tab screen
        self.tabs = QTabWidget()

        self.tab1 = Operation()
        self.tab2 = Calibration()
        self.tab3 = QWidget()
        self.tab4 = View_Plan()
        self.tab5 = QWidget()
        # self.tabs.resize(300, 200)
        # Add tabs
        self.tabs.addTab(self.tab1, "Operation")
        self.tabs.addTab(self.tab2, "Calibration")
        self.tabs.addTab(self.tab3, "Measurement Setup")
        self.tabs.addTab(self.tab4, "View_Plan")
        self.tabs.addTab(self.tab5, "Settings")

        self.new_window = None
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

# def multiprocess_loading(viewer, folder_path, camera, pose, gridLayout):
def multiprocess_loading(viewer, folder_path, camera, pose):
    viewer = QtImageViewer()
    v = folder_path + '/' + camera + '/p' + str(pose) + '.png'
    viewer.open(v)
    print('multiprocess: ', pose)



def f(name):
    print('hello', name)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
