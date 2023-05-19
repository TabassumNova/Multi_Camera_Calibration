# Ref
# https://stackoverflow.com/questions/35508711/how-to-enable-pan-and-zoom-in-a-qgraphicsview
# https://github.com/marcel-goldschen-ohm/PyQtImageViewer
# https://www.pythonguis.com/tutorials/pyqt-layouts/
# https://www.geeksforgeeks.org/pyqt5-qtabwidget/

import os.path
import pyvista as pv
import sys
import random
import time
from QtImageViewer import *
from pathlib import Path

import pyvistaqt
from PIL import Image
from qtpy.QtWidgets import QFrame, QAction
# from pyvistaqt import QtInteractor


# INFO: QtInteractor can not work with PyQt6
# PyQt5
from PyQt5.QtCore import Qt, QRectF, QPoint, QPointF, pyqtSignal, QEvent, QSize, QRect
from PyQt5.QtGui import QImage, QPixmap, QPainterPath, QMouseEvent, QPainter, QPen, QColor
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QFileDialog, QSizePolicy, \
    QGraphicsItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QTableWidget, QTableWidgetItem
from PyQt5.QtWidgets import QApplication, QMainWindow, QSpinBox, QWidget, QPushButton, QTextEdit, QVBoxLayout, \
        QHBoxLayout, QGridLayout, QLineEdit, QLabel, QTabWidget, QScrollArea, QTextBrowser
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
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout(self)

        # Initialize tab screen
        self.tabs = QTabWidget()
        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tab3 = QWidget()
        self.tab4 = QWidget()
        # self.tabs.resize(300, 200)
        # Add tabs
        self.tabs.addTab(self.tab1, "Operation")
        self.tabs.addTab(self.tab2, "Calibration")
        self.tabs.addTab(self.tab3, "Measurement Setup")
        self.tabs.addTab(self.tab4, "Settings")

        # Create first tab
        self.tab1.layout = QGridLayout(self)
        self.viewer = [None] * 8

        # add buttons
        self.btnLoad = QPushButton(self.tab1)
        self.btnLoad.setObjectName('Live View')
        self.btnLoad.setText('Live View')
        self.btnLoad.setGeometry(QRect(0, 0, 93, 28))
        self.btnLoad.clicked.connect(self.saveImage)

        self.btnLoad1 = QPushButton(self.tab1)
        self.btnLoad1.setObjectName('Start')
        self.btnLoad1.setText('Start')
        self.btnLoad1.setGeometry(QRect(90, 0, 93, 28))
        self.btnLoad1.clicked.connect(self.nextPose)

        self.btnLoad2 = QPushButton(self.tab1)
        self.btnLoad2.setObjectName('Stop')
        self.btnLoad2.setText('Stop')
        self.btnLoad2.setGeometry(QRect(180, 0, 93, 28))
        self.btnLoad2.clicked.connect(self.nextPose)

        self.btnLoad3 = QPushButton(self.tab1)
        self.btnLoad3.setObjectName('x')
        self.btnLoad3.setText('x')
        self.btnLoad3.clicked.connect(self.nextPose)
        self.btnLoad3.setGeometry(QRect(290, 0, 30, 28))

        self.btnLoad4 = QPushButton(self.tab1)
        self.btnLoad4.setObjectName('Save')
        self.btnLoad4.setText('Save')
        self.btnLoad4.clicked.connect(self.nextPose)
        self.btnLoad4.setGeometry(QRect(320, 0, 93, 28))

        self.btnLoad5 = QPushButton(self.tab1)
        self.btnLoad5.setObjectName('<')
        self.btnLoad5.setText('<')
        self.btnLoad5.clicked.connect(self.loadPrevious)
        self.btnLoad5.setGeometry(QRect(420, 0, 30, 28))

        self.btnLoad6 = QPushButton(self.tab1)
        self.btnLoad6.setObjectName('>')
        self.btnLoad6.setText('>')
        self.btnLoad6.clicked.connect(self.loadNext)
        self.btnLoad6.setGeometry(QRect(543, 0, 30, 28))

        self.btnLoad7 = QPushButton(self.tab1)
        self.btnLoad7.setObjectName('Load')
        self.btnLoad7.setText('Load')
        self.btnLoad7.clicked.connect(self.open_dir_dialog)
        self.btnLoad7.setGeometry(QRect(450, 0, 93, 28))

        # Grid for images
        self.gridLayoutWidget1 = QWidget(self.tab1)
        self.gridLayoutWidget1.setGeometry(QRect(0, 50, 1880, 300))
        self.gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.gridLayout1 = QGridLayout(self.gridLayoutWidget1)
        self.gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout1.setObjectName("gridLayout")

        self.folder_path = ''
        self.live_view = ''
        self.pose = 1
        self.last_pose = 0

        # Grid for Scatter plot and table
        self.gridLayoutWidget2 = QWidget(self.tab1)
        self.gridLayoutWidget2.setGeometry(QRect(0, 400, 1880, 400))
        self.gridLayoutWidget2.setObjectName("gridLayoutWidget")
        self.gridLayout2 = QGridLayout(self.gridLayoutWidget2)
        self.gridLayout2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout2.setObjectName("gridLayout")

        # Grid for INFO
        self.gridLayoutWidget3 = QWidget(self.tab1)
        self.gridLayoutWidget3.setGeometry(QRect(0, 810, 1880, 130))
        self.gridLayoutWidget3.setObjectName("gridLayoutWidget")
        self.gridLayout3 = QGridLayout(self.gridLayoutWidget3)
        self.gridLayout3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout3.setObjectName("gridLayout")

        # add scatter widget
        self.frame = QFrame()
        self.plotter = pyvistaqt.QtInteractor(self.frame)
        self.gridLayout2.addWidget(self.plotter.interactor, 0, 0)

        # add table widget
        self.table = QTableWidget()
        self.gridLayout2.addWidget(self.table, 0, 1)

        # INFO Area
        self.scrollArea = QScrollArea(self.tab1)
        self.scrollArea.setGeometry(QRect(0, 800, 481, 171))
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollArea.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.scrollArea.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        # self.scrollAreaWidgetContents = QWidget()
        # self.scrollAreaWidgetContents.setGeometry(QRect(0, 810, 480, 171))
        # self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.infoLabel = QLabel()
        self.infoLabel.setGeometry(QRect(0, 700, 481, 171))
        self.infoLabel.setObjectName("infoLabel")
        self.infoLabel.setText('INFO')
        self.scrollArea.setWidget(self.infoLabel)
        self.gridLayout3.addWidget(self.scrollArea, 0, 0)

        self.tab1.setLayout(self.tab1.layout)
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def set_viewer(self, group_name="group01"):
        print('folder path: ',self.folder_path)
        print('pose: ', self.pose)

        if self.folder_path:
            cam1 = self.folder_path + '/cam1'
            self.last_pose = (len([name for name in os.listdir(cam1) if os.path.isfile(os.path.join(cam1, name))]))
            # start = time.time()
            # Open images as numpy array
            v1 = self.folder_path + '/cam1' + '/p' + str(self.pose) + '.png'
            v2 = self.folder_path + '/cam2' + '/p' + str(self.pose) + '.png'
            v3 = self.folder_path + '/cam3' + '/p' + str(self.pose) + '.png'
            v4 = self.folder_path + '/cam4' + '/p' + str(self.pose) + '.png'
            v5 = self.folder_path + '/cam5' + '/p' + str(self.pose) + '.png'
            v6 = self.folder_path + '/cam6' + '/p' + str(self.pose) + '.png'

            start = time.time()
            # Create image viewer.
            self.viewer[0] = QtImageViewer()
            self.viewer[0].open(v1)
            self.viewer[0].leftMouseButtonReleased.connect(self.handleLeftClick)

            self.viewer[1] = QtImageViewer()
            self.viewer[1].open(v2)
            self.viewer[1].leftMouseButtonReleased.connect(self.handleLeftClick)

            self.viewer[2] = QtImageViewer()
            self.viewer[2].open(v3)
            self.viewer[2].leftMouseButtonReleased.connect(self.handleLeftClick)

            self.viewer[3] = QtImageViewer()
            self.viewer[3].open(v4)
            self.viewer[3].leftMouseButtonReleased.connect(self.handleLeftClick)

            self.viewer[4] = QtImageViewer()
            self.viewer[4].open(v5)
            self.viewer[4].leftMouseButtonReleased.connect(self.handleLeftClick)

            self.viewer[5] = QtImageViewer()
            self.viewer[5].open(v6)
            self.viewer[5].leftMouseButtonReleased.connect(self.handleLeftClick)

            self.add_3d_scatter()
            self.add_table_widget()

            label1 = QLabel()
            label1.setText("Camera 01")
            label1.setAlignment(Qt.AlignCenter)
            self.gridLayout1.addWidget(label1, 1, 0)
            label2 = QLabel()
            label2.setText("Camera 02")
            label2.setAlignment(Qt.AlignCenter)
            self.gridLayout1.addWidget(label2, 1, 1)
            label3 = QLabel()
            label3.setText("Camera 03")
            label3.setAlignment(Qt.AlignCenter)
            self.gridLayout1.addWidget(label3, 1, 2)
            label4 = QLabel()
            label4.setText("Camera 04")
            label4.setAlignment(Qt.AlignCenter)
            self.gridLayout1.addWidget(label4, 1, 3)
            label5 = QLabel()
            label5.setText("Camera 05")
            label5.setAlignment(Qt.AlignCenter)
            self.gridLayout1.addWidget(label5, 1, 4)
            label6 = QLabel()
            label6.setText("Camera 06")
            label6.setAlignment(Qt.AlignCenter)
            self.gridLayout1.addWidget(label6, 1, 5)

            # self.layout.update()
            self.gridLayout1.addWidget(self.viewer[0], 0, 0)
            self.gridLayout1.addWidget(self.viewer[1], 0, 1)
            self.gridLayout1.addWidget(self.viewer[2], 0, 2)
            self.gridLayout1.addWidget(self.viewer[3], 0, 3)
            self.gridLayout1.addWidget(self.viewer[4], 0, 4)
            self.gridLayout1.addWidget(self.viewer[5], 0, 5)

            end = time.time()
            print("Passed time: ", end - start)

        elif self.live_view:
            v1 = Image.open(group_name + "_img4.png")
            i1 = np.asarray(v1)
            v2 = Image.open(group_name + "_img2.png")
            i2 = np.asarray(v2)
            v3 = Image.open(group_name + "_img3.png")
            i3 = np.asarray(v3)
            v4 = Image.open(group_name + "_img4.png")
            i4 = np.asarray(v4)
            v5 = Image.open(group_name + "_img5.png")
            i5 = np.asarray(v5)
            v6 = Image.open(group_name + "_img6.png")
            i6 = np.asarray(v6)
            v7 = Image.open(group_name + "_img1.png")
            i7 = np.asarray(v7)
            pass

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        self.set_viewer()
        return self.folder_path

    def loadNext(self):
        if self.last_pose >= self.pose >= 1:
            self.pose += 1
            self.set_viewer()
        # return 0

    def loadPrevious(self):
        if self.pose > 1:
            self.pose -= 1
            self.set_viewer()
        # return 0

    def handleLeftClick(self, x, y):
        row = int(y)
        column = int(x)
        print("Clicked on image pixel (row=" + str(row) + ", column=" + str(column) + ")")

    def handleViewChange(self):
        print("viewChanged")

    def saveImage(self):
        print("Button clicked, Hello! Save Image")

    def nextPose(self):
        print("Button clicked, Next!")
        self.set_viewer(group_name="group02")
        self.show()

    def add_3d_scatter(self):
        point_cloud = np.random.random((100, 3))
        pdata = pv.PolyData(point_cloud)
        pdata['orig_sphere'] = np.arange(100)
        print('add scatter')
        # create many spheres from the point cloud
        sphere = pv.Sphere(radius=0.02, phi_resolution=10, theta_resolution=10)
        pc = pdata.glyph(scale=False, geom=sphere, orient=False)
        # pc.plot(cmap='Reds')
        # self.plotter.subplot(1, 1)
        self.plotter.add_mesh(pc, show_edges=True)
        self.plotter.reset_camera()

    def add_table_widget(self):
        def get_rgb_from_hex(code):
            code_hex = code.replace("#", "")
            rgb = tuple(int(code_hex[i:i + 2], 16) for i in (0, 2, 4))
            return QColor.fromRgb(rgb[0], rgb[1], rgb[2])

        colors = [("Red", "#FF0000"),
                  ("Green", "#00FF00"),
                  ("Blue", "#0000FF"),
                  ("Black", "#000000"),
                  ("White", "#FFFFFF"),
                  ("Electric Green", "#41CD52"),
                  ("Dark Blue", "#222840"),
                  ("Yellow", "#F9E56d")]

        self.table.setRowCount(len(colors))
        self.table.setColumnCount(len(colors[0]) + 1)
        self.table.setHorizontalHeaderLabels(["Name", "Hex Code", "Color"])

        for i, (name, code) in enumerate(colors):
            item_name = QTableWidgetItem(name)
            item_code = QTableWidgetItem(code)
            item_color = QTableWidgetItem()
            item_color.setBackground(get_rgb_from_hex(code))
            self.table.setItem(i, 0, item_name)
            self.table.setItem(i, 1, item_code)
            self.table.setItem(i, 2, item_color)

    def change_name(self):
        self.name_label.setText("Name Changed")


if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
