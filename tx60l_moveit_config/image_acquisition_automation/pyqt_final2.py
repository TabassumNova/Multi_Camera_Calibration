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
import json
from pathlib import Path
from multiprocessing import Process, Manager
import struct

# import rospy
import sys
# from src.aravis_show_image import find_cameras, show_image
from random import randint
import pyvistaqt
from PIL import Image
from qtpy.QtWidgets import QFrame, QAction
# from pyvistaqt import QtInteractor
from another_Window import *

# INFO: QtInteractor can not work with PyQt6
# PyQt5
from PyQt5.QtCore import Qt, QRectF, QPoint, QPointF, pyqtSignal, QEvent, QSize, QRect
from PyQt5.QtGui import QImage, QPixmap, QPainterPath, QMouseEvent, QPainter, QPen, QColor
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QFileDialog, QSizePolicy, \
    QGraphicsItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QTableWidget, \
    QTableWidgetItem
from PyQt5.QtWidgets import QApplication, QMainWindow, QSpinBox, QWidget, QPushButton, QTextEdit, QVBoxLayout, \
    QHBoxLayout, QGridLayout, QLineEdit, QLabel, QTabWidget, QScrollArea, QTextBrowser, QCheckBox

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
        self.tab5 = QWidget()
        # self.tabs.resize(300, 200)
        # Add tabs
        self.tabs.addTab(self.tab1, "Operation")
        self.tabs.addTab(self.tab2, "Calibration")
        self.tabs.addTab(self.tab3, "Measurement Setup")
        self.tabs.addTab(self.tab4, "Cameras")
        self.tabs.addTab(self.tab5, "Settings")

        self.new_window = None
        #################### TAB1 ##############################
        self.tab1.layout = QGridLayout(self)
        self.tab1_workspace = None
        self.tab1_cameras = None
        self.tab1_images = None
        self.tab1_boards = None
        # self.viewer = [None] * 8
        self.viewer = {}
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
        self.pose_count = 0
        self.last_pose = 0
        self.last_pose_count = 0

        self.tab1.setLayout(self.tab1.layout)

        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def create_ImageViewer(self, folder_path = True):
        if folder_path:
            for c in self.tab1_cameras:
                self.viewer[c] = QtImageViewer()
                self.viewer[c].leftMouseButtonReleased.connect(self.handleLeftClick)
        pass

    def open_ImageViewer(self, gridLayout):
        for path, subdirs, files in os.walk(self.folder_path):
            if path == self.folder_path:
                for idx, dir in enumerate(subdirs):
                    v = os.path.join(self.folder_path, dir, self.tab1_images[self.pose_count])
                    self.viewer[dir].open(v)
                    gridLayout.addWidget(self.viewer[dir], 0, idx)
        label = QLabel()
        label.setText(self.tab1_images[self.pose_count])
        label.setAlignment(Qt.AlignCenter)
        gridLayout.addWidget(label, 2, 0)

    def add_cameraLabel(self, gridLayout):
        for idx, cam in enumerate(self.tab1_cameras):
            btn = QPushButton(self.tab1)
            btn.setObjectName(cam)
            btn.setText(cam)
            btn.clicked.connect(lambda checked: self.show_CamImages(self.tab1_cameras))
            # btn.setGeometry(QRect(543, 0, 30, 28))
            gridLayout.addWidget(btn, 1, idx)

    def show_CamImages(self, cameras):
        if self.new_window is None:
            self.new_window = AnotherWindow(self.folder_path, self.tab1_workspace)
        self.new_window.resize(500, 500)
        self.new_window.show()

    def set_viewer(self, gridLayout, group_name="group01", live_view=False, cameraImgs={}):
        if self.folder_path:
            # # Create image viewer.
            self.create_ImageViewer()
            self.open_ImageViewer(gridLayout)
            self.add_cameraLabel(gridLayout)
            # self.add_3d_scatter()
            # self.add_table_widget()

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        print(self.folder_path)
        self.workspace_load()
        self.set_viewer(gridLayout=self.gridLayout1)
        return self.folder_path

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                workspace_path = os.path.join(self.folder_path, [f for f in files if f == "workspace.json"][0])
                self.tab1_workspace = json.load(open(workspace_path))
                self.tab1_cameras = self.tab1_workspace['names']['camera']
                self.tab1_images = self.tab1_workspace['names']['image']
                self.tab1_boards = self.tab1_workspace['names']['board']
                self.last_pose_count = len(self.tab1_images)

    def loadNext(self):
        if self.last_pose_count >= self.pose_count >= 0:
            self.pose_count += 1
            self.clearLayout(self.gridLayout1)
            self.set_viewer(gridLayout=self.gridLayout1)
        else:
            self.clearLayout(self.gridLayout1)
        # return 0

    def loadPrevious(self):
        if self.pose_count > 0:
            self.pose_count -= 1
            self.clearLayout(self.gridLayout1)
            self.set_viewer(gridLayout=self.gridLayout1)
        else:
            self.clearLayout(self.gridLayout1)
        # return 0

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

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
