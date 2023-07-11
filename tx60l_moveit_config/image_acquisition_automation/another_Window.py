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

# import rospy
import sys
# from src.aravis_show_image import find_cameras, show_image
from random import randint
import pyvistaqt
from PIL import Image
from qtpy.QtWidgets import QFrame, QAction
# from pyvistaqt import QtInteractor


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


class AnotherWindow(QWidget):
    """
    This "window" is a QWidget. If it has no parent, it
    will appear as a free-floating window as we want.
    """
    def __init__(self, base_path, workspace):
        super().__init__()
        self.base_path = base_path
        self.workspace = workspace
        self.cameras = self.workspace['names']['camera']
        self.images = self.workspace['names']['image']
        self.boards = self.workspace['names']['board']
        # layout = QVBoxLayout()
        # self.label = QLabel("Another Window % d" % randint(0,100))
        # layout.addWidget(self.label)
        # self.setLayout(layout)
        self.layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        self.tab_num = {}
        self.gridLayoutWidget1 = {}
        self.gridLayout1 = {}
        self.btnLoad1 = {}
        self.btnLoad2 = {}
        self.btnLoad3 = {}
        self.folder_path = {}
        self.viewer = {}
        self.pose_count = {}
        self.last_pose_count = {}
        for idx, cam in enumerate(self.cameras):
            self.folder_path[cam] = os.path.join(self.base_path, cam)
            self.tab_num[cam] = QWidget()
            self.tabs.addTab(self.tab_num[cam], cam)
            self.pose_count[cam] = 0
            self.last_pose_count[cam] = 0

            # Grid for images
            self.gridLayoutWidget1[cam] = QWidget(self.tab_num[cam])
            self.gridLayoutWidget1[cam].setGeometry(QRect(0, 50, 1880, 1880))
            self.gridLayoutWidget1[cam].setObjectName("gridLayoutWidget")
            self.gridLayout1[cam] = QGridLayout(self.gridLayoutWidget1[cam])
            self.gridLayout1[cam].setContentsMargins(0, 0, 0, 0)
            self.gridLayout1[cam].setObjectName("gridLayout")

            self.btnLoad1 = QPushButton(self.tab_num[cam])
            self.btnLoad1.setObjectName('<')
            self.btnLoad1.setText('<')
            self.btnLoad1.clicked.connect(self.loadPrevious)
            self.btnLoad1.setGeometry(QRect(0, 0, 30, 28))

            self.btnLoad2[cam] = QPushButton(self.tab_num[cam])
            self.btnLoad2[cam].setObjectName('Load')
            self.btnLoad2[cam].setText('Load')
            print(cam)
            self.btnLoad2[cam].clicked.connect(self.open_dir_dialog(cam))
            # self.btnLoad2[cam].clicked.connect(self.set_viewer(self.folder_path[cam], self.pose_count[cam], self.gridLayout1[cam]))
            self.btnLoad2[cam].setGeometry(QRect(30, 0, 93, 28))

            self.btnLoad3[cam] = QPushButton(self.tab_num[cam])
            self.btnLoad3[cam].setObjectName('>')
            self.btnLoad3[cam].setText('>')
            self.btnLoad3[cam].clicked.connect(self.loadNext)
            self.btnLoad3[cam].setGeometry(QRect(123, 0, 30, 28))

            self.tab_num[cam].layout = QGridLayout(self)
            self.tab_num[cam].setLayout(self.tab_num[cam].layout)


        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def loadNext(self):
        pass

    def loadPrevious(self):
        pass

    def create_ImageViewer(self):
        viewer = QtImageViewer()
        viewer.leftMouseButtonReleased.connect(self.handleLeftClick)
        return viewer

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def handleLeftClick(self, x, y):
        row = int(y)
        column = int(x)
        print("Clicked on image pixel (row=" + str(row) + ", column=" + str(column) + ")")

    def open_ImageViewer(self, cam):

        v = os.path.join(self.folder_path[cam], self.images[self.pose_count[cam]])
        self.viewer[cam].open(v)
        self.gridLayout1[cam].addWidget(self.viewer[cam], 0, 0)

        # for path, subdirs, files in os.walk(self.folder_path):
        #     if path == self.folder_path:
        #         for idx, dir in enumerate(subdirs):
        #             v = os.path.join(self.folder_path, dir, self.tab1_images[self.pose_count])
        #             self.viewer[dir].open(v)
        #             gridLayout.addWidget(self.viewer[dir], 0, idx)
        # label = QLabel()
        # label.setText(self.tab1_images[self.pose_count])
        # label.setAlignment(Qt.AlignCenter)
        # gridLayout.addWidget(label, 2, 0)

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
            self.new_window = AnotherWindow(cameras)
        self.new_window.resize(500, 500)
        self.new_window.show()

    def set_viewer(self, cam):
        self.viewer[cam] = self.create_ImageViewer()
        self.open_ImageViewer(cam)
        # self.add_cameraLabel(gridLayout)
        # self.add_3d_scatter()
        # self.add_table_widget()

    def open_dir_dialog(self, cam):
        # self.workspace_load()
        self.set_viewer(cam)

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