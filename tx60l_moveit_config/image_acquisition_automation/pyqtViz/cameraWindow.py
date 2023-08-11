# Ref
# https://stackoverflow.com/questions/35508711/how-to-enable-pan-and-zoom-in-a-qgraphicsview
# https://github.com/marcel-goldschen-ohm/PyQtImageViewer
# https://www.pythonguis.com/tutorials/pyqt-layouts/
# https://www.geeksforgeeks.org/pyqt5-qtabwidget/

import os.path
import cv2
import pickle
import math
import pyvista as pv
import sys
import random
import time
from functools import partial
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
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
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


class CameraWindow(QWidget):
    """
    This "window" is a QWidget. If it has no parent, it
    will appear as a free-floating window as we want.
    """
    def __init__(self, base_path, workspace):
        super().__init__()

        self.base_path = base_path
        self.workspace = workspace
        self.cameras = self.workspace.names.camera
        self.images = self.workspace.names.image
        self.boards = self.workspace.names.board
        self.layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        self.tab_num = {}
        self.gridLayoutWidget1 = {}
        self.gridLayoutWidget2 = {}
        self.gridLayoutWidget3 = {}
        self.gridLayout1 = {}
        self.gridLayout2 = {}
        self.gridLayout3 = {}
        self.btnLoad1 = {}
        self.btnLoad2 = {}
        self.btnLoad3 = {}
        self.folder_path = {}
        self.viewer = {}
        self.pose_count = 0
        self.last_pose_count = len(self.images)
        self.table = {}
        self.cell_clicked = None
        for idx, cam in enumerate(self.cameras):
            self.tab_num[cam] = QWidget()
            self.tabs.addTab(self.tab_num[cam], cam)
            self.tab_num[cam].layout = QGridLayout(self)

            self.current_camera = cam
            self.folder_path[cam] = os.path.join(self.base_path, cam)

            self.btnLoad1 = QPushButton(self.tab_num[cam])
            self.btnLoad1.setObjectName('<')
            self.btnLoad1.setText('<')
            self.btnLoad1.setGeometry(QRect(0, 0, 30, 28))

            self.btnLoad2[cam] = QPushButton(self.tab_num[cam])
            self.btnLoad2[cam].setObjectName('Load')
            self.btnLoad2[cam].setText('Load')

            self.btnLoad2[cam].setGeometry(QRect(30, 0, 93, 28))

            self.btnLoad3[cam] = QPushButton(self.tab_num[cam])
            self.btnLoad3[cam].setObjectName('>')
            self.btnLoad3[cam].setText('>')
            self.btnLoad3[cam].setGeometry(QRect(123, 0, 30, 28))

            # Grid for images
            self.gridLayoutWidget1[cam] = QWidget(self.tab_num[cam])
            self.gridLayoutWidget1[cam].setGeometry(QRect(0, 50, 1100, 900))
            self.gridLayoutWidget1[cam].setObjectName("gridLayoutWidget")
            self.gridLayout1[cam] = QGridLayout(self.gridLayoutWidget1[cam])
            self.gridLayout1[cam].setContentsMargins(0, 0, 0, 0)
            self.gridLayout1[cam].setObjectName("gridLayout")

            # Grid for Pose num
            self.gridLayoutWidget2[cam] = QWidget(self.tab_num[cam])
            self.gridLayoutWidget2[cam].setGeometry(QRect(1130, 50, 600, 30))
            self.gridLayoutWidget2[cam].setObjectName("gridLayoutWidget")
            self.gridLayout2[cam] = QGridLayout(self.gridLayoutWidget2[cam])
            self.gridLayout2[cam].setContentsMargins(0, 0, 0, 0)
            self.gridLayout2[cam].setObjectName("gridLayout")

            # Grid for table
            self.gridLayoutWidget3[cam] = QWidget(self.tab_num[cam])
            self.gridLayoutWidget3[cam].setGeometry(QRect(1130, 100, 700, 700))
            self.gridLayoutWidget3[cam].setObjectName("gridLayoutWidget")
            self.gridLayout3[cam] = QGridLayout(self.gridLayoutWidget3[cam])
            self.gridLayout3[cam].setContentsMargins(0, 0, 0, 0)
            self.gridLayout3[cam].setObjectName("gridLayout")

            # add table widget
            self.table[cam] = QTableWidget()
            self.table[cam].cellClicked.connect(partial(self.cell_was_clicked, cam, self.gridLayout1[cam]))
            # self.gridLayout3[cam].addWidget(self.table[cam])

            # self.set_viewer(self.gridLayout1[cam])
            self.set_viewer(cam, self.gridLayout1[cam], self.folder_path[cam], self.images[self.pose_count],
                            self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam])
            # self.btnLoad2[cam].clicked.connect(partial(self.set_viewer, self.gridLayout1[cam], self.folder_path[cam],
            #                                            self.images[self.pose_count]))
            self.btnLoad1.clicked.connect(partial(self.loadPrevious, cam, self.gridLayout1[cam],
                                                  self.folder_path[cam], self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam]))
            self.btnLoad3[cam].clicked.connect(partial(self.loadNext, cam, self.gridLayout1[cam],
                                                       self.folder_path[cam], self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam]))

            self.tab_num[cam].setLayout(self.tab_num[cam].layout)


        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def loadNext(self, cam, gridLayout, folder_path, table, tableLayout, label_Layout):
        if self.last_pose_count >= self.pose_count >= 0:
            self.pose_count += 1
            self.clearLayout(gridLayout)
            # self.clearLayout(tableLayout)
            self.set_viewer(cam, gridLayout, folder_path, self.images[self.pose_count], table, tableLayout, label_Layout)
        else:
            self.clearLayout(gridLayout)
            # self.clearLayout(tableLayout)
        pass

    def loadPrevious(self, cam, gridLayout, folder_path, table, tableLayout, label_layout):
        if self.pose_count > 0:
            self.pose_count -= 1
            self.clearLayout(gridLayout)
            # self.clearLayout(tableLayout)
            self.set_viewer(cam, gridLayout, folder_path, self.images[self.pose_count], table, tableLayout, label_layout)
        else:
            self.clearLayout(gridLayout)
            # self.clearLayout(tableLayout)
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

    def open_ImageViewer(self, gridLayout, viewer, folder_path, image_name, label_Layout):

        v = os.path.join(folder_path, image_name)
        viewer.open(v)
        gridLayout.addWidget(viewer, 0, 0)
        self.clearLayout(label_Layout)
        label1 = QLabel()
        label1.setText('Pose')
        label1.setFont(QFont("Times", 10, QFont.Bold))
        label1.setAlignment(Qt.AlignCenter)
        label_Layout.addWidget(label1, 2, 0)

        label = QLabel()
        label.setText(self.images[self.pose_count])
        label.setAlignment(Qt.AlignCenter)
        label_Layout.addWidget(label, 2, 1)

        label2 = QLabel()
        label2.setText('Search pose')
        label2.setFont(QFont("Times", 10, QFont.Bold))
        label2.setAlignment(Qt.AlignCenter)
        label_Layout.addWidget(label2, 2, 2)

        self.label3 = QLineEdit(self)
        self.label3.textChanged.connect(self.textchanged)
        self.label3.setAlignment(Qt.AlignCenter)
        label_Layout.addWidget(self.label3, 2, 3)

        # label4 = QPushButton()
        # label4.setText('Ok')
        # label4.clicked.connect(partial(self.clickMethod, self.label3.text()))
        # label_Layout.addWidget(label4, 2, 4)
        pass

    def textchanged(self, text):
        pass
        # self.pose_count = int(text)-1
        # self.set_viewer(self.current_camera, self.gridLayout1[self.current_camera], self.folder_path[self.current_camera], self.images[self.pose_count],
        #                 self.table[self.current_camera], self.gridLayout3[self.current_camera], self.gridLayout2[self.current_camera])
        pass

    def set_viewer(self, cam, imageLayout, folder_path, image_name, table, tableLayout, label_Layout):
        # self.viewer[self.current_camera] = self.create_ImageViewer()
        viewer = self.create_ImageViewer()
        self.open_ImageViewer(imageLayout, viewer, folder_path, image_name, label_Layout)
        # self.add_cameraLabel(gridLayout)
        # self.add_3d_scatter()
        self.add_table_widget(cam, table, tableLayout)

    def cell_was_clicked(self, cam, layout, row, column):
        print(row)
        print(column)
        print(cam)
        # self.cell_clicked = {}
        self.cell_clicked = {'camera': cam, 'board':self.boards[row]}
        print(self.cell_clicked)
        # print(cam)
        self.show_board(cam, self.boards[row], layout)
        # row = table.currentRow()
        pass

    def show_board(self, cam, board, layout):
        image_path = os.path.join(self.folder_path[cam], self.images[self.pose_count])
        print(image_path)
        frame = cv2.imread(image_path)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(1100, 1100, Qt.KeepAspectRatio)
        imageLabel = QLabel()
        x = QPixmap.fromImage(p)
        print(h, w, ch)
        imageLabel.setPixmap(x)
        self.clearLayout(layout)
        layout.addWidget(imageLabel)
        ## add corner detection and frameaxes
        # cv2.aruco.drawDetectedMarkers(frame, corners)
        # cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvecs, tvecs, 0.1)
        pass



    def add_table_widget(self, cam, table, tableLayout):
        table.setRowCount(len(self.boards))
        table.setColumnCount(4)
        table.setHorizontalHeaderLabels(['Num_Points', "Reprojection Error", "View Angles", "Translation"])
        table.setVerticalHeaderLabels(self.boards)
        # table.cellClicked.connect(self.cell_was_clicked)

        camera_id = self.cameras.index(cam)
        for board in self.boards:
            board_id = self.boards.index(board)
            num_points = self.workspace.pose_table.num_points[camera_id][self.pose_count][board_id]
            repo_error = "{:.2f}".format(
                self.workspace.pose_table.reprojection_error[camera_id][self.pose_count][board_id])
            viewAngles = [float("{:.2f}".format(angle)) for angle in (self.workspace.pose_table.view_angles[camera_id][self.pose_count][board_id])]

            item1 = QTableWidgetItem()
            item2 = QTableWidgetItem()
            item3 = QTableWidgetItem()
            item4 = QTableWidgetItem()
            if num_points == 0:
                item1.setText("")
                table.setItem(board_id, 0, item1)
                item2.setText("")
                table.setItem(board_id, 1, item2)
                item3.setText("")
                table.setItem(board_id, 2, item3)
                item4.setText("")
                table.setItem(board_id, 3, item4)
            else:
                # text = 'num_points: ' + str(num_points) + ' | ' + 'Repo_error: ' + str(repo_error)
                item1.setText(str(num_points))
                table.setItem(board_id, 0, item1)
                item2.setText(str(repo_error))
                table.setItem(board_id, 1, item2)
                item3.setText(str(viewAngles))
                table.setItem(board_id, 2, item3)
                item4.setText(str(0))
                table.setItem(board_id, 3, item4)
            pass
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        # self.clearLayout(tableLayout)
        tableLayout.addWidget(table)

    def open_dir_dialog(self):
        # self.workspace_load()
        self.set_viewer()

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                workspace_path = os.path.join(self.folder_path, [f for f in files if f == "workspace.pkl"][0])
                self.tab1_workspace = pickle.load(open(workspace_path, "rb"))
                self.tab1_cameras = self.tab1_workspace.names.camera
                self.tab1_images = self.tab1_workspace.names.image
                self.tab1_boards = self.tab1_workspace.names.board
                self.last_pose_count = len(self.tab1_images)

