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
# from QtImageViewer import *
import json
from matplotlib import pyplot as plt
from pathlib import Path
from multiprocessing import Process, Manager
import plotly.graph_objects as go
import plotly.express as px

# import rospy
import sys
# from src.aravis_show_image import find_cameras, show_image
from random import randint
import pyvistaqt
from PIL import Image
from qtpy.QtWidgets import QFrame, QAction
# from pyvistaqt import QtInteractor


# INFO: QtInteractor can not work with PyQt6
from PyQt5.QtCore import Qt, QRectF, QPoint, QPointF, pyqtSignal, QEvent, QSize, QRect
from PyQt5.QtGui import QImage, QPixmap, QPainterPath, QMouseEvent, QPainter, QPen, QColor
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QFileDialog, QSizePolicy, \
    QGraphicsItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QTableWidget, \
    QTableWidgetItem
from PyQt5.QtWidgets import QApplication, QMainWindow, QSpinBox, QWidget, QPushButton, QTextEdit, QVBoxLayout, \
    QHBoxLayout, QGridLayout, QLineEdit, QLabel, QTabWidget, QScrollArea, QTextBrowser, QCheckBox
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
# PyQt6
# from PyQt6.QtCore import Qt, QRectF, QPoint, QPointF, pyqtSignal, QEvent, QSize, QRect
# from PyQt6.QtGui import QImage, QPixmap, QPainterPath, QMouseEvent, QPainter, QPen, QColor
# from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QFileDialog, QSizePolicy, \
#     QGraphicsItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QTableWidget, \
#     QTableWidgetItem
# from PyQt6.QtWidgets import QApplication, QMainWindow, QSpinBox, QWidget, QPushButton, QTextEdit, QVBoxLayout, \
#     QHBoxLayout, QGridLayout, QLineEdit, QLabel, QTabWidget, QScrollArea, QTextBrowser, QCheckBox
# from PyQt6.QtGui import *
# from PyQt6.QtWidgets import *
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
from src.QtImageViewer import *
from src.multical.transform.rtvec import *

class CameraWindow(QWidget):
    """
    This "window" is a QWidget. If it has no parent, it
    will appear as a free-floating window as we want.
    """
    def __init__(self, base_path, workspace):
        super().__init__()

        self.base_path = base_path
        self.workspace = None
        self.cameras = None
        self.images = None
        self.boards = None
        self.intrinsic_boards = None
        self.initial_calibration = None
        self.intrinsic = None
        self.intrinsic_dataset = {}
        self.workspace_load()
        self.view = 'pose_table'

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
        self.pose_count = {}
        self.last_pose_count = len(self.images)
        self.table = {}
        self.cell_clicked = None
        self.cb = {}
        for idx, cam in enumerate(self.cameras):
            self.tab_num[cam] = QWidget()
            self.tabs.addTab(self.tab_num[cam], cam)
            self.tab_num[cam].layout = QGridLayout(self)

            self.current_camera = cam
            self.folder_path[cam] = os.path.join(self.base_path, cam)

            self.btnLoad1 = QPushButton(self.tab_num[cam])
            self.btnLoad1.setObjectName('Load Intrinsic')
            self.btnLoad1.setText('Load Intrinsic')
            self.btnLoad1.clicked.connect(partial(self.load_intrinsic, cam))
            self.btnLoad1.setGeometry(QRect(0, 0, 120, 28))

            self.btnLoad2 = QPushButton(self.tab_num[cam])
            self.btnLoad2.setObjectName('Load Pose-table')
            self.btnLoad2.setText('Load Pose-table')
            self.btnLoad2.clicked.connect(partial(self.load_poseTable, cam))
            self.btnLoad2.setGeometry(QRect(120, 0, 120, 28))

            self.btnLoad3 = QPushButton(self.tab_num[cam])
            self.btnLoad3.setObjectName('Rotation Viz')
            self.btnLoad3.setText('Rotation Viz')
            self.btnLoad3.clicked.connect(partial(self.rotation_viz, cam))
            self.btnLoad3.setGeometry(QRect(240, 0, 120, 28))

            self.btnLoad4 = QPushButton(self.tab_num[cam])
            self.btnLoad4.setObjectName('Translation Viz')
            self.btnLoad4.setText('Translation Viz')
            self.btnLoad4.clicked.connect(partial(self.translation_viz, cam))
            self.btnLoad4.setGeometry(QRect(360, 0, 120, 28))

            self.btnLoad5 = QPushButton(self.tab_num[cam])
            self.btnLoad5.setObjectName('Point spread')
            self.btnLoad5.setText('Point spread')
            self.btnLoad5.clicked.connect(partial(self.point_spread, cam))
            self.btnLoad5.setGeometry(QRect(480, 0, 120, 28))

            self.btnLoad6 = QPushButton(self.tab_num[cam])
            self.btnLoad6.setObjectName('Plot histogram')
            self.btnLoad6.setText('Plot histogram')
            self.btnLoad6.clicked.connect(partial(self.plot_hist, cam))
            self.btnLoad6.setGeometry(QRect(600, 0, 150, 28))

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

        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def plot_hist(self, cam):
        cam_path = os.path.join(self.base_path, cam)
        images = []
        for file in os.listdir(cam_path):
            file_path = os.path.join(cam_path, file)
            img0 = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
            # new_size = (int((img0.shape[0]+1)/2), int((img0.shape[1]+1)/2))
            img1 = cv2.pyrDown(img0)
            img2 = cv2.pyrDown(img1)
            images.extend(list(img2.ravel()))
            plt.hist(images, 256, [0, 256])
            plt.show()
            pass
        plt.hist(images, 256, [0, 256])
        plt.show()
        pass

    def point_spread(self, cam):
        x = []
        y = []
        z = []
        img_name = []
        if self.view == 'intrinsic':
            for img in self.intrinsic_dataset[cam].keys():
                for board in self.intrinsic_dataset[cam][img]:
                    cam_id = self.cameras.index(cam)
                    img_id = self.images.index(img)
                    board_id = self.boards.index(board)
                    ids = np.flatnonzero(self.workspace.point_table.valid[cam_id][img_id][board_id])
                    point_table = self.workspace.point_table.points[cam_id][img_id][board_id][ids]
                    point_x = [point[0] for point in point_table]
                    point_y = [point[1] for point in point_table]
                    x.extend(point_x)
                    y.extend(point_y)
                    img_name.append(img)
        elif self.view == 'pose_table':
            for img in self.images:
                for board in self.boards:
                    cam_id = self.cameras.index(cam)
                    img_id = self.images.index(img)
                    board_id = self.boards.index(board)
                    if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                        ids = np.flatnonzero(self.workspace.point_table.valid[cam_id][img_id][board_id])
                        point_table = self.workspace.point_table.points[cam_id][img_id][board_id][ids]
                        point_x = [point[0] for point in point_table]
                        point_y = [point[1] for point in point_table]
                        x.extend(point_x)
                        y.extend(point_y)
                        img_name.append(img)

        fig = px.scatter(x=x, y=y)
        fig.show()

    def rotation_viz(self, cam):
        roll = []
        pitch = []
        yaw = []
        img_name = []
        if self.view == 'intrinsic':
            for img in self.intrinsic_dataset[cam].keys():
                for board in self.intrinsic_dataset[cam][img]:
                    cam_id = self.cameras.index(cam)
                    img_id = self.images.index(img)
                    board_id = self.boards.index(board)
                    view_angles = self.workspace.pose_table.view_angles[cam_id][img_id][board_id]
                    roll.append(view_angles[0])
                    pitch.append(view_angles[1])
                    # yaw.append(view_angles[2])
                    yaw.append(0)
                    img_name.append(img)
        elif self.view == 'pose_table':
            for img in self.images:
                for board in self.boards:
                    cam_id = self.cameras.index(cam)
                    img_id = self.images.index(img)
                    board_id = self.boards.index(board)
                    if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                        view_angles = self.workspace.pose_table.view_angles[cam_id][img_id][board_id]
                        roll.append(view_angles[0])
                        pitch.append(view_angles[1])
                        # yaw.append(view_angles[2])
                        yaw.append(0)
                        img_name.append(img)

        final_layout = go.Figure()
        final_layout.add_trace(
            go.Scatter3d(
                x=roll,
                y=pitch,
                z=yaw,
                mode='markers',
                text=img_name, textposition="bottom center",
            )
        )
        final_layout.update_layout(scene=dict(
            xaxis_title='Roll',
            yaxis_title='Pitch',
            zaxis_title='Yaw'))
        final_layout.show()
        pass

    def translation_viz(self, cam):
        x = []
        y = []
        z = []
        img_name = []
        if self.view == 'intrinsic':
            for img in self.intrinsic_dataset[cam].keys():
                for board in self.intrinsic_dataset[cam][img]:
                    cam_id = self.cameras.index(cam)
                    img_id = self.images.index(img)
                    board_id = self.boards.index(board)
                    pose_table = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                    _, tvec = split(from_matrix(pose_table))
                    x.append(tvec[0])
                    y.append(tvec[1])
                    z.append(tvec[2])
                    img_name.append(img)
        elif self.view == 'pose_table':
            for img in self.images:
                for board in self.boards:
                    cam_id = self.cameras.index(cam)
                    img_id = self.images.index(img)
                    board_id = self.boards.index(board)
                    if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                        pose_table = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                        _, tvec = split(from_matrix(pose_table))
                        x.append(tvec[0])
                        y.append(tvec[1])
                        z.append(tvec[2])
                        img_name.append(img)

        final_layout = go.Figure()
        final_layout.add_trace(
            go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode='markers',
                text=img_name, textposition="bottom center",
            )
        )
        final_layout.update_layout(scene=dict(
            xaxis_title='X',
            yaxis_title='Y',
            zaxis_title='Z'))
        final_layout.show()

    def load_intrinsic(self, cam):
        self.clearLayout(self.gridLayout1[cam])
        self.clearLayout(self.gridLayout2[cam])
        self.clearLayout(self.gridLayout3[cam])
        if self.intrinsic_dataset:
            self.view = 'intrinsic'
            self.table[cam] = QTableWidget()
            self.table[cam].cellClicked.connect(partial(self.cell_was_clicked, cam, self.gridLayout1[cam]))

            first_im = list(self.intrinsic_dataset[cam].keys())[0]
            self.pose_count[cam] = self.images.index(first_im)
            self.set_image_dropDown(cam)
            self.set_viewer(cam, self.gridLayout1[cam], self.folder_path[cam], self.images[self.pose_count[cam]],
                            self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam])

            self.tab_num[cam].setLayout(self.tab_num[cam].layout)
        else:
            self.label1 = QLabel(self.tab_num[cam])
            self.label1.setObjectName(self.view)
            self.label1.setText('Error: Intrinsic calculated from another dataset')
            self.label1.setStyleSheet("border: 1px solid black;")
            self.label1.setGeometry(QRect(240, 0, 120, 28))
            self.gridLayout2[cam].addWidget(self.label1, 2, 0)
        pass

    def load_poseTable(self, cam):
        self.clearLayout(self.gridLayout1[cam])
        self.clearLayout(self.gridLayout2[cam])
        self.clearLayout(self.gridLayout3[cam])

        self.view = 'pose_table'
        # add table widget
        self.table[cam] = QTableWidget()
        self.table[cam].cellClicked.connect(partial(self.cell_was_clicked, cam, self.gridLayout1[cam]))

        self.pose_count[cam] = 0
        self.set_image_dropDown(cam)
        self.set_viewer(cam, self.gridLayout1[cam], self.folder_path[cam], self.images[self.pose_count[cam]],
                        self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam])

        self.tab_num[cam].setLayout(self.tab_num[cam].layout)
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


    def group_decode(self, cam, group):
        images = list(self.intrinsic_dataset[cam].keys())
        current_img = images[group]
        self.pose_count[cam] = self.images.index(current_img)

    def selectionchange(self, cam, group):
        print('group: ',group)
        if self.view == 'pose_table':
            self.pose_count[cam] = group
        if self.view == 'intrinsic':
            self.group_decode(cam, group)
        self.clearLayout(self.gridLayout1[cam])
        self.set_viewer(cam, self.gridLayout1[cam], self.folder_path[cam], self.images[self.pose_count[cam]],
                    self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam])
        pass

    def set_image_dropDown(self, cam):
        self.cb[cam] = QComboBox(self)
        self.cb[cam].setGeometry(QRect(0, 0, 150, 28))
        self.cb[cam].currentIndexChanged.connect(partial(self.selectionchange, cam))

        self.label1 = QLabel(self.tab_num[cam])
        self.label1.setObjectName(self.view)
        self.label1.setText(self.view)
        self.label1.setStyleSheet("border: 1px solid black;")
        self.label1.setGeometry(QRect(240, 0, 120, 28))
        self.gridLayout2[cam].addWidget(self.label1, 2, 0)

        self.gridLayout2[cam].addWidget(self.cb[cam], 2, 1)

        if self.view == 'pose_table':
            images = self.images
        elif self.view == 'intrinsic':
            images = self.intrinsic_dataset[cam].keys()
        for img in images:
            self.cb[cam].addItem(str(img))
        pass

    def set_viewer(self, cam, imageLayout, folder_path, image_name, table, tableLayout, label_Layout):
        # self.viewer[self.current_camera] = self.create_ImageViewer()
        viewer = self.create_ImageViewer()
        self.open_ImageViewer(imageLayout, viewer, folder_path, image_name, label_Layout)
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

    def draw_corners(self, frame, corners):
        for c in corners:
            x = tuple(c.astype('int'))
            frame = cv2.circle(frame, x, radius=0, color=(0, 0, 255), thickness=30)
        return frame

    def draw_corner_axes(self, frame, cam, board):
        cam_id = self.cameras.index(cam)
        board_id = self.boards.index(board)
        img_id = self.pose_count[cam]
        corners_ids = np.flatnonzero(self.workspace.point_table.valid[cam_id][img_id][board_id])
        corners = [self.workspace.point_table.points[cam_id][img_id][board_id][c] for c in corners_ids]
        #
        cam_matrix = np.array(self.initial_calibration['cameras'][cam]['K'])
        cam_dist = np.array(self.initial_calibration['cameras'][cam]['dist'])
        rtvecs = from_matrix(self.workspace.pose_table.poses[cam_id][img_id][board_id])
        marker_length = self.workspace.boards[board_id].marker_length
        rvecs, tvecs = split(rtvecs)
        cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvecs, tvecs, 0.1, thickness=20)
        frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame2 = self.draw_corners(frame1, corners)

        return frame2

    def show_board(self, cam, board, layout):
        image_path = os.path.join(self.folder_path[cam], self.images[self.pose_count[cam]])
        print(image_path)
        frame = cv2.imread(image_path)
        frame1 = self.draw_corner_axes(frame, cam, board)
        h, w, ch = frame1.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(frame1.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(1100, 1100, Qt.KeepAspectRatio)
        imageLabel = QLabel()
        x = QPixmap.fromImage(p)
        print(h, w, ch)
        imageLabel.setPixmap(x)
        self.clearLayout(layout)
        layout.addWidget(imageLabel)

    def add_table_widget(self, cam, table, tableLayout):
        table.setRowCount(len(self.boards))
        table.setColumnCount(4)
        table.setHorizontalHeaderLabels(['Num_Points', "Reprojection Error", "View Angles", "Translation"])
        table.setVerticalHeaderLabels(self.boards)
        # table.cellClicked.connect(self.cell_was_clicked)

        camera_id = self.cameras.index(cam)
        for board in self.boards:
            board_id = self.boards.index(board)
            img_name = self.images[self.pose_count[cam]]

            num_points = self.workspace.pose_table.num_points[camera_id][self.pose_count[cam]][board_id]
            repo_error = "{:.2f}".format(
                self.workspace.pose_table.reprojection_error[camera_id][self.pose_count[cam]][board_id])
            viewAngles = [float("{:.2f}".format(angle)) for angle in (self.workspace.pose_table.view_angles[camera_id][self.pose_count[cam]][board_id])]

            if self.view == 'intrinsic':
                if board not in self.intrinsic_dataset[cam][img_name]:
                    num_points = 0

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
        tableLayout.addWidget(table)


    def workspace_load(self):
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
                self.workspace = pickle.load(open(workspace_path, "rb"))
                self.cameras = self.workspace.names.camera
                self.images = self.workspace.names.image
                self.boards = self.workspace.names.board
                if "Calibration_handeye.json" in files:
                    path = os.path.join(self.base_path, "Calibration_handeye.json")
                    self.initial_calibration = json.load(open(path))
                if "intrinsic_dataset.json" in files:
                    path = os.path.join(self.base_path, "intrinsic_dataset.json")
                    dataset = json.load(open(path))
                    for cam_id, cam in enumerate(self.cameras):
                        self.intrinsic_dataset[cam] = {}
                        for id, img in enumerate(dataset[cam]['images']):
                            if img not in self.intrinsic_dataset[cam]:
                                self.intrinsic_dataset[cam][img] = []
                            self.intrinsic_dataset[cam][img].append(dataset[cam]['boards'][id])

