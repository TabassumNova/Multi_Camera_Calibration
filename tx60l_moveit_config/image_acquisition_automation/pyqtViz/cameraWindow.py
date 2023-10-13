# Ref
# https://stackoverflow.com/questions/35508711/how-to-enable-pan-and-zoom-in-a-qgraphicsview
# https://github.com/marcel-goldschen-ohm/PyQtImageViewer
# https://www.pythonguis.com/tutorials/pyqt-layouts/
# https://www.geeksforgeeks.org/pyqt5-qtabwidget/

import os.path
import cv2
import pickle
import math
# import pyvista as pv
import sys
import random
import time
import pandas as pd
# import openpyxl
from functools import partial
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib.cm as cm           # import colormap stuff!
import numpy as np
# from QtImageViewer import *
import json
from matplotlib import pyplot as plt
from pathlib import Path
from multiprocessing import Process, Manager
import plotly.graph_objects as go
import plotly.express as px
import pandas as pd
from src.multical_scripts.extrinsic_viz import *
from src.multical_scripts.singleCalib_viz import *
# import rospy
import sys
# from src.aravis_show_image import find_cameras, show_image
from random import randint
# import pyvistaqt
from PIL import Image
# from qtpy.QtWidgets import QFrame, QAction
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
        self.setWindowTitle('Camera Window')
        self.base_path = base_path
        self.workspace = None
        self.workspace_test = None
        self.inlier_mask = {}
        self.final_images = []
        self.cameras = None
        self.images = None
        self.boards = None
        self.intrinsic_boards = None
        self.initial_calibration = None
        self.intrinsic = None
        self.intrinsic_dataset = {}
        self.camera_color = {} 
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

            self.btnLoad8 = QPushButton(self.tab_num[cam])
            self.btnLoad8.setObjectName('Load Bundle-Adjustment')
            self.btnLoad8.setText('Load Bundle-Adjustment')
            self.btnLoad8.clicked.connect(partial(self.load_bundleAdjustment, cam))
            self.btnLoad8.setGeometry(QRect(240, 0, 150, 28))

            self.btnLoad3 = QPushButton(self.tab_num[cam])
            self.btnLoad3.setObjectName('Rt Viz')
            self.btnLoad3.setText('Rt Viz')
            self.btnLoad3.clicked.connect(partial(self.rotation_translation_viz))
            self.btnLoad3.setGeometry(QRect(240+150, 0, 120, 28))

            self.btnLoad4 = QPushButton(self.tab_num[cam])
            self.btnLoad4.setObjectName('Extrinsic Viz')
            self.btnLoad4.setText('extrinsic Viz')
            self.btnLoad4.clicked.connect(partial(self.extrinsic_viz, cam))
            self.btnLoad4.setGeometry(QRect(360+150, 0, 120, 28))

            self.btnLoad5 = QPushButton(self.tab_num[cam])
            self.btnLoad5.setObjectName('Point_Angle_Error')
            self.btnLoad5.setText('Point_Angle_Error')
            self.btnLoad5.clicked.connect(partial(self.point_angle_error, cam))
            self.btnLoad5.setGeometry(QRect(480+150, 0, 120, 28))

            self.btnLoad6 = QPushButton(self.tab_num[cam])
            self.btnLoad6.setObjectName('Plot histogram')
            self.btnLoad6.setText('Plot histogram')
            self.btnLoad6.clicked.connect(partial(self.plot_hist, cam))
            self.btnLoad6.setGeometry(QRect(600+150, 0, 150, 28))

            self.btnLoad7 = QPushButton(self.tab_num[cam])
            self.btnLoad7.setObjectName('Final extrinsic')
            self.btnLoad7.setText('Final extrinsic')
            self.btnLoad7.clicked.connect(partial(self.final_extrinsic, cam))
            self.btnLoad7.setGeometry(QRect(750+150, 0, 150, 28))

            self.btnLoad9 = QPushButton(self.tab_num[cam])
            self.btnLoad9.setObjectName('View_error viz')
            self.btnLoad9.setText('View_error viz')
            self.btnLoad9.clicked.connect(partial(self.view_error_viz, cam))
            self.btnLoad9.setGeometry(QRect(750 + 300, 0, 150, 28))

            # self.label1 = QLabel(self.tab_num[cam])
            # self.label1.setObjectName('Variation(max-min)')
            # self.label1.setText('Variation(max-min)')
            # self.label1.setStyleSheet("border: 1px solid black;")
            # self.label1.setGeometry(QRect(900+150, 0, 120, 28))
            #
            # self.label2 = QLabel(self.tab_num[cam])
            # self.label2.setObjectName('_')
            # self.label2.setText('_')
            # self.label2.setStyleSheet("border: 1px solid black;")
            # self.label2.setGeometry(QRect(1020+120, 0, 70, 28))
            #
            # self.label3 = QLabel(self.tab_num[cam])
            # self.label3.setObjectName('std')
            # self.label3.setText('std')
            # self.label3.setStyleSheet("border: 1px solid black;")
            # self.label3.setGeometry(QRect(1090+70, 0, 50, 28))
            #
            # self.label4 = QLabel(self.tab_num[cam])
            # self.label4.setObjectName('_')
            # self.label4.setText('_')
            # self.label4.setStyleSheet("border: 1px solid black;")
            # self.label4.setGeometry(QRect(1140+50, 0, 70, 28))
            #
            # self.label5 = QLabel(self.tab_num[cam])
            # self.label5.setObjectName('num_view')
            # self.label5.setText('num_view')
            # self.label5.setStyleSheet("border: 1px solid black;")
            # self.label5.setGeometry(QRect(1210+70, 0, 70, 28))
            #
            # self.label6 = QLabel(self.tab_num[cam])
            # self.label6.setObjectName('_')
            # self.label6.setText('_')
            # self.label6.setStyleSheet("border: 1px solid black;")
            # self.label6.setGeometry(QRect(1280+70, 0, 70, 28))

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

    def final_extrinsic(self, cam):
        cam_init = 'initial_calibration_M' + cam + '.json'
        cam_final = 'calibration_' + cam + '.json'
        i = Interactive_calibration(self.base_path)
        for path, subdirs, files in os.walk(base_path):
            for name in files:
                if name == cam_init:
                    cam_init_path = os.path.join(self.base_path, name)
                    i.load_campose(cam_init_path, calib_type='initial')
                elif name == cam_final:
                    cam_final_path = os.path.join(self.base_path, name)
                    i.load_campose(cam_final_path, calib_type='final')
        i.draw_cameras()

    def set_Cam_color(self):
            colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime', 'pink', 'teal', 'darkcyan', 'violet', 'brown', 'indigo']
            for idx, cam in enumerate(self.workspace.names.camera):
                self.camera_color[cam] = colors[idx]

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

    def point_angle_error(self, cam):
        x = []
        y = []
        z = []

        if self.view == 'intrinsic':
            observation_dict = {}
            fig, axs = plt.subplots(2, math.ceil(self.workspace.sizes.camera/2))
            fig2, axs2 = plt.subplots(2, math.ceil(self.workspace.sizes.camera/2))
            for idx, cam in enumerate(self.workspace.names.camera):
                observation_dict[cam] = {}
                x = []
                y = []
                z = []
                for img in self.intrinsic_dataset[cam].keys():
                    for board in self.boards:
                        cam_id = self.cameras.index(cam)
                        img_id = self.images.index(img)
                        board_id = self.boards.index(board)
                        if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            num_points = self.workspace.pose_table.num_points[cam_id][img_id][board_id]
                            error = self.workspace.pose_table.reprojection_error[cam_id][img_id][board_id]
                            pose = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                            rvec, tvec = split(from_matrix(pose))
                            rotation_deg = np.linalg.norm([rvec[0], rvec[1]]) * 180.0 / math.pi
                            x.append(rotation_deg)
                            y.append(num_points)
                            z.append(error)
                ## value output
                observation_dict[cam]['Total_view'] = len(y)              
                p = [i for i in y if i>=20]                               
                observation_dict[cam]['num_points>20'] = len(p)           
                observation_dict[cam]['percentile'] = np.percentile(y, 75)


                # observation_dict[cam]['Detected_point_variation'] = max(y) - min(y)
                # observation_dict[cam]['std'] = np.std(y)
                # observation_dict[cam]['num_view'] = len(y)
                # observation_dict[cam]['mean'] = np.mean(y)
                ## end
                bin = np.arange(0, 100, 5)
                if idx < int(self.workspace.sizes.camera/2):
                    axs[0, idx].hist(y, bin, edgecolor='black')
                    axs2[0, idx].scatter(x,y, s=5)
                    axs[0, idx].set_title('Cam-'+cam)
                    axs2[0, idx].set_title('Cam-'+cam)
                else:
                    i = idx - math.ceil(self.workspace.sizes.camera/2)
                    axs[1, i].hist(y, bin, edgecolor='black')
                    axs2[1, i].scatter(x,y, s=5)
                    axs[1, i].set_title('Cam-'+cam)
                    axs2[1, i].set_title('Cam-'+cam)
                for ax, ax2 in zip(axs.flat, axs2.flat):
                    # ax.set(xlabel='View Angle(degrees)', ylabel='Number of Points')
                    ax.set(xlabel='Detected Points', ylabel='Number of Views')
                    ax2.set(xlabel='View Angle(degrees)', ylabel='Number of Points')

                # Hide x labels and tick labels for top plots and y ticks for right plots.
                for ax, ax2 in zip(axs.flat, axs2.flat):
                    ax.label_outer()
                    ax2.label_outer()
            folder = self.base_path[-3:]
            path = os.path.join(self.base_path, folder+'_intrinsic_numPoints_viz1.png')
            fig.savefig(path)
            path2 = os.path.join(self.base_path, folder+'_intrinsic_numPoints_viz2.png')
            fig2.savefig(path2)
            # plt.savefig(path)
            plt.show()

            df = pd.DataFrame(observation_dict)
            excel_path = os.path.join(self.base_path, folder+'_intrinsic_numPoints_viz1.xlsx')
            df.to_excel(excel_path, sheet_name='Intrinsic')

            
        elif self.view == 'pose_table' or self.view == 'bundle_adjustment':
            observation_dict = {}
            fig, axs = plt.subplots(2, math.ceil(self.workspace.sizes.camera/2))
            fig2, axs2 = plt.subplots(2, math.ceil(self.workspace.sizes.camera/2))
            for idx, cam in enumerate(self.workspace.names.camera):
                observation_dict[cam] = {}
                x = []
                y = []
                z = []
                if self.view == 'pose_table':
                    images = self.images
                elif self.view == 'bundle_adjustment':
                    images = self.final_images
                for img in images:
                    for board in self.boards:
                        cam_id = self.cameras.index(cam)
                        img_id = self.images.index(img)
                        board_id = self.boards.index(board)
                        if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            num_points = self.workspace.pose_table.num_points[cam_id][img_id][board_id]
                            error = self.workspace.pose_table.reprojection_error[cam_id][img_id][board_id]
                            pose = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                            rvec, tvec = split(from_matrix(pose))
                            rotation_deg = np.linalg.norm([rvec[0], rvec[1]]) * 180.0 / math.pi
                            x.append(rotation_deg)
                            y.append(num_points)
                            z.append(error)
                ## value output
                # observation_dict[cam]['Detected_point_variation'] = max(y) - min(y)
                # observation_dict[cam]['std'] = np.std(y)
                observation_dict[cam]['Total_view'] = len(y)
                p = [i for i in y if i>=20]
                observation_dict[cam]['num_points>20'] = len(p)
                observation_dict[cam]['percentile'] = np.percentile(y, 75)
                ## end
                bin = np.arange(0, 100, 5)
                if idx < int(self.workspace.sizes.camera/2):
                    axs[0, idx].hist(y, bin, edgecolor='black')
                    axs2[0, idx].scatter(x,y, s=5)
                    axs[0, idx].set_title('Cam-'+cam)
                    axs2[0, idx].set_title('Cam-'+cam)
                else:
                    i = idx - math.ceil(self.workspace.sizes.camera/2)
                    axs[1, i].hist(y, bin, edgecolor='black')
                    axs2[1, i].scatter(x,y, s=5)
                    axs[1, i].set_title('Cam-'+cam)
                    axs2[1, i].set_title('Cam-'+cam)
                for ax, ax2 in zip(axs.flat, axs2.flat):
                    # ax.set(xlabel='View Angle(degrees)', ylabel='Number of Points')
                    ax.set(xlabel='Detected Points', ylabel='Number of Views')
                    ax2.set(xlabel='View Angle(degrees)', ylabel='Number of Points')

                # Hide x labels and tick labels for top plots and y ticks for right plots.
                for ax, ax2 in zip(axs.flat, axs2.flat):
                    ax.label_outer()
                    ax2.label_outer()

            if self.view == 'pose_table':
                folder = self.base_path[-3:]
                path = os.path.join(self.base_path, folder+'_numPoints_viz1.png')
                fig.savefig(path)
                path2 = os.path.join(self.base_path, folder+'_numPoints_viz2.png')
                fig2.savefig(path2)
                # plt.savefig(path)
                plt.show()

                df = pd.DataFrame(observation_dict)
                excel_path = os.path.join(self.base_path, folder+'_numPoints_viz1.xlsx')
                df.to_excel(excel_path, sheet_name='Extrinsic')
            elif self.view == 'bundle_adjustment':
                folder = self.base_path[-3:]
                path = os.path.join(self.base_path, folder + '_bundle_numPoints_viz1.png')
                fig.savefig(path)
                path2 = os.path.join(self.base_path, folder + '_bundle_numPoints_viz2.png')
                fig2.savefig(path2)
                # plt.savefig(path)
                plt.show()

                df = pd.DataFrame(observation_dict)
                excel_path = os.path.join(self.base_path, folder + '_bundle_numPoints_viz1.xlsx')
                df.to_excel(excel_path, sheet_name='bundle')


        # data = {'rotation_deg': x, 'num_points': y, 'error': z}
        # df = pd.DataFrame(data)
        # fig = px.scatter_3d(df, x='rotation_deg', y='num_points', z='error', title=cam)
        # fig.show()

    def hist3D(self, x, y):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # x, y = np.random.rand(2, 100) * 4
        hist, xedges, yedges = np.histogram2d(x, y, bins=5, range=[[0, 100], [0, 100]])

        # Construct arrays for the anchor positions of the 16 bars.
        # Note: np.meshgrid gives arrays in (ny, nx) so we use 'F' to flatten xpos,
        # ypos in column-major order. For numpy >= 1.7, we could instead call meshgrid
        # with indexing='ij'.
        # xpos, ypos = np.meshgrid(xedges[:-1] + 0.25, yedges[:-1] + 0.25)
        xpos, ypos = np.meshgrid(xedges[:-1] + 0.01, yedges[:-1] + 0.01)
        xpos = xpos.flatten('F')
        ypos = ypos.flatten('F')
        zpos = np.zeros_like(xpos)

        # Construct arrays with the dimensions for the 16 bars.
        dx = 0.5 * np.ones_like(zpos)
        dy = dx.copy()
        dz = hist.flatten()

        cmap = cm.get_cmap('jet') # Get desired colormap
        max_height = np.max(dz)   # get range of colorbars
        min_height = np.min(dz)

        # scale each z to [0,1], and get their rgb values
        rgba = [cmap((k-min_height)/max_height) for k in dz]

        ax.set(xlabel='View Angle(degrees)', ylabel='Number of Points')
        # ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color=rgba, zsort='average')
        ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color=rgba)

        plt.show()

        pass

    def draw_cube(self,frame, cam, rvec, tvec):
        cam_matrix = np.array(self.initial_calibration['cameras'][cam]['K'])
        cam_dist = np.array(self.initial_calibration['cameras'][cam]['dist'])
        axis = np.float32([[3 * .013, 0, 0], [0, 3 * .013, 0], [0, 0, -3 * .013]]).reshape(-1, 3)
        axis_cube = np.float32([[0, 0, 0], [0, 3 * 0.013, 0], [3 * 0.013, 3 * 0.013, 0], [3 * 0.013, 0, 0],
                                [0, 0, -3 * 0.013], [0, 3 * 0.013, -3 * 0.013], [3 * 0.013, 3 * 0.013, -3 * 0.013],
                                [3 * 0.013, 0, -3 * 0.013]])
        imgpts, jac = cv2.projectPoints(axis_cube, rvec, tvec, cam_matrix, cam_dist)
        # cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvec, tvec, 0.1)
        img = np.float64(frame)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        # draw ground floor in green
        # img = cv2.drawContours(img, [imgpts[:4]],-1,(0,255,0),-3)
        # draw pillars in blue color
        for i, j in zip(range(4), range(4, 8)):
            img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 10)
        # draw top layer in red color
        img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 10)
        return img

    def view_error_viz(self, cam):
        if self.view == 'bundle_adjustment':
            images = self.final_images
            x = []
            y = []
            err = []
            im_size = self.initial_calibration['cameras'][cam]['image_size']
            frame = np.ones((im_size[1], im_size[0], 3))*255
            for img in images:
                img_id = self.images.index(img)
                cam_id = self.cameras.index(cam)
                for board_id, board in enumerate(self.boards):
                    corners_ids = np.flatnonzero(self.workspace.point_table.valid[cam_id][img_id][board_id])
                    if len(corners_ids)!= 0:
                        pose = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                        rvec, tvec = rtvec.split(rtvec.from_matrix(pose))
                        frame = self.draw_cube(frame, cam, rvec, tvec)
                    corners = [self.workspace.point_table.points[cam_id][img_id][board_id][c] for c in corners_ids]
                    width = [c[0] for c in corners]
                    height = [c[1] for c in corners]
                    x.extend(width)
                    y.extend(height)
                    points_value = [self.workspace.pose_table.reprojection_error[cam_id][img_id][board_id]]*len(corners)
                    err.extend(points_value)

            folder = self.base_path[-3:]
            path = os.path.join(self.base_path, folder+'_view.png')
            cv2.imwrite(path, frame)
            fig = px.scatter(x=x, y=y, color=err)
            fig.update_layout(
                xaxis_title="Image width", yaxis_title="Image height"
            )
            # plt.scatter(x, y, c=err)
            # plt.show()
            fig.show()

        pass

    def rotation_translation_viz(self):
        if self.view == 'intrinsic':
            observation_dict = {}
            fig, axs = plt.subplots(2, math.ceil(self.workspace.sizes.camera/2))
            for idx, cam in enumerate(self.workspace.names.camera):
                observation_dict[cam] = {}
                rotation_list = []
                camera_list = []
                translation_list = []
                for img in self.intrinsic_dataset[cam].keys():
                    for board in self.intrinsic_dataset[cam][img]:
                        cam_id = self.cameras.index(cam)
                        img_id = self.images.index(img)
                        board_id = self.boards.index(board)
                        pose = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                        r, t = (matrix.split(pose))
                        rvec, tvec = split(from_matrix(pose))
                        rotation_deg = np.linalg.norm([rvec[0], rvec[1]]) * 180.0 / math.pi
                        translation = np.linalg.norm(t)
                        rotation_list.append(rotation_deg)
                        translation_list.append(translation)
                        camera_list.append(cam)

                ## value output
                observation_dict[cam]['angle_variation'] = max(rotation_list) - min(rotation_list)
                observation_dict[cam]['std'] = np.std(rotation_list)
                observation_dict[cam]['num_view'] = len(rotation_list)
                observation_dict[cam]['mean'] = np.mean(rotation_list) 
                ## end
                bin = np.arange(0, 120, 5)
                if idx < math.ceil(self.workspace.sizes.camera/2):
                    axs[0, idx].hist(rotation_list, bin, edgecolor='black')
                    axs[0, idx].set_title('Cam-'+cam)
                else:
                    i = idx - math.ceil(self.workspace.sizes.camera/2)
                    axs[1, i].hist(rotation_list, bin, edgecolor='black')
                    axs[1, i].set_title('Cam-'+cam)
                for ax in axs.flat:
                    ax.set(xlabel='View Angle(degrees)', ylabel='Number of Views')

                # Hide x labels and tick labels for top plots and y ticks for right plots.
                for ax in axs.flat:
                    ax.label_outer()
            folder = self.base_path[-3:]
            path = os.path.join(self.base_path, folder+'-intrinsic_viz.png')
            plt.savefig(path)
            plt.show()

            df = pd.DataFrame(observation_dict)
            excel_path = os.path.join(self.base_path, folder+'-intrinsic_viz.xlsx')
            df.to_excel(excel_path, sheet_name='Intrinsic')



            # json_object = json.dumps(observation_dict, indent=4)
            # # Writing to sample.json
            # json_path = os.path.join(self.base_path, folder+'-intrinsic_viz.json')
            # with open(json_path, "w") as outfile:
            #     outfile.write(json_object)

        elif self.view == 'pose_table' or self.view == 'bundle_adjustment':
            observation_dict = {}
            fig, axs = plt.subplots(2, math.ceil(self.workspace.sizes.camera/2))
            for idx, cam in enumerate(self.workspace.names.camera):
                observation_dict[cam] = {}
                rotation_list = []   
                camera_list = []     
                translation_list = []
                num_point_list = []
                if self.view == 'pose_table':
                    images = self.images
                elif self.view == 'bundle_adjustment':
                    images = self.final_images
                for img in images:
                    for board in self.boards:
                        cam_id = self.cameras.index(cam)
                        img_id = self.images.index(img)
                        board_id = self.boards.index(board)
                        if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            pose = self.workspace.pose_table.poses[cam_id][img_id][board_id]
                            r, t = (matrix.split(pose))
                            rvec, tvec = split(from_matrix(pose))
                            rotation_deg = np.linalg.norm([rvec[0], rvec[1]]) * 180.0 / math.pi
                            translation = np.linalg.norm(t)
                            rotation_list.append(rotation_deg)
                            translation_list.append(translation)
                            num_points = self.workspace.pose_table.num_points[cam_id][img_id][board_id]
                            num_point_list.append(num_points)
                            camera_list.append(cam)
                ## value output
                observation_dict[cam]['angle_variation'] = max(rotation_list) - min(rotation_list)
                observation_dict[cam]['std'] = np.std(rotation_list)
                observation_dict[cam]['num_view'] = len(rotation_list)
                observation_dict[cam]['mean'] = np.mean(rotation_list)
                ## end

                bin = np.arange(0, 120, 5)
                if idx < int(self.workspace.sizes.camera/2):
                    axs[0, idx].hist(rotation_list, bin, edgecolor='black')
                    axs[0, idx].set_title('Cam-'+cam)
                else:
                    i = idx - math.ceil(self.workspace.sizes.camera/2)
                    axs[1, i].hist(rotation_list, bin, edgecolor='black')
                    axs[1, i].set_title('Cam-'+cam)
                for ax in axs.flat:
                    ax.set(xlabel='View Angle(degrees)', ylabel='Number of Views')

                # Hide x labels and tick labels for top plots and y ticks for right plots.
                for ax in axs.flat:
                    ax.label_outer()

            if self.view == 'pose_table':
                folder = self.base_path[-3:]
                path = os.path.join(self.base_path, folder + '-poseTable_viz.png')
                plt.savefig(path)
                plt.show()

                df = pd.DataFrame(observation_dict)
                excel_path = os.path.join(self.base_path, folder + '-poseTable_viz.xlsx')
                df.to_excel(excel_path, sheet_name='Extrinsic')
            elif self.view == 'bundle_adjustment':
                folder = self.base_path[-3:]
                path = os.path.join(self.base_path, folder + '-bundle_viz.png')
                plt.savefig(path)
                plt.show()

                df = pd.DataFrame(observation_dict)
                excel_path = os.path.join(self.base_path, folder + '-bundle_viz.xlsx')
                df.to_excel(excel_path, sheet_name='bundle')

            # json_object = json.dumps(observation_dict, indent=4)
            # # Writing to sample.json
            # json_path = os.path.join(self.base_path, folder+'-poseTable_viz.json')
            # with open(json_path, "w") as outfile:
            #     outfile.write(json_object)
        # # for plotly plot
        # folder = self.base_path[-3:]
        # data = {'x': camera_list, 'y': rotation_list, 'cameras': camera_list}
        # df = pd.DataFrame(data)
        # fig = px.scatter(df, x='x', y='y', color='cameras',
        #                  labels={"x": "Cameras",
        #                          "y": "Calibration Board Rotation(degrees)",
        #                          "cameras": "Cameras"},
        #                  width=1000, height=1000)
        # fig.update_traces(marker_size=10)
        # fig.update_layout(legend=dict(font=dict(size=20)))
        # fig.update_layout(font={'size': 20}, title=folder+'-'+self.view)
        # fig.show()
        #
        # data = {'x': camera_list, 'y': translation_list, 'cameras': camera_list}
        # df = pd.DataFrame(data)
        # fig = px.scatter(df, x='x', y='y', color='cameras',
        #                  labels={"x": "Cameras",
        #                          "y": "Translation(meters)",
        #                          "cameras": "Cameras"},
        #                  width=1000, height=1000)
        # fig.update_traces(marker_size=10)
        # fig.update_layout(legend=dict(font=dict(size=20)))
        # fig.update_layout(font={'size': 20}, title=folder+'-'+self.view)
        # fig.show()

        pass

    def extrinsic_viz(self, cam):
        i = Interactive_Extrinsic(self.base_path)

    def load_bundleAdjustment(self, cam):
        self.clearLayout(self.gridLayout1[cam])
        self.clearLayout(self.gridLayout2[cam])
        self.clearLayout(self.gridLayout3[cam])
        if self.workspace_test:
            self.view = 'bundle_adjustment'
            self.table[cam] = QTableWidget()
            self.table[cam].cellClicked.connect(partial(self.cell_was_clicked, cam, self.gridLayout1[cam]))

            first_im = self.final_images[0]
            self.pose_count[cam] = self.images.index(first_im)
            self.set_image_dropDown(cam)
            self.set_viewer(cam, self.gridLayout1[cam], self.folder_path[cam], self.images[self.pose_count[cam]],
                            self.table[cam], self.gridLayout3[cam], self.gridLayout2[cam])

            self.tab_num[cam].setLayout(self.tab_num[cam].layout)
        else:
            self.label1 = QLabel(self.tab_num[cam])
            self.label1.setObjectName(self.view)
            self.label1.setText('Error: workspace_test.pkl not available')
            self.label1.setStyleSheet("border: 1px solid black;")
            self.label1.setGeometry(QRect(240, 0, 120, 28))
            self.gridLayout2[cam].addWidget(self.label1, 2, 0)
        pass

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
        if self.view == 'bundle_adjustment':
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
        elif self.view == 'bundle_adjustment':
            images = self.final_images
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
        table.setColumnCount(3)
        table.setHorizontalHeaderLabels(['Num_Points', "Reprojection Error", "View Angles"])
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
                # item4.setText("")
                # table.setItem(board_id, 3, item4)
            else:
                # text = 'num_points: ' + str(num_points) + ' | ' + 'Repo_error: ' + str(repo_error)
                item1.setText(str(num_points))
                table.setItem(board_id, 0, item1)
                item2.setText(str(repo_error))
                table.setItem(board_id, 1, item2)
                item3.setText(str(viewAngles))
                table.setItem(board_id, 2, item3)
                # item4.setText(str(0))
                # table.setItem(board_id, 3, item4)
            pass
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeMode.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeMode.ResizeToContents)
        # header.setSectionResizeMode(3, QHeaderView.ResizeMode.ResizeToContents)
        tableLayout.addWidget(table)

    def collect_inlier_dataset(self):
        for cam in self.cameras:
            file = os.path.join(self.base_path, 'calibration_'+ cam + '.pkl')
            if os.path.exists(file):
                data = pickle.load(open(file, "rb"))
                self.inlier_mask[cam] = data.calibrations['calibration'].point_table
        pass

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
                self.workspace = pickle.load(open(workspace_path, "rb"))
                self.cameras = self.workspace.names.camera
                self.images = self.workspace.names.image
                self.boards = self.workspace.names.board
                self.set_Cam_color()
                self.collect_inlier_dataset()
                if "workspace_test.pkl" in files:
                    path = os.path.join(self.base_path, "workspace_test.pkl")
                    self.workspace_test = pickle.load(open(path, "rb"))
                    self.final_images = self.workspace_test.names.image
                if "Calibration_handeye.json" in files:
                    path = os.path.join(self.base_path, "Calibration_handeye.json")
                    self.initial_calibration = json.load(open(path))
                elif "Calibration_handeye.json" not in files and 'calibration.json' in files:
                    path = os.path.join(self.base_path, "calibration.json")
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

