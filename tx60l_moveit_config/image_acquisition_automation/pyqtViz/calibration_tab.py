import os.path
from .cameraWindow import *
# from calibrtaion_tab import *
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
import pandas as pd
import plotly.express as px
# from PyQt6.QtCore import Qt, QRectF, QPoint, QPointF, pyqtSignal, QEvent, QSize, QRect
# from PyQt6.QtGui import QImage, QPixmap, QPainterPath, QMouseEvent, QPainter, QPen, QColor
# from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QFileDialog, QSizePolicy, \
#     QGraphicsItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsLineItem, QGraphicsPolygonItem, QTableWidget, \
#     QTableWidgetItem
# from PyQt6.QtWidgets import QApplication, QMainWindow, QSpinBox, QWidget, QPushButton, QTextEdit, QVBoxLayout, \
#     QHBoxLayout, QGridLayout, QLineEdit, QLabel, QTabWidget, QScrollArea, QTextBrowser, QCheckBox
# from PyQt6.QtWidgets import *
# from PyQt6.QtGui import *

from src.multical_scripts.handEye_viz import *
from src.multical_scripts.board_angle import *
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

from .operation_tab import *
import cv2
import math
import matplotlib.pyplot as plt
from src.multical_scripts.handEye_check import *

class Calibration(QWidget):
    def __init__(self):
        super().__init__()

        self.layout = QVBoxLayout(self)
        self.Current_poseCount = 0
        self.Last_poseCount = 0
        self.camera_color = {}
        self.cb = QComboBox(self)
        self.cb1 = QComboBox(self)
        self.cb2 = QComboBox(self)
        self.handEyeCamera = None
        self.workspace = None
        self.folder_path = None
        self.cameras = None
        self.images = None
        self.boards = None
        self.detectedPoints = None
        self.intrinsic = None
        self.selected_pose = []
        self.image_checkBox = {}
        self.camera_groups = {}

        self.btnLoad1 = QPushButton(self)
        self.btnLoad1.setObjectName('Load')
        self.btnLoad1.setText('Load')
        self.btnLoad1.clicked.connect(self.open_dir_dialog)
        self.btnLoad1.setGeometry(QRect(0, 0, 93, 28))

        self.cb.setGeometry(QRect(95, 0, 400, 28))
        self.cb.currentIndexChanged.connect(self.selectionchange)

        self.btnLoad2 = QPushButton(self)
        self.btnLoad2.setObjectName('<')
        self.btnLoad2.setText('<')
        self.btnLoad2.clicked.connect(self.loadPrevious)
        self.btnLoad2.setGeometry(QRect(495, 0, 30, 28))

        self.btnLoad3 = QPushButton(self)
        self.btnLoad3.setObjectName('>')
        self.btnLoad3.setText('>')
        # self.btnLoad6.setStyleSheet("background-color : cyan;")
        self.btnLoad3.clicked.connect(self.loadNext)
        self.btnLoad3.setGeometry(QRect(618, 0, 30, 28))

        self.btnLoad4 = QPushButton(self)
        self.btnLoad4.setObjectName('Show visualization')
        self.btnLoad4.setText('Show visualization')
        self.btnLoad4.clicked.connect(self.showviz)
        self.btnLoad4.setGeometry(QRect(650, 0, 150, 28))

        self.btnLoad5 = QPushButton(self)
        self.btnLoad5.setObjectName('Export Matplotlib')
        self.btnLoad5.setText('Export Matplotlib')
        self.btnLoad5.clicked.connect(self.export_matplotlib)
        self.btnLoad5.setGeometry(QRect(800, 0, 140, 28))

        self.cb1.setGeometry(QRect(940, 0, 250, 28))
        self.cb1.currentIndexChanged.connect(self.selectionchange2)

        self.cb2.setGeometry(QRect(1190, 0, 250, 28))
        self.cb2.currentIndexChanged.connect(self.selectionchange3)

        self.label1 = QLabel(self)
        self.label1.setObjectName('Pose')
        self.label1.setText('Pose')
        self.label1.setStyleSheet("border: 1px solid black;")
        self.label1.setGeometry(QRect(525, 0, 93, 28))

        self.btnLoad6 = QPushButton(self)
        self.btnLoad6.setObjectName('Hand-Eye Calibration')
        self.btnLoad6.setText('Hand-Eye Calibration')
        self.btnLoad6.clicked.connect(self.handEye_Calibration)
        self.btnLoad6.setGeometry(QRect(0, 100, 150, 28))

        self.label3 = QLabel(self)
        self.label3.setObjectName('Result')
        self.label3.setText('Result')
        self.label3.setStyleSheet("border: 1px solid black;")
        self.label3.setGeometry(QRect(153, 100, 200, 28))

        # Grid for images
        self.gridLayoutWidget1 = QWidget(self)
        self.gridLayoutWidget1.setGeometry(QRect(0, 130, 1880, 500))
        self.gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.gridLayout1 = QGridLayout(self.gridLayoutWidget1)
        self.gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout1.setObjectName("gridLayout")

        # Grid for image check boxes
        self.gridLayoutWidget2 = QWidget(self)
        self.gridLayoutWidget2.setGeometry(QRect(0, 30, 1880, 60))
        self.gridLayoutWidget2.setObjectName("gridLayoutWidget")
        self.gridLayout2 = QGridLayout(self.gridLayoutWidget2)
        self.gridLayout2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout2.setObjectName("gridLayout")



        # Grid for table
        self.gridLayoutWidget3 = QWidget(self)
        self.gridLayoutWidget3.setGeometry(QRect(0, 630, 700, 700))
        self.gridLayoutWidget3.setObjectName("gridLayoutWidget")
        self.gridLayout3 = QGridLayout(self.gridLayoutWidget3)
        self.gridLayout3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout3.setObjectName("gridLayout")
        self.table = QTableWidget()
        self.gridLayout3.addWidget(self.table)

        self.setLayout(self.layout)

    def handEye_Calibration(self):
        master_cam = self.workspace.names.camera.index(self.cam_num)
        m_board = self.handEyeCamera[self.cam_num][str(self.cam_group)]['master_board']
        master_board = self.workspace.names.board.index(m_board)
        s_cam = self.handEyeCamera[self.cam_num][str(self.cam_group)]['slave_cam']
        slave_cam = self.workspace.names.camera.index(s_cam)
        s_board = self.handEyeCamera[self.cam_num][str(self.cam_group)]['slave_board']
        slave_board = self.workspace.names.board.index(s_board)
        masterR_list = []
        masterT_list = []
        slaveR_list = []
        slaveT_list = []
        for img in self.selected_pose:
            img_id = self.workspace.names.image.index(img)
            master_pose = np.linalg.inv(self.workspace.pose_table.poses[master_cam][img_id][master_board])
            master_R, master_t = matrix.split(master_pose)
            slave_pose = np.linalg.inv(self.workspace.pose_table.poses[slave_cam][img_id][slave_board])
            slave_R, slave_t = matrix.split(slave_pose)
            masterR_list.append(master_R)
            masterT_list.append(master_t)
            slaveR_list.append(slave_R)
            slaveT_list.append(slave_t)

        h = handEye(self.folder_path)
        slaveCam_wrt_masterCam, slaveB_wrt_masterB, masterCam_wrt_masterB, slaveCam_wrt_slaveB, \
        estimated_slaveB_slaveCam, err, err2 = h.hand_eye_robot_world(np.array(masterR_list),
                                                                      np.array(masterT_list), np.array(slaveR_list), np.array(slaveT_list))
        self.label3.setText('handeye possible')

    def group_decode(self, group):
        group_num = {}
        total = 0
        for cam_num, cam_group in self.handEyeCamera.items():
            group_num[cam_num] = np.arange(total, total+len(self.handEyeCamera[cam_num]))
            total += len(self.handEyeCamera[cam_num])
            if group in group_num[cam_num]:
                return cam_num, np.where(group_num[cam_num]==group)[0][0]
        pass


    def selectionchange2(self, group):
        '''
        this shows scatter plot of rvec
        '''
        camera_num = self.workspace.sizes.camera
        master_cam_id = math.floor(group/camera_num)
        slave_cam_id = group - master_cam_id*camera_num
        master_cam = self.workspace.names.camera[master_cam_id]
        slave_cam = self.workspace.names.camera[slave_cam_id]
        if self.camera_groups:
            camera_groups = self.camera_groups[master_cam][slave_cam]

            print(master_cam, slave_cam)

            final_layout = go.Figure()
            for group in camera_groups.keys():
                master_x = []
                master_y = []
                master_z = []
                master_name = []
                slave_x = []
                slave_y = []
                slave_z = []
                slave_name = []
                for k in camera_groups[group]['masterBoard_angle'].keys():
                    master_x.append(camera_groups[group]['masterBoard_angle'][k][0])
                    master_y.append(camera_groups[group]['masterBoard_angle'][k][1])
                    master_z.append(camera_groups[group]['masterBoard_angle'][k][2])
                    master_name.append(group)
                    slave_x.append(camera_groups[group]['slaveBoard_angle'][k][0])
                    slave_y.append(camera_groups[group]['slaveBoard_angle'][k][1])
                    slave_z.append(camera_groups[group]['slaveBoard_angle'][k][2])
                    slave_name.append(group)

                nameM = 'M-' + master_cam + ' group-' + str(group)

                final_layout.add_trace(
                    go.Scatter3d(
                        x=master_x,
                        y=master_y,
                        z=master_z,
                        mode='markers',
                        text=master_name, textposition="bottom center",
                        name=nameM
                    )
                )
                nameS = 'S-' + slave_cam + ' group-' + str(group)
                final_layout.add_trace(
                    go.Scatter3d(
                        x=slave_x,
                        y=slave_y,
                        z=slave_z,
                        mode='markers',
                        text=slave_name, textposition="bottom center",
                        name=nameS
                    )
                )
            final_layout.update_layout(scene=dict(
                xaxis_title='Roll',
                yaxis_title='Pitch',
                zaxis_title='Yaw'))
            final_layout.show()

    def set_Cam_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime', 'pink', 'teal', 'darkcyan', 'violet', 'brown', 'indigo']
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]

    def selectionchange3(self, group):
        '''
        shows magnitude of rvec and tvec
        '''
        camera_num = self.workspace.sizes.camera
        master_cam = self.workspace.names.camera[group]
        if self.camera_groups:
            angle_dict = {}
            translation_dict = {}
            angle_dict[master_cam] = []
            translation_dict[master_cam] = []
            camera_groups = self.camera_groups[master_cam]
            for slave_cam, groups in camera_groups.items():
                angle_dict[slave_cam] = []
                translation_dict[slave_cam] = []
                img_list = []
                for g in groups:
                    all_images = camera_groups[slave_cam][g]['masterBoard_pose']
                    for img in all_images:
                        if img not in img_list:
                            # img_list.append(img)
                            master_pose = np.array(camera_groups[slave_cam][g]['masterBoard_pose'][img])
                            slave_pose = np.array(camera_groups[slave_cam][g]['slaveBoard_pose'][img])
                            r, t = (matrix.split(master_pose))
                            # rotation_deg = R.magnitude(R.from_matrix(r)) * 180.0 / math.pi
                            rvec, tvec = split(from_matrix(master_pose))
                            rotation_deg = np.linalg.norm([rvec[0], rvec[1]])* 180.0 / math.pi
                            translation = np.linalg.norm(t)
                            angle_dict[master_cam].append(rotation_deg)
                            translation_dict[master_cam].append(translation)
                            r, t = (matrix.split(slave_pose))
                            # rotation_deg = R.magnitude(R.from_matrix(r)) * 180.0 / math.pi
                            rvec, tvec = split(from_matrix(slave_pose))
                            rotation_deg = np.linalg.norm([rvec[0], rvec[1]])* 180.0 / math.pi
                            translation = np.linalg.norm(t)
                            angle_dict[slave_cam].append(rotation_deg)
                            translation_dict[slave_cam].append(translation)

            x_rot = []
            y_rot = []
            colour_rot = []
            x_T = []
            y_T = []
            colour_T = []
            for cam_id, cam_name in enumerate(self.workspace.names.camera):
                if cam_name in angle_dict.keys():
                    y_rot.extend(angle_dict[cam_name])
                    x_rot.extend([cam_name]*len(angle_dict[cam_name]))
                    colour_rot.extend([cam_name]*len(angle_dict[cam_name]))
                    y_T.extend(translation_dict[cam_name])
                    x_T.extend([cam_name] * len(translation_dict[cam_name]))
                    colour_T.extend([cam_name] * len(translation_dict[cam_name]))
                else:
                    y_rot.append(0)
                    x_rot.append(cam_name)
                    colour_rot.append(cam_name)
                    y_T.append(0)
                    x_T.append(cam_name)
                    colour_T.append(cam_name)
            folder = self.folder_path[-3:]
            data = {'x': x_rot, 'y': y_rot, 'cameras':colour_rot}
            df = pd.DataFrame(data)
            fig = px.scatter(df, x='x', y='y', color='cameras',
                    labels={"x": "Cameras",
                     "y": "Calibration Board Rotation(degrees)",
                     "cameras": "Cameras"},
                    width=1000, height=1000)
            fig.update_traces(marker_size=10)
            fig.update_layout(legend=dict(font=dict(size=20)))
            fig.update_layout(font={'size': 20}, title=folder + '-MasterCam-'+master_cam)
            fig.show()

            # translation
            data = {'x': x_T, 'y': y_T, 'cameras': colour_T}
            df = pd.DataFrame(data)
            fig = px.scatter(df, x='x', y='y', color='cameras',
                             labels={"x": "Cameras",
                                     "y": "Translation(meter)",
                                     "cameras": "Cameras"},
                             width=1000, height=1000)
            fig.update_traces(marker_size=10)
            fig.update_layout(legend=dict(font=dict(size=20)))
            fig.update_layout(font={'size': 20}, title=folder+ ' MasterCam-'+master_cam)
            fig.show()

        pass

    def selectionchange(self, group, poseCount=0):
        self.clearLayout(self.gridLayout1)
        self.clearLayout(self.gridLayout2)
        if poseCount == 0:
            self.Current_poseCount = 0
        self.cam_num, self.cam_group = self.group_decode(group)

        self.group = group
        handEye_Cam = self.handEyeCamera[self.cam_num][str(self.cam_group)]
        self.Last_poseCount = len(handEye_Cam["image_list"])
        master_cam = handEye_Cam["master_cam"]
        master_board = handEye_Cam["master_board"]
        slave_cam = handEye_Cam["slave_cam"]
        slave_board = handEye_Cam["slave_board"]
        image_list = handEye_Cam["image_list"]
        ## add image check box
        select_allImage = QCheckBox(text='Select all')
        select_allImage.stateChanged.connect(partial(self.seletedImage, 'all'))
        select_allImage.setGeometry(QRect(0, 60, 30, 28))
        self.gridLayout2.addWidget(select_allImage, 0, 0)
        for idx, img in enumerate(image_list):
            self.image_checkBox[img] = QCheckBox(text=img)
            self.image_checkBox[img].stateChanged.connect(partial(self.seletedImage, img))
            self.image_checkBox[img].setGeometry(QRect(0, 60, 30, 28))
            if idx<15:
                self.gridLayout2.addWidget(self.image_checkBox[img], 0, idx+1)
            else:
                self.gridLayout2.addWidget(self.image_checkBox[img], 1, idx-15 + 1)
        ## end
        master_path = os.path.join(self.folder_path, master_cam, image_list[poseCount])
        slave_path = os.path.join(self.folder_path, slave_cam, image_list[poseCount])
        pose = 'Pose '+ image_list[poseCount][1:-4]
        self.label1.setText(pose)
        imageLabel1 = self.image_load(master_path, master_cam, master_board, image_list[poseCount])
        imageLabel2 = self.image_load(slave_path, slave_cam, slave_board, image_list[poseCount])
        # self.clearLayout(layout)
        self.gridLayout1.addWidget(imageLabel1, 1, 0)
        self.gridLayout1.addWidget(imageLabel2, 1, 1)

        self.calibrationTab_Table()
        # print(self.group)
        pass

    def seletedImage(self, image):
        handEye_Cam = self.handEyeCamera[self.cam_num][str(self.cam_group)]
        if image == 'all':
            image_list = handEye_Cam["image_list"]
            self.selected_pose = image_list
            state = [self.image_checkBox[img].setChecked(True) for img in image_list]
        else:
            state = self.image_checkBox[image].isChecked()
            if state:
                if image not in self.selected_pose:
                    self.selected_pose.append(image)
            else:
                if image in self.selected_pose:
                    self.selected_pose.remove(image)

        pass

    def calibrationTab_Table(self):
        self.table.setRowCount(7)
        self.table.setColumnCount(1)
        self.table.setHorizontalHeaderLabels(["Values"])
        self.table.setVerticalHeaderLabels(["Master Cam", "Master Board", "Slave Cam",
                                                 "Slave Board", "Num_poses", "Initial_Reprojection_Error", "Final_Reprojection_Error"])
        item1 = QTableWidgetItem()
        item2 = QTableWidgetItem()
        item3 = QTableWidgetItem()
        item4 = QTableWidgetItem()
        item5 = QTableWidgetItem()
        item6 = QTableWidgetItem()
        item7 = QTableWidgetItem()
        handEye_Cam = self.handEyeCamera[self.cam_num][str(self.cam_group)]
        item1.setText(str(handEye_Cam["master_cam"]))
        item2.setText(str(handEye_Cam["master_board"]))
        item3.setText(str(handEye_Cam["slave_cam"]))
        item4.setText(str(handEye_Cam["slave_board"]))
        item5.setText(str(len(handEye_Cam["image_list"])))
        # item6.setText(str("{:.2f}".format(float(handEye_Cam["initial_reprojection_error"]))))
        # item7.setText(str("{:.2f}".format(float(handEye_Cam["final_reprojection_error"]))))
        self.table.setItem(0, 0, item1)
        self.table.setItem(1, 0, item2)
        self.table.setItem(2, 0, item3)
        self.table.setItem(3, 0, item4)
        self.table.setItem(4, 0, item5)
        # self.table.setItem(5, 0, item6)
        # self.table.setItem(6, 0, item7)

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        self.workspace_load()

        if self.handEyeCamera != None:
            for cam_num, cam_value in self.handEyeCamera.items():
                for k in cam_value.keys():
                    text = "Cam" + cam_num + " Group-" + str(k)
                    self.cb.addItem(text)

        for cam_id0, cam_name0 in enumerate(self.workspace.names.camera):
            text = 'MasterCam- '+cam_name0
            self.cb2.addItem(text)
            for cam_id1, cam_name1 in enumerate(self.workspace.names.camera):
                text = 'camM-'+ cam_name0 + '_to_' + 'camS-' + cam_name1
                self.cb1.addItem(text)

        self.organize_camGroups()
        pass

    def organize_camGroups(self):
        for cam_id0, cam_name0 in enumerate(self.workspace.names.camera):
            self.camera_groups[cam_name0] = {}
            for i, group_name in enumerate(self.handEyeCamera[cam_name0]):
                slave_cam = self.handEyeCamera[cam_name0][group_name]['slave_cam']
                if slave_cam not in self.camera_groups[cam_name0].keys():
                    self.camera_groups[cam_name0][slave_cam] = {}
                    # group = 0
                self.camera_groups[cam_name0][slave_cam][group_name] = {}
                self.camera_groups[cam_name0][slave_cam][group_name]['masterBoard_angle'] = \
                                                self.handEyeCamera[cam_name0][group_name]['masterBoard_angle']
                self.camera_groups[cam_name0][slave_cam][group_name]['slaveBoard_angle'] = \
                                                self.handEyeCamera[cam_name0][group_name]['slaveBoard_angle']
                self.camera_groups[cam_name0][slave_cam][group_name]['masterBoard_pose'] = \
                    self.handEyeCamera[cam_name0][group_name]['masterBoard_pose']
                self.camera_groups[cam_name0][slave_cam][group_name]['slaveBoard_pose'] = \
                    self.handEyeCamera[cam_name0][group_name]['slaveBoard_pose']

        pass

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                workspace_path = os.path.join(self.folder_path, [f for f in files if f == "workspace.pkl"][0])
                self.workspace = pickle.load(open( workspace_path, "rb"))
                self.cameras = self.workspace.names.camera
                self.images = self.workspace.names.image
                self.boards = self.workspace.names.board
                self.last_pose_count = len(self.images)
                self.set_Cam_color()
                for file in files:
                    if file == 'calibration.detections.pkl':
                        pickle_file = pickle.load(open(os.path.join(self.folder_path, 'calibration.detections.pkl'),'rb'))
                        self.detectedPoints = pickle_file.detected_points
                    if file == "calibration.json":
                        intrinsic_path = os.path.join(self.folder_path, 'calibration.json')
                        self.intrinsic = json.load(open(intrinsic_path))
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.folder_path, "handEyeCamera.json")
                        self.handEyeCamera = json.load(open(handEye_path))
        pass


    def loadNext(self):
        if self.Last_poseCount-1 >= self.Current_poseCount >= 0:
            self.clearLayout(self.gridLayout1)
            self.selectionchange(self.group, self.Current_poseCount)
            self.Current_poseCount += 1
        else:
            self.clearLayout(self.gridLayout1)
            print("Pose end")
            self.label1.setText('Pose')

    def loadPrevious(self):
        if self.Current_poseCount > 0:
            self.Current_poseCount -= 1
            self.clearLayout(self.gridLayout1)
            self.selectionchange(self.group, self.Current_poseCount)
        else:
            self.clearLayout(self.gridLayout1)
            self.label1.setText('Pose')

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def showviz(self):
        camera = str(self.cam_num)
        group = str(self.cam_group)
        self.draw_viz(camera, group)

    def export_matplotlib(self):
        images = self.handEyeCamera[self.cam_num][str(self.cam_group)]['image_list']
        masterBoard_angle = self.handEyeCamera[self.cam_num][str(self.cam_group)]['masterBoard_angle']
        slaveBoard_angle = self.handEyeCamera[self.cam_num][str(self.cam_group)]['slaveBoard_angle']
        mBoard_x = []
        mBoard_y = []
        mBoard_z = []
        sBoard_x = []
        sBoard_y = []
        sBoard_z = []
        for img in images:
            mBoard_x.append(masterBoard_angle[img][0])
            mBoard_y.append(masterBoard_angle[img][1])
            mBoard_z.append(masterBoard_angle[img][2])
            sBoard_x.append(slaveBoard_angle[img][0])
            sBoard_y.append(slaveBoard_angle[img][1])
            sBoard_z.append(slaveBoard_angle[img][2])

        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(mBoard_x, mBoard_y, mBoard_z, marker='o', label='Master Board')

        ax.scatter(sBoard_x, sBoard_y, sBoard_z, marker='^', label='Slave Board')
        ax.set_xlabel('Roll', fontweight='bold')
        ax.set_ylabel('Pitch', fontweight='bold')
        ax.set_zlabel('Yaw', fontweight='bold')
        ax.legend()
        plt.show()
        pass


    def draw_viz(self, master_cam, group):
        masterBoard_angles = self.handEyeCamera[master_cam][group]['masterBoard_angle']
        slaveBoard_angles = self.handEyeCamera[master_cam][group]['slaveBoard_angle']
        masterBoard_error = self.handEyeCamera[master_cam][group]['masterBoard_error']
        slaveBoard_error = self.handEyeCamera[master_cam][group]['slaveBoard_error']
        master_x = []
        master_y = []
        master_z = []
        master_name = []
        slave_x = []
        slave_y = []
        slave_z = []
        slave_name = []
        for k in masterBoard_angles.keys():
            master_x.append(masterBoard_angles[k][0])
            master_y.append(masterBoard_angles[k][1])
            master_z.append(masterBoard_error[k])
            master_name.append(k)
            slave_x.append(slaveBoard_angles[k][0])
            slave_y.append(slaveBoard_angles[k][1])
            slave_z.append(slaveBoard_error[k])
            slave_name.append(k)

        nameM = master_cam +' group-'+ str(group)
        final_layout = go.Figure()
        final_layout.add_trace(
            go.Scatter3d(
                x=master_x,
                y=master_y,
                z=master_z,
                mode = 'markers',
                text=master_name, textposition="bottom center",
                name=nameM
            )
        )
        nameS = self.handEyeCamera[master_cam][group]['slave_cam']
        final_layout.add_trace(
            go.Scatter3d(
                x=slave_x,
                y=slave_y,
                z=slave_z,
                mode='markers',
                text=slave_name, textposition="bottom center",
                name=nameS
            )
        )
        final_layout.update_layout(scene = dict(
                    xaxis_title='Roll',
                    yaxis_title='Pitch',
                    zaxis_title='Re-projection error'))
        final_layout.show()
        pass


    def draw_corners(self, frame, corners):
        for c in corners:
            x = tuple(c.astype('int'))
            frame = cv2.circle(frame, x, radius=0, color=(0, 0, 255), thickness=30)
        return frame

    def image_load(self, path, cam, board, image):
        frame = cv2.imread(path)
        cam_id = self.cameras.index(cam)
        board_id = self.boards.index(board)
        img_id = self.images.index(image)
        corners = self.detectedPoints[cam_id][img_id][board_id].corners

        cam_matrix = np.array(self.intrinsic["cameras"][cam]['K'])
        cam_dist = np.array(self.intrinsic["cameras"][cam]['dist'])
        rtvecs = from_matrix(np.array(self.workspace.pose_table.poses[cam_id][img_id][board_id]))
        marker_length = self.workspace.boards[board_id].marker_length
        rvecs, tvecs = split(rtvecs)
        ## add corner detection and frameaxes
        print(path, 'rvec: ', rvecs)
        cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvecs, tvecs, 0.1, thickness=20)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = self.draw_corners(frame, corners)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(700, 700, Qt.KeepAspectRatio)
        imageLabel = QLabel()
        x = QPixmap.fromImage(p)
        # print(h, w, ch)
        imageLabel.setPixmap(x)
        return imageLabel