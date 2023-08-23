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

class Calibration(QWidget):
    def __init__(self):
        super().__init__()

        self.layout = QVBoxLayout(self)
        self.Current_poseCount = 0
        self.Last_poseCount = 0

        self.cb = QComboBox(self)
        self.handEyeCamera = None
        self.workspace = None
        self.folder_path = None
        self.cameras = None
        self.images = None
        self.boards = None
        self.detectedPoints = None
        self.intrinsic = None
        self.selected_pose = []

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
        self.btnLoad4.setObjectName('Select Current Pose')
        self.btnLoad4.setText('Select Current Pose')
        self.btnLoad4.clicked.connect(self.selectCurrentPose)
        self.btnLoad4.setGeometry(QRect(650, 0, 180, 28))

        self.btnLoad5 = QPushButton(self)
        self.btnLoad5.setObjectName('Show visualization')
        self.btnLoad5.setText('Show visualization')
        self.btnLoad5.clicked.connect(self.showviz)
        self.btnLoad5.setGeometry(QRect(830, 0, 150, 28))

        self.label1 = QLabel(self)
        self.label1.setObjectName('Pose')
        self.label1.setText('Pose')
        self.label1.setStyleSheet("border: 1px solid black;")
        self.label1.setGeometry(QRect(525, 0, 93, 28))

        self.label2 = QLabel(self)
        self.label2.setObjectName('Selected poses')
        self.label2.setText('Selected poses')
        self.label2.setStyleSheet("border: 1px solid black;")
        self.label2.setGeometry(QRect(0, 30, 1880, 28))

        self.btnLoad6 = QPushButton(self)
        self.btnLoad6.setObjectName('Calibrate Selected poses')
        self.btnLoad6.setText('Calibrate Selected poses')
        self.btnLoad6.clicked.connect(self.handEye_Calibration)
        self.btnLoad6.setGeometry(QRect(0, 60, 150, 28))

        self.label3 = QLabel(self)
        self.label3.setObjectName('Result')
        self.label3.setText('Result')
        self.label3.setStyleSheet("border: 1px solid black;")
        self.label3.setGeometry(QRect(153, 60, 200, 28))

        # Grid for images
        self.gridLayoutWidget1 = QWidget(self)
        self.gridLayoutWidget1.setGeometry(QRect(0, 100, 1880, 500))
        self.gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.gridLayout1 = QGridLayout(self.gridLayoutWidget1)
        self.gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout1.setObjectName("gridLayout")

        # Grid for table
        self.gridLayoutWidget3 = QWidget(self)
        self.gridLayoutWidget3.setGeometry(QRect(0, 600, 700, 700))
        self.gridLayoutWidget3.setObjectName("gridLayoutWidget")
        self.gridLayout3 = QGridLayout(self.gridLayoutWidget3)
        self.gridLayout3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout3.setObjectName("gridLayout")
        self.table = QTableWidget()
        self.gridLayout3.addWidget(self.table)

        self.setLayout(self.layout)

    def handEye_Calibration(self):
        pass

    def group_decode(self, group):
        group_num = {}
        total = 0
        for cam_num, cam_group in self.handEyeCamera.items():
            group_num[cam_num] = np.arange(total, total+len(self.handEyeCamera[cam_num]))
            total += len(self.handEyeCamera[cam_num])
            if group in group_num[cam_num]:
                return cam_num, np.where(group_num[cam_num]==group)[0][0]
        pass

    def selectionchange(self, group, poseCount=0):
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
        master_path = os.path.join(self.folder_path, master_cam, image_list[poseCount])
        slave_path = os.path.join(self.folder_path, slave_cam, image_list[poseCount])
        pose = 'Pose '+ image_list[poseCount][1:-4]
        self.label1.setText(pose)
        imageLabel1 = self.image_load(master_path, master_cam, master_board, image_list[poseCount])
        imageLabel2 = self.image_load(slave_path, slave_cam, slave_board, image_list[poseCount])
        # self.clearLayout(layout)
        self.gridLayout1.addWidget(imageLabel1, 0, 0)
        self.gridLayout1.addWidget(imageLabel2, 0, 1)

        self.calibrationTab_Table()
        # print(self.group)
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
        item6.setText(str("{:.2f}".format(float(handEye_Cam["initial_reprojection_error"]))))
        item7.setText(str("{:.2f}".format(float(handEye_Cam["final_reprojection_error"]))))
        self.table.setItem(0, 0, item1)
        self.table.setItem(1, 0, item2)
        self.table.setItem(2, 0, item3)
        self.table.setItem(3, 0, item4)
        self.table.setItem(4, 0, item5)
        self.table.setItem(5, 0, item6)
        self.table.setItem(6, 0, item7)

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        self.workspace_load()

        if self.handEyeCamera != None:
            for cam_num, cam_value in self.handEyeCamera.items():
                for k in cam_value.keys():
                    text = "Cam" + cam_num + " Group-" + str(k)
                    self.cb.addItem(text)

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                workspace_path = os.path.join(self.folder_path, [f for f in files if f == "workspace.pkl"][0])
                self.workspace = pickle.load(open( workspace_path, "rb"))
                self.cameras = self.workspace.names.camera
                self.images = self.workspace.names.image
                self.boards = self.workspace.names.board
                self.last_pose_count = len(self.images)
                for file in files:
                    if file == 'calibration.detections.pkl':
                        pickle_file = ws.Workspace.load(os.path.join(self.folder_path, 'calibration.detections.pkl'))
                        self.detectedPoints = pickle_file.detected_points
                    if file == "calibration.json":
                        intrinsic_path = os.path.join(self.folder_path, 'calibration.json')
                        self.intrinsic = json.load(open(intrinsic_path))
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.folder_path, "handEyeCamera.json")
                        self.handEyeCamera = json.load(open(handEye_path))


    def loadNext(self):
        if self.Last_poseCount-1 > self.Current_poseCount >= 0:
            self.Current_poseCount += 1
            self.clearLayout(self.gridLayout1)
            self.selectionchange(self.group, self.Current_poseCount)
        else:
            self.clearLayout(self.gridLayout1)
            print("Pose end")
        # return 0

    def loadPrevious(self):
        if self.Current_poseCount > 0:
            self.Current_poseCount -= 1
            self.clearLayout(self.gridLayout1)
            self.selectionchange(self.group, self.Current_poseCount)
        else:
            self.clearLayout(self.gridLayout1)

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def showviz(self):
        camera = str(self.cam_num)
        group = str(self.cam_group)
        self.draw_viz(camera, group)

    def selectCurrentPose(self):
        image = self.handEyeCamera[self.cam_num][str(self.cam_group)]['image_list'][self.Current_poseCount]
        if image not in self.selected_pose:
            self.selected_pose.append(image)
        self.label2.setText(str(self.selected_pose))
        print(self.selected_pose, self.cam_num, self.cam_group)

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
        frame = self.draw_corners(frame, corners)

        cv2.drawFrameAxes(frame, cam_matrix, cam_dist, rvecs, tvecs, 0.1, thickness=20)
        h, w, ch = frame.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(700, 700, Qt.KeepAspectRatio)
        imageLabel = QLabel()
        x = QPixmap.fromImage(p)
        # print(h, w, ch)
        imageLabel.setPixmap(x)
        return imageLabel