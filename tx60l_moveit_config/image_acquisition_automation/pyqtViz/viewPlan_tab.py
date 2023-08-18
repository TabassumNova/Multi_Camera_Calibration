import os.path

from cameraWindow import *
from calibrtaion_tab2 import *
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
from src.multical_scripts.board_angle import *
from src.multical_scripts.handEye_viz import *
# from src.aravis_show_image import *
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

from calibration_tab import *
from src.aravis_show_image import find_cameras, show_image
from functools import partial
from src.helpers import *
from src.multical_scripts.camcalib_detect import *
from src.multical.board import load_config as board_load_config
from src.multical_scripts import *
import cv2

import rospy
# from src.box_attacher_3 import *
# from src.data_robot_mover2 import *

class View_Plan(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QGridLayout(self)
        self.box_attacher = None
        # self.start_boxAttacher()
        self.viewer = {}
        self.cameras = None
        self.camera_checkBox = {}
        self.board_checkBox = {}
        self.saved_path = None
        self.handEyeGripper = None
        self.cameraIntrinsic_file = None
        self.cameraIntrinsic = None
        self.board_config = None
        self.boards = None
        self.camera_images = None
        self.pose = 1
        self.currentCamera = None
        self.currentBoard =None
        self.detection = None
        self.tvecs = None
        self.euler_angle = None
        self.new_tvecs = None
        self.new_euler_angle = None
        # add buttons
        self.btn4_1 = QPushButton(self)
        self.btn4_1.setObjectName('Find Cameras')
        self.btn4_1.setText('Find Cameras')
        self.btn4_1.setGeometry(QRect(10, 50, 111, 28))
        self.btn4_1.clicked.connect(self.findCameras)

        self.btn4_2 = QPushButton(self)
        self.btn4_2.setObjectName('Directory')
        self.btn4_2.setText('Directory')
        self.btn4_2.setGeometry(QRect(10, 20 , 111, 28))
        self.btn4_2.clicked.connect(self.open_dir_dialog)

        self.label1 = QLabel(self)
        self.label1.setObjectName('Cameras')
        self.label1.setText('Cameras')
        self.label1.setStyleSheet("border: 1px solid black;")
        self.label1.setGeometry(QRect(125, 50, 1200, 28))

        self.label10 = QLabel(self)
        self.label10.setObjectName('Directory')
        self.label10.setText('Directory')
        self.label10.setStyleSheet("border: 1px solid black;")
        self.label10.setGeometry(QRect(125, 20, 1200, 28))


        self.btn4_3 = QPushButton(self)
        self.btn4_3.setObjectName('Show Next Images')
        self.btn4_3.setText('Show Next Images')
        self.btn4_3.setGeometry(QRect(10, 80, 150, 28))
        self.btn4_3.clicked.connect(self.showImages)

        self.btn4_4 = QPushButton(self)
        self.btn4_4.setObjectName('Save Pose')
        self.btn4_4.setText('Save Pose')
        self.btn4_4.setGeometry(QRect(160, 80, 150, 28))
        self.btn4_4.clicked.connect(self.savePose)

        self.btn4_5 = QPushButton(self)
        self.btn4_5.setObjectName('Detect Boards')
        self.btn4_5.setText('Detect Boards')
        self.btn4_5.setGeometry(QRect(0, 690, 100, 28))
        self.btn4_5.clicked.connect(self.detectBoards)

        self.label2 = QLabel(self)
        self.label2.setObjectName('Translation')
        self.label2.setText('Translation')
        self.label2.setStyleSheet("border: 1px solid black;")
        self.label2.setGeometry(QRect(0, 730, 100, 28))

        self.btn3 = QPushButton(self)
        self.btn3.setObjectName('-')
        self.btn3.setText('-')
        self.btn3.setGeometry(QRect(110, 730, 30, 28))
        self.btn3.clicked.connect(partial(self.create_Matrix, 'Translation_X', '-'))

        self.label3 = QLabel(self)
        self.label3.setObjectName('X')
        self.label3.setText('X')
        self.label3.setStyleSheet("border: 1px solid black;")
        self.label3.setGeometry(QRect(140, 730, 80, 28))

        self.btn4 = QPushButton(self)
        self.btn4.setObjectName('+')
        self.btn4.setText('+')
        self.btn4.setGeometry(QRect(220, 730, 30, 28))
        self.btn4.clicked.connect(partial(self.create_Matrix, 'Translation_X', '+'))

        self.btn5 = QPushButton(self)
        self.btn5.setObjectName('-')
        self.btn5.setText('-')
        self.btn5.setGeometry(QRect(260, 730, 30, 28))
        self.btn5.clicked.connect(partial(self.create_Matrix, 'Translation_Y', '-'))

        self.label4 = QLabel(self)
        self.label4.setObjectName('Y')
        self.label4.setText('Y')
        self.label4.setStyleSheet("border: 1px solid black;")
        self.label4.setGeometry(QRect(290, 730, 80, 28))

        self.btn6 = QPushButton(self)
        self.btn6.setObjectName('+')
        self.btn6.setText('+')
        self.btn6.setGeometry(QRect(370, 730, 30, 28))
        self.btn6.clicked.connect(partial(self.create_Matrix, 'Translation_Y', '+'))

        self.btn7 = QPushButton(self)
        self.btn7.setObjectName('-')
        self.btn7.setText('-')
        self.btn7.setGeometry(QRect(410, 730, 30, 28))
        self.btn7.clicked.connect(partial(self.create_Matrix, 'Translation_Z', '-'))

        self.label5 = QLabel(self)
        self.label5.setObjectName('Z')
        self.label5.setText('Z')
        self.label5.setStyleSheet("border: 1px solid black;")
        self.label5.setGeometry(QRect(440, 730, 80, 28))

        self.btn8 = QPushButton(self)
        self.btn8.setObjectName('+')
        self.btn8.setText('+')
        self.btn8.setGeometry(QRect(520, 730, 30, 28))
        self.btn8.clicked.connect(partial(self.create_Matrix, 'Translation_Z', '+'))

        self.label6 = QLabel(self)
        self.label6.setObjectName('Rotation (Euler angle)')
        self.label6.setText('Rotation (Euler angle)')
        self.label6.setStyleSheet("border: 1px solid black;")
        self.label6.setGeometry(QRect(0, 770, 150, 28))

        self.btn9 = QPushButton(self)
        self.btn9.setObjectName('-')
        self.btn9.setText('-')
        self.btn9.setGeometry(QRect(150, 770, 30, 28))
        self.btn9.clicked.connect(partial(self.create_Matrix, 'Rotation_X', '-'))

        self.label7 = QLabel(self)
        self.label7.setObjectName('X')
        self.label7.setText('X')
        self.label7.setStyleSheet("border: 1px solid black;")
        self.label7.setGeometry(QRect(180, 770, 80, 28))

        self.btn10 = QPushButton(self)
        self.btn10.setObjectName('+')
        self.btn10.setText('+')
        self.btn10.setGeometry(QRect(260, 770, 30, 28))
        self.btn10.clicked.connect(partial(self.create_Matrix, 'Rotation_X', '+'))

        self.btn11 = QPushButton(self)
        self.btn11.setObjectName('-')
        self.btn11.setText('-')
        self.btn11.setGeometry(QRect(290, 770, 30, 28))
        self.btn11.clicked.connect(partial(self.create_Matrix, 'Rotation_Y', '-'))

        self.label8 = QLabel(self)
        self.label8.setObjectName('Y')
        self.label8.setText('Y')
        self.label8.setStyleSheet("border: 1px solid black;")
        self.label8.setGeometry(QRect(320, 770, 80, 28))

        self.btn12 = QPushButton(self)
        self.btn12.setObjectName('+')
        self.btn12.setText('+')
        self.btn12.setGeometry(QRect(400, 770, 30, 28))
        self.btn12.clicked.connect(partial(self.create_Matrix, 'Rotation_Y', '+'))

        self.btn13 = QPushButton(self)
        self.btn13.setObjectName('-')
        self.btn13.setText('-')
        self.btn13.setGeometry(QRect(430, 770, 30, 28))
        self.btn13.clicked.connect(partial(self.create_Matrix, 'Rotation_Z', '-'))

        self.label9 = QLabel(self)
        self.label9.setObjectName('Z')
        self.label9.setText('Z')
        self.label9.setStyleSheet("border: 1px solid black;")
        self.label9.setGeometry(QRect(460, 770, 80, 28))

        self.btn14 = QPushButton(self)
        self.btn14.setObjectName('+')
        self.btn14.setText('+')
        self.btn14.setGeometry(QRect(540, 770, 30, 28))
        self.btn14.clicked.connect(partial(self.create_Matrix, 'Rotation_Z', '+'))

        self.btn15 = QPushButton(self)
        self.btn15.setObjectName('Start motion')
        self.btn15.setText('Start motion')
        self.btn15.setGeometry(QRect(0, 850, 100, 28))
        self.btn15.clicked.connect(self.move_board)

        self.label11 = QLabel(self)
        self.label11.setObjectName('Info')
        self.label11.setText('Info')
        self.label11.setStyleSheet("border: 1px solid black;")
        self.label11.setGeometry(QRect(110, 850, 1200, 28))

        self.gridLayoutWidget4_1 = QWidget(self)
        self.gridLayoutWidget4_1.setGeometry(QRect(0, 200, 1880, 300))
        self.gridLayoutWidget4_1.setObjectName("gridLayoutWidget")
        self.gridLayout4_1 = QGridLayout(self.gridLayoutWidget4_1)
        self.gridLayout4_1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout4_1.setObjectName("gridLayout")

        self.gridLayoutWidget4_2 = QWidget(self)
        self.gridLayoutWidget4_2.setGeometry(QRect(110, 690, 300, 28))
        self.gridLayoutWidget4_2.setObjectName("gridLayoutWidget")
        self.gridLayout4_2 = QGridLayout(self.gridLayoutWidget4_2)
        self.gridLayout4_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout4_2.setObjectName("gridLayout")

        self.btn4_5 = QPushButton(self)
        self.btn4_5.setObjectName('Detect Boards')
        self.btn4_5.setText('Detect Boards')
        self.btn4_5.setGeometry(QRect(0, 690, 100, 28))
        self.btn4_5.clicked.connect(partial(self.detectBoards, self.gridLayout4_2))

        self.setLayout(self.layout)

    def start_boxAttacher(self):
        rospy.init_node('box_attacher_3_node', anonymous=True)
        self.box_attacher = Box_Attacher_3()
        size = (0.25, 0.25, 0.25)
        self.box_attacher.replace_box(size)
        self.box_attacher.add_obstacle("camera_wall_1")
        self.box_attacher.add_obstacle("camera_wall_2")
        self.box_attacher.add_obstacle("glass_wall")
        self.box_attacher.add_obstacle("base")

        pass

    def open_dir_dialog(self):
        '''
        Select a folder that contains - cameraIntrinsic.json, boards.yaml, handEyeGripper.json file
        '''
        dialog = QFileDialog()
        self.saved_path = dialog.getExistingDirectory(None, "Select Folder")
        self.label10.setText(self.saved_path)
        for path, subdirs, files in os.walk(self.saved_path):
            for name in files:
                if name == 'handEyeGripper.json':
                    file = open(os.path.join(self.saved_path, name))
                    self.handEyeGripper = json.load(file)
                if name == 'cameraIntrinsic.json':
                    file = open(os.path.join(self.saved_path, name))
                    self.cameraIntrinsic = json.load(file)
                if name == 'boards.yaml':
                    file = open(os.path.join(self.saved_path, name))
                    self.boards_config = board_load_config(file)
                    self.boards = [b for b in self.boards_config.keys()]

    def findCameras(self):
        num_cams, camera_serial = find_cameras()
        self.cameras = camera_serial
        self.label1.setText(str(camera_serial))
        for cam in self.cameras:
            path = self.saved_path + '/' + cam + '/'
            make_directory(path)

    def showImages(self):
        self.camera_images = show_image(size_percentage=1)
        self.set_viewer(gridLayout=self.gridLayout4_1, cameraImgs=self.camera_images)
        return 0

    def savePose(self):
        for cam in self.cameras:
            path = self.saved_path + '/' + cam + '/p' + str(self.pose) + '.png'
            cv2.imwrite(path, self.camera_images[cam])
        self.pose += 1
        print('save pose')
        pass

    def set_viewer(self, gridLayout, cameraImgs):
        # # Create image viewer.
        self.create_ImageViewer()
        self.open_ImageViewer(gridLayout, cameraImgs)
        self.add_cameraCheck(gridLayout)

    def create_ImageViewer(self):
        for c in self.cameras:
            self.viewer[c] = QtImageViewer()
            self.viewer[c].leftMouseButtonReleased.connect(self.handleLeftClick)

    def handleLeftClick(self, x, y):
        row = int(y)
        column = int(x)
        print("Clicked on image pixel (row=" + str(row) + ", column=" + str(column) + ")")

    def open_ImageViewer(self, gridLayout, cam_imgs):
        for idx,v in enumerate(self.viewer):
            self.viewer[v].setImage(cam_imgs[v])
            gridLayout.addWidget(self.viewer[v], 0, idx)

    def add_cameraCheck(self, gridLayout):
        for idx, cam in enumerate(self.cameras):
            self.camera_checkBox[cam] = QCheckBox(text=cam)
            self.camera_checkBox[cam].stateChanged.connect(partial(self.selectedCamera, cam))
            self.camera_checkBox[cam].setGeometry(QRect(0, 650, 30, 28))
            gridLayout.addWidget(self.camera_checkBox[cam], 1, idx)

    def detectBoards(self, gridLayout):
        image = self.camera_images[self.currentCamera]
        _, self.detection = detect_img(self.boards_config, image)
        board_list = []
        for board_id, d in enumerate(self.detection):
            if d.corners.size != 0:
                board_list.append(self.boards[board_id])

        for idx, board in enumerate(board_list):
            self.board_checkBox[board] = QCheckBox(text=board)
            self.board_checkBox[board].stateChanged.connect(partial(self.selectedBoard, board))
            self.board_checkBox[board].setGeometry(QRect(110, 690, 30, 28))
            gridLayout.addWidget(self.board_checkBox[board], 0, idx)
        pass

    def selectedCamera(self, cam):
        state = self.camera_checkBox[cam].isChecked()
        if state:
            self.currentCamera = cam
        else:
            self.currentCamera = None
        print(self.currentCamera)
        pass

    def selectedBoard(self, board):
        state = self.board_checkBox[board].isChecked()
        if state:
            self.currentBoard = board
        else:
            self.currentBoard = None
        self.calculate_current_pose()

    def calculate_current_pose(self):
        board_num = self.boards.index(self.currentBoard)
        cam_matrix, cam_dist = np.array(self.cameraIntrinsic['cameras'][self.currentCamera]['K'], dtype=np.float32),\
                                 np.array(self.cameraIntrinsic['cameras'][self.currentCamera]['dist'], dtype=np.float32)
        ids = self.detection[board_num].ids
        corners = np.array(self.detection[board_num].corners, dtype=np.float32).reshape(-1, 2)
        undistorted = cv2.undistortPoints(corners, cam_matrix, cam_dist, P=cam_matrix)
        detected_board = self.boards[board_num]
        adjusted_points = self.boards_config[detected_board].adjusted_points
        objpoints = np.array([adjusted_points[a] for a in ids], dtype=np.float32).reshape((-1, 3))
        
        ret, rvecs, tvecs, euler_deg, view_angle = board_pose(objpoints,
                                            undistorted, ids, cam_matrix, cam_dist, method="solvePnP")
        self.tvecs = tvecs
        self.euler_angle = euler_deg
        self.new_tvecs = self.tvecs
        self.new_euler_angle = self.euler_angle
        self.label3.setText('X= '+ str(tvecs[0]))
        self.label4.setText('Y= '+str(tvecs[1]))
        self.label5.setText('Z= ' + str(tvecs[2]))
        self.label7.setText('X= ' + str(euler_deg[0]))
        self.label8.setText('Y= ' + str(euler_deg[1]))
        self.label9.setText('Z= ' + str(euler_deg[2]))
        pass

    def create_Matrix(self, action, operator):
        if action == 'Translation_X':
            if operator == '+':
                self.new_tvecs[0] += 0.01
            if operator == '-':
                self.new_tvecs[0] -= 0.01
            self.label3.setText('X= ' + str(self.new_tvecs[0]))
        if action == 'Translation_Y':
            if operator == '+':
                self.new_tvecs[1] += 0.01
            if operator == '-':
                self.new_tvecs[1] -= 0.01
            self.label4.setText('Y= ' + str(self.new_tvecs[1]))
        if action == 'Translation_Z':
            if operator == '+':
                self.new_tvecs[2] += 0.01
            if operator == '-':
                self.new_tvecs[2] -= 0.01
            self.label5.setText('Z= ' + str(self.new_tvecs[2]))
        if action == 'Rotation_X':
            if operator == '+':
                self.new_euler_angle[0] += 1
            if operator == '-':
                self.new_euler_angle[0] -= 1
            self.label7.setText('X= ' + str(self.new_euler_angle[0]))
        if action == 'Rotation_Y':
            if operator == '+':
                self.new_euler_angle[1] += 1
            if operator == '-':
                self.new_euler_angle[1] -= 1
            self.label7.setText('Y= ' + str(self.new_euler_angle[1]))
        if action == 'Rotation_Z':
            if operator == '+':
                self.new_euler_angle[2] += 1
            if operator == '-':
                self.new_euler_angle[2] -= 1
            self.label7.setText('Z= ' + str(self.new_euler_angle[2]))

    def move_board(self):
        rvec = euler_to_rvec(self.new_euler_angle)
        board_wrt_camera = rtvec.to_matrix(rtvec.join(rvec, self.new_tvecs))
        if self.currentCamera in self.handEyeGripper:
            if self.currentBoard in self.handEyeGripper[self.currentCamera]:
                gripper_wrt_board = self.handEyeGripper[self.currentCamera][self.currentBoard]["gripper_wrt_world"]
                camera_wrt_base = np.linalg.inv(self.handEyeGripper[self.currentCamera][self.currentBoard]["base_wrt_cam"])
                gripper_wrt_base = gripper_wrt_board @ board_wrt_camera @ camera_wrt_base
                move_robot(self.box_attacher, gripper_wrt_base)
                self.label11.setText('Transformation Found; Press "Show Next image"')
            else:
                self.label11.setText('Transformation not found')
        else:
            self.label11.setText('Transformation not found')
        pass

