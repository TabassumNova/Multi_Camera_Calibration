import os.path
from .cameraWindow import *
# from calibrtaion_tab2 import *
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
# from QtImageViewer import QtImageViewer
from src.multical_scripts.board_angle import *
# from src.multical_scripts.handEye_viz import *
from src.multical_scripts.handEye_final import *
from src.multical_scripts.extrinsic_viz import *
from src.multical_scripts.camcalib_calibrate import *
from src.multical_scripts.singleCalib_viz import *
import shutil
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

from .calibration_tab import *
from src.QtImageViewer import *

class Operation(QScrollArea):
    def __init__(self):
        super(Operation, self).__init__()
        self.workspace = None
        self.camera_checkBox = {}
        self.masterCamera = None
        self.test_dir = None
        self.final_calibration = None

        self.widget1 = QWidget()
        layout1 = QGridLayout(self.widget1)
        layout1.setAlignment(Qt.AlignTop)
        l1 = QLabel(self.widget1)
        l1.setPixmap(QPixmap("cover1.png"))
        l1.setGeometry(QRect(50, 0, 1880, 300))
        # layout1.addWidget(l1)

        label1 = QLabel(self.widget1)
        label1.setText('Select Directory')
        label1.setFont(QFont("Times", 10, QFont.Bold))
        # label1.setAlignment(Qt.AlignCenter)
        label1.setGeometry(QRect(0, 320, 150, 28))

        self.btn1 = QPushButton(self.widget1)
        self.btn1.setObjectName('Directory')
        self.btn1.setText('Directory')
        self.btn1.setGeometry(QRect(0, 360, 111, 28))
        self.btn1.clicked.connect(self.open_dir_dialog)
        # layout1.addWidget(self.btn4_2)

        self.label1 = QLabel(self)
        self.label1.setObjectName('Select Directory')
        self.label1.setText('Select Directory')
        self.label1.setStyleSheet("border: 1px solid black;")
        self.label1.setGeometry(QRect(125, 360, 500, 28))

        label2 = QLabel(self.widget1)
        label2.setText('Initialization')
        label2.setFont(QFont("Times", 10, QFont.Bold))
        # label2.setAlignment(Qt.AlignCenter)
        label2.setGeometry(QRect(0, 400, 150, 28))

        self.btn2 = QPushButton(self.widget1)
        self.btn2.setObjectName('Start Calculation')
        self.btn2.setText('Start Calculation')
        self.btn2.setGeometry(QRect(0, 430, 111, 28))
        self.btn2.clicked.connect(self.initialization_calculation)
        # layout1.addWidget(self.btn4_2)

        self.label2 = QLabel(self.widget1)
        self.label2.setObjectName('Info')
        self.label2.setText('Info')
        self.label2.setStyleSheet("border: 1px solid black;")
        self.label2.setGeometry(QRect(125, 430, 300, 28))

        self.btn3 = QPushButton(self.widget1)
        self.btn3.setObjectName('Show Camera Poses')
        self.btn3.setText('Show Camera Poses')
        self.btn3.setGeometry(QRect(450, 430, 180, 28))
        self.btn3.clicked.connect(self.showCameras)

        label3 = QLabel(self.widget1)
        label3.setText('Select one Master Camera')
        label3.setGeometry(QRect(0, 460, 200, 28))

        self.gridLayoutWidget1 = QWidget(self)
        self.gridLayoutWidget1.setGeometry(QRect(200, 460, 700, 28))
        self.gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.gridLayout1 = QGridLayout(self.gridLayoutWidget1)
        self.gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout1.setObjectName("gridLayout")

        label4 = QLabel(self.widget1)
        label4.setText('Bundle Adjustment')
        label4.setFont(QFont("Times", 10, QFont.Bold))
        # label2.setAlignment(Qt.AlignCenter)
        label4.setGeometry(QRect(0, 500, 180, 28))

        self.btn3 = QPushButton(self.widget1)
        self.btn3.setObjectName('Start Calculation')
        self.btn3.setText('Start Calculation')
        self.btn3.setGeometry(QRect(0, 540, 111, 28))
        self.btn3.clicked.connect(self.bundleAdjustment_calculation)
        # layout1.addWidget(self.btn4_2)

        self.label3 = QLabel(self.widget1)
        self.label3.setObjectName('Info')
        self.label3.setText('Info')
        self.label3.setStyleSheet("border: 1px solid black;")
        self.label3.setGeometry(QRect(125, 540, 300, 28))

        self.btn4 = QPushButton(self.widget1)
        self.btn4.setObjectName('Show Final Camera Poses')
        self.btn4.setText('Show Final Camera Poses')
        self.btn4.setGeometry(QRect(450, 540, 200, 28))
        self.btn4.clicked.connect(self.finalCam_pose)

        label5 = QLabel(self.widget1)
        label5.setText('Results')
        label5.setFont(QFont("Times", 10, QFont.Bold))
        # label2.setAlignment(Qt.AlignCenter)
        label5.setGeometry(QRect(0, 580, 180, 28))

        # for index in range(100):
        #     layout1.addWidget(QLabel('Label %02d' % index))
        self.setWidget(self.widget1)
        self.setWidgetResizable(True)
        self.widget1.setLayout(layout1)
        # self.setWidgetResizable(True)
        # scrollContent = QWidget(self)
        # scrollLayout = QVBoxLayout(scrollContent)
        # scrollContent.setLayout(scrollLayout)
        # self.setWidget(scrollContent)


    def saveImage(self):
        print("Button clicked, Hello! Save Image")

    def nextPose(self):
        print("Button clicked, Next!")
        self.set_viewer()
        self.show()

    def set_viewer(self, gridLayout):
        if self.folder_path:
            # # Create image viewer.
            self.create_ImageViewer()
            self.open_ImageViewer(gridLayout)
            self.add_cameraLabel(gridLayout)
            # self.add_3d_scatter()
            self.add_table_widget()

    def create_ImageViewer(self, folder_path = True):
        if folder_path:
            for c in self.cameras:
                self.viewer[c] = QtImageViewer()
                self.viewer[c].leftMouseButtonReleased.connect(self.handleLeftClick)
        pass

    def handleLeftClick(self, x, y):
        row = int(y)
        column = int(x)
        print("Clicked on image pixel (row=" + str(row) + ", column=" + str(column) + ")")

    def open_ImageViewer(self, gridLayout):
        for path, subdirs, files in os.walk(self.folder_path):
            if path == self.folder_path:
                for idx, dir in enumerate(subdirs):
                    v = os.path.join(self.folder_path, dir, self.images[self.pose_count])
                    self.viewer[dir].open(v)
                    gridLayout.addWidget(self.viewer[dir], 0, idx)
        self.clearLayout(self.gridLayout2)

        label1 = QLabel()
        label1.setText('Pose')
        label1.setFont(QFont("Times", 10, QFont.Bold))
        label1.setAlignment(Qt.AlignCenter)
        self.gridLayout2.addWidget(label1, 2, 0)


        label = QLabel()
        label.setText(self.images[self.pose_count])
        label.setAlignment(Qt.AlignCenter)
        self.gridLayout2.addWidget(label, 2, 1)
        self.gridLayout2.addWidget(label, 2, 1)

    def add_cameraLabel(self, gridLayout):
        for idx, cam in enumerate(self.cameras):
            btn = QPushButton(self)
            btn.setObjectName(cam)
            btn.setText(cam)
            btn.clicked.connect(lambda checked: self.show_CamImages())
            # btn.setGeometry(QRect(543, 0, 30, 28))
            gridLayout.addWidget(btn, 1, idx)

    def show_CamImages(self):
        if self.new_window is None:
            self.new_window = CameraWindow(self.folder_path, self.workspace)
        self.new_window.resize(500, 500)
        self.new_window.show()

    def add_table_widget(self):
        self.table.setRowCount(len(self.boards))
        self.table.setColumnCount(len(self.cameras))
        self.table.setHorizontalHeaderLabels(self.cameras)
        self.table.setVerticalHeaderLabels(self.boards)

        for cam in self.cameras:
            camera_id = self.cameras.index(cam)
            for board in self.boards:
                board_id = self.boards.index(board)
                num_points = self.workspace.pose_table.num_points[camera_id][self.pose_count][board_id]
                repo_error = "{:.2f}".format(self.workspace.pose_table.reprojection_error[camera_id][self.pose_count][board_id])
                item = QTableWidgetItem()
                if num_points == 0:
                    text = ''
                else:
                    text = 'num_points: '+ str(num_points) + ' | '+ 'Repo_error: '+str(repo_error)
                item.setText(text)
                self.table.setItem(board_id, camera_id, item)
                pass
            header = self.table.horizontalHeader()
            header.setSectionResizeMode(camera_id, QHeaderView.ResizeMode.ResizeToContents)
            # header.setStretchLastSection(True)

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def showCameras(self):
        v = Interactive_Extrinsic(self.folder_path)

    def initialization_calculation(self):
        if os.path.isfile(os.path.join(self.folder_path, 'workspace.pkl')) and os.path.isfile(os.path.join(self.folder_path, 'meanCameras.json')):
            self.label2.setText('Load previous Initialization')
            workspace_path = os.path.join(self.folder_path, 'workspace.pkl')
            self.workspace = pickle.load(open(workspace_path, "rb"))

        else:
            h = handEye(self.folder_path)
            h.initiate_workspace()
            h.calc_camPose_param(limit_images=6, limit_board_image=6, calculate_handeye=True, check_cluster=True)
            h.export_handEye_Camera()
            for path, subdirs, files in os.walk((self.folder_path)):
                if path == self.folder_path:
                    if 'workspace.pkl' in files:
                        workspace_path = os.path.join(self.folder_path, 'workspace.pkl')
                        self.workspace = pickle.load(open( workspace_path, "rb"))
            self.label2.setText('Initialization Done')

        if self.workspace:
            for idx, cam in enumerate(self.workspace.names.camera):
                self.camera_checkBox[cam] = QCheckBox(text=cam)
                self.camera_checkBox[cam].stateChanged.connect(partial(self.selectedCamera, cam))
                self.camera_checkBox[cam].setGeometry(QRect(0, 650, 30, 28))
                self.gridLayout1.addWidget(self.camera_checkBox[cam], 1, idx)


    def createbundle_dir(self):
        os.chdir(self.folder_path)
        mycwd = os.getcwd()
        os.chdir("..")
        mycwd1 = os.getcwd()
        dataset = mycwd.split(mycwd1)
        dataset_test = dataset[1][1:]+'_test'
        self.test_dir = os.path.join(mycwd1, dataset_test)
        if os.path.exists(self.test_dir):
            for path, subdirs, files in os.walk((self.test_dir)):
                if path == self.test_dir:
                    for f in files:
                        if 'initial_calibration_M' in f:
                            mCam0 = f.split('initial_calibration_M')[1]
                            mCam = mCam0.split('.json')[0]
                            s = 'calibration_' + mCam + '.pkl'
                            self.masterCamera = mCam
                            if s in files:
                                self.final_calibration = pickle.load(open(os.path.join(self.test_dir, s), "rb"))
                                self.workspace = None
                            break

        else:
            os.mkdir(self.test_dir)
            print("test path: ", self.test_dir)
            # copy boards.yaml
            org_file = os.path.join(self.folder_path, 'boards.yaml')
            new_file = os.path.join(self.test_dir, 'boards.yaml')
            shutil.copy(org_file, new_file)
            # copy initial_calibration_M
            file = 'initial_calibration_M' + self.masterCamera + '.json'
            org_file1 = os.path.join(self.folder_path, file)
            new_file1 = os.path.join(self.test_dir, file)
            shutil.copy(org_file1, new_file1)
            # select first 20 images of each camera
            image_name = self.workspace.names.image[0:20]
            for cam_idx, cam in enumerate(self.workspace.names.camera):
                org_cam_path = os.path.join(self.folder_path, cam)
                new_cam_path = os.path.join(self.test_dir, cam)
                os.mkdir(new_cam_path)
                for img in image_name:
                    org_img_file = os.path.join(org_cam_path, img)
                    new_img_file = os.path.join(new_cam_path, img)
                    shutil.copy(org_img_file, new_img_file)

            pass

    def finalCam_pose(self):
        v = Interactive_calibration(self.test_dir)
        pass

    def bundleAdjustment_calculation(self):
        if self.masterCamera == None:
            self.label3.setText('Select one Master Camera')
        else:
            self.createbundle_dir()
            if self.final_calibration:
                self.label3.setText('Load previous Calculation for '+str(self.masterCamera))
            else:
                file = 'initial_calibration_M' + self.masterCamera + '.json'
                intrinsic_path = os.path.join(self.test_dir, file)
                final_calibration(base_path=self.test_dir, master_cam=self.masterCamera, intrinsic_path=intrinsic_path)
                s = 'calibration_'+self.masterCamera+'.pkl'
                if os.path.exists(os.path.join(self.test_dir, s)):
                    self.final_calibration = pickle.load(open(os.path.join(self.test_dir, s), "rb"))
                    self.workspace = None
                    self.label3.setText('Bundle Adjustment Done for '+str(self.masterCamera))
                pass



    def selectedCamera(self, cam):
        state = self.camera_checkBox[cam].isChecked()
        if state:
            self.masterCamera = cam
        else:
            self.masterCamera = None
        print('Master-Camera: ', self.masterCamera)
        pass

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        self.workspace_load()


    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                if 'boards.yaml' not in files:
                    self.label1.setText('No boards.yaml file found')
                else:
                    self.label1.setText(self.folder_path)

    def loadNext(self):
        if self.last_pose_count >= self.pose_count >= 0:
            # self.pose_count += 1
            self.clearLayout(self.gridLayout1)
            self.set_viewer(gridLayout=self.gridLayout1)
            self.pose_count += 1
        else:
            self.clearLayout(self.gridLayout1)
        # return 0

    def loadPrevious(self):
        if self.pose_count > 0:
            # self.pose_count -= 1
            self.clearLayout(self.gridLayout1)
            self.set_viewer(gridLayout=self.gridLayout1)
            self.pose_count -= 1
        else:
            self.clearLayout(self.gridLayout1)