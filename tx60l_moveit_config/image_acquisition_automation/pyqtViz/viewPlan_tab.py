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
from src.multical_scripts.board_angle import *
from src.multical_scripts.handEye_viz import *
from src.aravis_show_image import *
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
# from src.aravis_show_image import find_cameras, show_image
from functools import partial
from src.helpers import *
import cv2

class View_Plan(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QGridLayout(self)
        self.viewer = {}
        self.cameras = None
        self.checkBox = {}
        self.saved_path = None
        self.handEyeGripper = None
        self.camera_images = None
        self.pose = 1
        self.currentCamera = None
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

        self.label2 = QLabel(self)
        self.label2.setObjectName('Translation')
        self.label2.setText('Translation')
        self.label2.setStyleSheet("border: 1px solid black;")
        self.label2.setGeometry(QRect(0, 690, 100, 28))

        self.btn3 = QPushButton(self)
        self.btn3.setObjectName('-')
        self.btn3.setText('-')
        self.btn3.setGeometry(QRect(110, 690, 30, 28))
        self.btn3.clicked.connect(self.showImages)

        self.label3 = QLabel(self)
        self.label3.setObjectName('X')
        self.label3.setText('X')
        self.label3.setStyleSheet("border: 1px solid black;")
        self.label3.setGeometry(QRect(140, 690, 80, 28))

        self.btn4 = QPushButton(self)
        self.btn4.setObjectName('+')
        self.btn4.setText('+')
        self.btn4.setGeometry(QRect(220, 690, 30, 28))
        self.btn4.clicked.connect(self.showImages)

        self.btn5 = QPushButton(self)
        self.btn5.setObjectName('-')
        self.btn5.setText('-')
        self.btn5.setGeometry(QRect(260, 690, 30, 28))
        self.btn5.clicked.connect(self.showImages)

        self.label4 = QLabel(self)
        self.label4.setObjectName('Y')
        self.label4.setText('Y')
        self.label4.setStyleSheet("border: 1px solid black;")
        self.label4.setGeometry(QRect(290, 690, 80, 28))

        self.btn6 = QPushButton(self)
        self.btn6.setObjectName('+')
        self.btn6.setText('+')
        self.btn6.setGeometry(QRect(370, 690, 30, 28))
        self.btn6.clicked.connect(self.showImages)

        self.btn7 = QPushButton(self)
        self.btn7.setObjectName('-')
        self.btn7.setText('-')
        self.btn7.setGeometry(QRect(410, 690, 30, 28))
        self.btn7.clicked.connect(self.showImages)

        self.label5 = QLabel(self)
        self.label5.setObjectName('Z')
        self.label5.setText('Z')
        self.label5.setStyleSheet("border: 1px solid black;")
        self.label5.setGeometry(QRect(440, 690, 80, 28))

        self.btn8 = QPushButton(self)
        self.btn8.setObjectName('+')
        self.btn8.setText('+')
        self.btn8.setGeometry(QRect(520, 690, 30, 28))
        self.btn8.clicked.connect(self.showImages)

        self.label6 = QLabel(self)
        self.label6.setObjectName('Rotation')
        self.label6.setText('Rotation')
        self.label6.setStyleSheet("border: 1px solid black;")
        self.label6.setGeometry(QRect(0, 730, 100, 28))

        self.btn9 = QPushButton(self)
        self.btn9.setObjectName('-')
        self.btn9.setText('-')
        self.btn9.setGeometry(QRect(110, 730, 30, 28))
        self.btn9.clicked.connect(self.showImages)

        self.label7 = QLabel(self)
        self.label7.setObjectName('X')
        self.label7.setText('X')
        self.label7.setStyleSheet("border: 1px solid black;")
        self.label7.setGeometry(QRect(140, 730, 80, 28))

        self.btn10 = QPushButton(self)
        self.btn10.setObjectName('+')
        self.btn10.setText('+')
        self.btn10.setGeometry(QRect(220, 730, 30, 28))
        self.btn10.clicked.connect(self.showImages)

        self.btn11 = QPushButton(self)
        self.btn11.setObjectName('-')
        self.btn11.setText('-')
        self.btn11.setGeometry(QRect(260, 730, 30, 28))
        self.btn11.clicked.connect(self.showImages)

        self.label8 = QLabel(self)
        self.label8.setObjectName('Y')
        self.label8.setText('Y')
        self.label8.setStyleSheet("border: 1px solid black;")
        self.label8.setGeometry(QRect(290, 730, 80, 28))

        self.btn12 = QPushButton(self)
        self.btn12.setObjectName('+')
        self.btn12.setText('+')
        self.btn12.setGeometry(QRect(370, 730, 30, 28))
        self.btn12.clicked.connect(self.showImages)

        self.btn13 = QPushButton(self)
        self.btn13.setObjectName('-')
        self.btn13.setText('-')
        self.btn13.setGeometry(QRect(410, 730, 30, 28))
        self.btn13.clicked.connect(self.showImages)

        self.label9 = QLabel(self)
        self.label9.setObjectName('Z')
        self.label9.setText('Z')
        self.label9.setStyleSheet("border: 1px solid black;")
        self.label9.setGeometry(QRect(440, 730, 80, 28))

        self.btn14 = QPushButton(self)
        self.btn14.setObjectName('+')
        self.btn14.setText('+')
        self.btn14.setGeometry(QRect(520, 730, 30, 28))
        self.btn14.clicked.connect(self.showImages)

        self.gridLayoutWidget4_1 = QWidget(self)
        self.gridLayoutWidget4_1.setGeometry(QRect(0, 200, 1880, 300))
        self.gridLayoutWidget4_1.setObjectName("gridLayoutWidget")
        self.gridLayout4_1 = QGridLayout(self.gridLayoutWidget4_1)
        self.gridLayout4_1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout4_1.setObjectName("gridLayout")

        self.setLayout(self.layout)

    def open_dir_dialog(self):
        '''
        Select a folder that contains handEyeGripper.json file
        '''
        dialog = QFileDialog()
        self.saved_path = dialog.getExistingDirectory(None, "Select Folder")
        self.label10.setText(self.saved_path)
        for path, subdirs, files in os.walk(self.saved_path):
            for name in files:
                if name == 'handEyeGripper.json':
                    self.handEyeGripper = os.path.join(self.saved_path, name)
        print(self.handEyeGripper)

    def findCameras(self):
        num_cams, camera_serial = find_cameras()
        self.cameras = camera_serial
        self.label1.setText(str(camera_serial))
        for cam in self.cameras:
            path = self.saved_path + '/' + cam + '/'
            make_directory(path)

    def showImages(self):
        self.camera_images = show_image()
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
            self.checkBox[cam] = QCheckBox(text=cam)
            # chkBox1 = QCheckBox(text=cam)
            # btn = QPushButton(self)
            # btn.setObjectName(cam)
            # btn.setText(cam)
            # self.checkBox[cam].clicked.connect(partial(self.selectedCamera, cam))
            self.checkBox[cam].stateChanged.connect(partial(self.selectedCamera, cam))
            self.checkBox[cam].setGeometry(QRect(0, 650, 30, 28))
            gridLayout.addWidget(self.checkBox[cam], 1, idx)

    def selectedCamera(self, cam):
        state = self.checkBox[cam].isChecked()
        if state:
            self.currentCamera = cam
        else:
            self.currentCamera = None
        print(self.currentCamera)

        # print(state)
        # if state == Qt.CheckState.Checked:
        #     print('Checked')
        # elif state == Qt.CheckState.Unchecked:
        #     print('Unchecked')
        # self.currentCamera = cam
        # print(self.currentCamera)
        # print(cam)
        pass
    
    def move_camera(self, cam):
        pass

    # def set_viewer(self, gridLayout, group_name="group01", live_view=False, cameraImgs={}):
    #     # # # Create image viewer.
    #     # self.viewer[0] = QtImageViewer()
    #     # self.viewer[0].leftMouseButtonReleased.connect(self.handleLeftClick)
    #     #
    #     # self.viewer[1] = QtImageViewer()
    #     # self.viewer[1].leftMouseButtonReleased.connect(self.handleLeftClick)
    #     #
    #     # self.viewer[2] = QtImageViewer()
    #     # self.viewer[2].leftMouseButtonReleased.connect(self.handleLeftClick)
    #     #
    #     # self.viewer[3] = QtImageViewer()
    #     # self.viewer[3].leftMouseButtonReleased.connect(self.handleLeftClick)
    #     #
    #     # self.viewer[4] = QtImageViewer()
    #     # self.viewer[4].leftMouseButtonReleased.connect(self.handleLeftClick)
    #     #
    #     # self.viewer[5] = QtImageViewer()
    #     # self.viewer[5].leftMouseButtonReleased.connect(self.handleLeftClick)
    #     #
    #     # if self.folder_path:
    #     #     cam1 = self.folder_path + '/cam1'
    #     #     self.last_pose = (len([name for name in os.listdir(cam1) if os.path.isfile(os.path.join(cam1, name))]))
    #     #     # start = time.time()
    #     #     # Open images as numpy array
    #     #     v1 = self.folder_path + '/cam1' + '/p' + str(self.pose) + '.png'
    #     #     v2 = self.folder_path + '/cam2' + '/p' + str(self.pose) + '.png'
    #     #     v3 = self.folder_path + '/cam3' + '/p' + str(self.pose) + '.png'
    #     #     v4 = self.folder_path + '/cam4' + '/p' + str(self.pose) + '.png'
    #     #     v5 = self.folder_path + '/cam5' + '/p' + str(self.pose) + '.png'
    #     #     v6 = self.folder_path + '/cam6' + '/p' + str(self.pose) + '.png'
    #     #
    #     #     self.viewer[0].open(v1)
    #     #     self.viewer[1].open(v2)
    #     #     self.viewer[2].open(v3)
    #     #     self.viewer[3].open(v4)
    #     #     self.viewer[4].open(v5)
    #     #     self.viewer[5].open(v6)
    #     #
    #     #     self.add_3d_scatter()
    #     #     self.add_table_widget()
    #
    #
    #     if live_view:
    #         cam1 = 11120229
    #         cam2 = 11120233
    #         cam3 = 11120237
    #         cam4 = 11120241
    #         cam5 = 42120642
    #         cam6 = 42120643
    #
    #         if cam1 in cameraImgs:
    #             v1 = cameraImgs[cam1]
    #         else:
    #             v1 = np.ones((5,5))
    #         if cam2 in cameraImgs:
    #             v2 = cameraImgs[cam2]
    #         else:
    #             v2 = np.ones((5,5))
    #         if cam3 in cameraImgs:
    #             v3 = cameraImgs[cam3]
    #         else:
    #             v3 = np.ones((5,5))
    #         if cam4 in cameraImgs:
    #             v4 = cameraImgs[cam4]
    #         else:
    #             v4 = np.ones((5,5))
    #         if cam5 in cameraImgs:
    #             v5 = cameraImgs[cam5]
    #         else:
    #             v5 = np.ones((5,5))
    #         if cam6 in cameraImgs:
    #             v6 = cameraImgs[cam6]
    #         else:
    #             v6 = np.ones((5,5))
    #
    #         self.viewer[0].setImage(v1)
    #         self.viewer[1].setImage(v2)
    #         self.viewer[2].setImage(v3)
    #         self.viewer[3].setImage(v4)
    #         self.viewer[4].setImage(v5)
    #         self.viewer[5].setImage(v6)
    #
    #     start = time.time()
    #     label1 = QLabel()
    #     label1.setText("Camera 01")
    #     label1.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label1, 1, 0)
    #     label2 = QLabel()
    #     label2.setText("Camera 02")
    #     label2.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label2, 1, 1)
    #     label3 = QLabel()
    #     label3.setText("Camera 03")
    #     label3.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label3, 1, 2)
    #     label4 = QLabel()
    #     label4.setText("Camera 04")
    #     label4.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label4, 1, 3)
    #     label5 = QLabel()
    #     label5.setText("Camera 05")
    #     label5.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label5, 1, 4)
    #     label6 = QLabel()
    #     label6.setText("Camera 06")
    #     label6.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label6, 1, 5)
    #     label7 = QLabel()
    #     label7.setText("Pose: "+ str(self.pose))
    #     label7.setAlignment(Qt.AlignCenter)
    #     gridLayout.addWidget(label7, 2, 0)
    #
    #     # self.layout.update()
    #     gridLayout.addWidget(self.viewer[0], 0, 0)
    #     gridLayout.addWidget(self.viewer[1], 0, 1)
    #     gridLayout.addWidget(self.viewer[2], 0, 2)
    #     gridLayout.addWidget(self.viewer[3], 0, 3)
    #     gridLayout.addWidget(self.viewer[4], 0, 4)
    #     gridLayout.addWidget(self.viewer[5], 0, 5)
    #
    #     end = time.time()
    #     print("Passed time: ", end - start)