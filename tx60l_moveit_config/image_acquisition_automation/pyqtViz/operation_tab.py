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
from src.multical_scripts.board_angle import *
from src.multical_scripts.handEye_viz import *
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

class Operation(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QGridLayout(self)
        self.workspace = None
        self.cameras = None
        self.images = None
        self.boards = None
        self.detectedPoints = None
        self.intrinsic = None
        self.handEyeCamera = None
        self.viewer = {}
        self.new_window = None

        self.btnLoad1 = QPushButton(self)
        self.btnLoad1.setObjectName('Live View')
        self.btnLoad1.setText('Live View')
        self.btnLoad1.setGeometry(QRect(0, 0, 93, 28))
        self.btnLoad1.clicked.connect(self.saveImage)

        self.btnLoad2 = QPushButton(self)
        self.btnLoad2.setObjectName('Start')
        self.btnLoad2.setText('Start')
        # self.btnLoad1.setStyleSheet("background-color : cyan;")
        self.btnLoad2.setGeometry(QRect(90, 0, 93, 28))
        self.btnLoad2.clicked.connect(self.nextPose)

        self.btnLoad3 = QPushButton(self)
        self.btnLoad3.setObjectName('Stop')
        self.btnLoad3.setText('Stop')
        # self.btnLoad2.setStyleSheet("background-color : cyan;")
        self.btnLoad3.setGeometry(QRect(180, 0, 93, 28))
        self.btnLoad3.clicked.connect(self.nextPose)

        self.btnLoad4 = QPushButton(self)
        self.btnLoad4.setObjectName('x')
        self.btnLoad4.setText('x')
        # self.btnLoad3.setStyleSheet("background-color : cyan;")
        self.btnLoad4.clicked.connect(self.nextPose)
        self.btnLoad4.setGeometry(QRect(290, 0, 30, 28))

        self.btnLoad5 = QPushButton(self)
        self.btnLoad5.setObjectName('Save')
        self.btnLoad5.setText('Save')
        # self.btnLoad4.setStyleSheet("background-color : cyan;")
        self.btnLoad5.clicked.connect(self.nextPose)
        self.btnLoad5.setGeometry(QRect(320, 0, 93, 28))

        self.btnLoad6 = QPushButton(self)
        self.btnLoad6.setObjectName('<')
        self.btnLoad6.setText('<')
        # self.btnLoad5.setStyleSheet("background-color : cyan;")
        self.btnLoad6.clicked.connect(self.loadPrevious)
        self.btnLoad6.setGeometry(QRect(420, 0, 30, 28))

        self.btnLoad7 = QPushButton(self)
        self.btnLoad7.setObjectName('Load')
        self.btnLoad7.setText('Load')
        # self.btnLoad7.setStyleSheet("background-color : cyan;")
        self.btnLoad7.clicked.connect(self.open_dir_dialog)
        self.btnLoad7.setGeometry(QRect(450, 0, 93, 28))

        self.btnLoad8 = QPushButton(self)
        self.btnLoad8.setObjectName('>')
        self.btnLoad8.setText('>')
        self.btnLoad8.clicked.connect(self.loadNext)
        self.btnLoad8.setGeometry(QRect(543, 0, 30, 28))

        # Grid for images
        self.gridLayoutWidget1 = QWidget(self)
        self.gridLayoutWidget1.setGeometry(QRect(0, 50, 1880, 300))
        self.gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.gridLayout1 = QGridLayout(self.gridLayoutWidget1)
        self.gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout1.setObjectName("gridLayout")

        # Grid for Summary title
        self.gridLayoutWidget2 = QWidget(self)
        self.gridLayoutWidget2.setGeometry(QRect(0, 350, 100, 30))
        self.gridLayoutWidget2.setObjectName("gridLayoutWidget")
        self.gridLayout2 = QGridLayout(self.gridLayoutWidget2)
        self.gridLayout2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout2.setObjectName("gridLayout")

        # Grid for table
        self.gridLayoutWidget3 = QWidget(self)
        self.gridLayoutWidget3.setGeometry(QRect(0, 400, 1880, 500))
        self.gridLayoutWidget3.setObjectName("gridLayoutWidget")
        self.gridLayout3 = QGridLayout(self.gridLayoutWidget3)
        self.gridLayout3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout3.setObjectName("gridLayout")

        # add table widget
        self.table = QTableWidget()
        self.gridLayout3.addWidget(self.table)

        self.folder_path = ''
        self.live_view = ''
        self.pose = 1
        self.pose_count = 0
        self.last_pose = 0
        self.last_pose_count = 0

        self.setLayout(self.layout)

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

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        self.workspace_load()
        self.set_viewer(gridLayout=self.gridLayout1)
        return self.folder_path

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                if "workspace.pkl" not in files:
                    print('No "workspace.pkl" file found; Run workspace_export.py')
                else:
                    workspace_path = os.path.join(self.folder_path, [f for f in files if f == "workspace.pkl"][0])
                    self.workspace = pickle.load(open( workspace_path, "rb"))
                    self.cameras = self.workspace.names.camera
                    self.images = self.workspace.names.image
                    self.boards = self.workspace.names.board
                    self.last_pose_count = len(self.images)
                if 'calibration.detections.pkl' not in files:
                    print('No "calibration.detections.pkl" file found')
                else:
                    pickle_file = pickle.load(open(os.path.join(self.folder_path, 'calibration.detections.pkl'), 'rb'))
                    self.detectedPoints = pickle_file.detected_points
                if "calibration.json" not in files:
                    print('No "calibration.json" file found')
                else:
                    intrinsic_path = os.path.join(self.folder_path, 'calibration.json')
                    self.intrinsic = json.load(open(intrinsic_path))
                if "handEyeCamera.json" not in files:
                    print('No "handEyeCamera.json" file found; Run main4 of handEye_check.py')
                else:
                    handEye_path = os.path.join(self.folder_path, "handEyeCamera.json")
                    self.handEyeCamera = json.load(open(handEye_path))

        pass

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