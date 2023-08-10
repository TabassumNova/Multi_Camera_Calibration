# Ref
# https://stackoverflow.com/questions/35508711/how-to-enable-pan-and-zoom-in-a-qgraphicsview
# https://github.com/marcel-goldschen-ohm/PyQtImageViewer
# https://www.pythonguis.com/tutorials/pyqt-layouts/
# https://www.geeksforgeeks.org/pyqt5-qtabwidget/

import os.path
import pickle

from another_Window import *
import src.multical.workspace as ws
from src.pyqt_final2_tab2 import *
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
from src.multical.transform.rtvec import *
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


# Creating the main window
class App(QMainWindow):
    def __init__(self):
        super().__init__()
        self.title = "RAPtOr_GUI"
        self.left = 0
        self.top = 0
        self.width = 1000
        self.height = 800
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        # self.setStyleSheet("background-color : black;")


        self.tab_widget = MyTabWidget(self)
        # self.tab_widget.setStyleSheet("background-color : blue;")
        self.setCentralWidget(self.tab_widget)

        self.show()




# Creating tab widgets
class MyTabWidget(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)
        self.layout = QVBoxLayout(self)
        # self.setStyleSheet("background-color : black;")
        # Initialize tab screen
        self.tabs = QTabWidget()
        self.tab1 = QWidget()
        # self.tab1.setStyleSheet("background-color : black;")
        self.tab2 = QWidget()
        self.tab3 = QWidget()
        self.tab4 = QWidget()
        self.tab5 = QWidget()
        # self.tabs.resize(300, 200)
        # Add tabs
        self.tabs.addTab(self.tab1, "Operation")
        self.tabs.addTab(self.tab2, "Calibration")
        self.tabs.addTab(self.tab3, "Measurement Setup")
        self.tabs.addTab(self.tab4, "Cameras")
        self.tabs.addTab(self.tab5, "Settings")

        self.new_window = None

        self.operation_tab()
        #############################################
        self.calibration_tab()
        # tab2()
        # self.tab2 = Calibration_tab()
        #########
        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    def operation_tab(self):
        #################### TAB1 ##############################
        self.tab1.layout = QGridLayout(self)
        # self.tab1.setStyleSheet("background-color : black;")
        self.tab1_workspace = None
        self.tab1_cameras = None
        self.tab1_images = None
        self.tab1_boards = None
        self.tab1_detectedPoints = None
        self.tab1_intrinsic = None
        self.tab1_handEyeCamera = None
        # self.viewer = [None] * 8
        self.viewer = {}
        # add buttons
        self.btnLoad = QPushButton(self.tab1)
        self.btnLoad.setObjectName('Live View')
        self.btnLoad.setText('Live View')
        # self.btnLoad.setStyleSheet("background-image : url(button1.png);")
        self.btnLoad.setGeometry(QRect(0, 0, 93, 28))
        self.btnLoad.clicked.connect(self.saveImage)

        self.btnLoad1 = QPushButton(self.tab1)
        self.btnLoad1.setObjectName('Start')
        self.btnLoad1.setText('Start')
        # self.btnLoad1.setStyleSheet("background-color : cyan;")
        self.btnLoad1.setGeometry(QRect(90, 0, 93, 28))
        self.btnLoad1.clicked.connect(self.nextPose)

        self.btnLoad2 = QPushButton(self.tab1)
        self.btnLoad2.setObjectName('Stop')
        self.btnLoad2.setText('Stop')
        # self.btnLoad2.setStyleSheet("background-color : cyan;")
        self.btnLoad2.setGeometry(QRect(180, 0, 93, 28))
        self.btnLoad2.clicked.connect(self.nextPose)

        self.btnLoad3 = QPushButton(self.tab1)
        self.btnLoad3.setObjectName('x')
        self.btnLoad3.setText('x')
        # self.btnLoad3.setStyleSheet("background-color : cyan;")
        self.btnLoad3.clicked.connect(self.nextPose)
        self.btnLoad3.setGeometry(QRect(290, 0, 30, 28))

        self.btnLoad4 = QPushButton(self.tab1)
        self.btnLoad4.setObjectName('Save')
        self.btnLoad4.setText('Save')
        # self.btnLoad4.setStyleSheet("background-color : cyan;")
        self.btnLoad4.clicked.connect(self.nextPose)
        self.btnLoad4.setGeometry(QRect(320, 0, 93, 28))

        self.btnLoad5 = QPushButton(self.tab1)
        self.btnLoad5.setObjectName('<')
        self.btnLoad5.setText('<')
        # self.btnLoad5.setStyleSheet("background-color : cyan;")
        self.btnLoad5.clicked.connect(self.loadPrevious)
        self.btnLoad5.setGeometry(QRect(420, 0, 30, 28))

        self.btnLoad6 = QPushButton(self.tab1)
        self.btnLoad6.setObjectName('>')
        self.btnLoad6.setText('>')
        # self.btnLoad6.setStyleSheet("background-color : cyan;")
        self.btnLoad6.clicked.connect(self.loadNext)
        self.btnLoad6.setGeometry(QRect(543, 0, 30, 28))

        self.btnLoad7 = QPushButton(self.tab1)
        self.btnLoad7.setObjectName('Load')
        self.btnLoad7.setText('Load')
        # self.btnLoad7.setStyleSheet("background-color : cyan;")
        self.btnLoad7.clicked.connect(self.open_dir_dialog)
        self.btnLoad7.setGeometry(QRect(450, 0, 93, 28))

        # Grid for images
        self.gridLayoutWidget1 = QWidget(self.tab1)
        self.gridLayoutWidget1.setGeometry(QRect(0, 50, 1880, 300))
        self.gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.gridLayout1 = QGridLayout(self.gridLayoutWidget1)
        self.gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.gridLayout1.setObjectName("gridLayout")

        # Grid for Summary title
        self.gridLayoutWidget2 = QWidget(self.tab1)
        self.gridLayoutWidget2.setGeometry(QRect(0, 350, 100, 30))
        self.gridLayoutWidget2.setObjectName("gridLayoutWidget")
        self.gridLayout2 = QGridLayout(self.gridLayoutWidget2)
        self.gridLayout2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout2.setObjectName("gridLayout")

        # Grid for table
        self.gridLayoutWidget3 = QWidget(self.tab1)
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

        self.tab1.setLayout(self.tab1.layout)

    def calibration_tab(self):
        self.tab2.layout = QVBoxLayout(self)
        self.calibrationTab_poseCount = 0
        self.calibrationTab_LastposeCount = 0

        self.cb = QComboBox(self.tab2)
        # MyTabWidget.cb.addItem("No Selection")
        if self.tab1_handEyeCamera != None:
            items = len(self.tab1_handEyeCamera)
            for cam_num, cam_value in self.tab1_handEyeCamera.items():
                for k in cam_value.keys():
                    text = "Cam"+ cam_num+ " Group-" + str(k)
                    self.cb.addItem(text)
        self.cb.setGeometry(QRect(0, 0, 400, 28))
        self.cb.currentIndexChanged.connect(self.selectionchange)

        self.btnLoad8 = QPushButton(self.tab2)
        self.btnLoad8.setObjectName('<')
        self.btnLoad8.setText('<')
        # self.btnLoad5.setStyleSheet("background-color : cyan;")
        self.btnLoad8.clicked.connect(self.calibrationTab_loadPrevious)
        self.btnLoad8.setGeometry(QRect(410, 0, 30, 28))

        self.btnLoad9 = QPushButton(self.tab2)
        self.btnLoad9.setObjectName('>')
        self.btnLoad9.setText('>')
        # self.btnLoad6.setStyleSheet("background-color : cyan;")
        self.btnLoad9.clicked.connect(self.calibrationTab_loadNext)
        self.btnLoad9.setGeometry(QRect(533, 0, 30, 28))

        self.btnLoad10 = QPushButton(self.tab2)
        self.btnLoad10.setObjectName('Show visualization')
        self.btnLoad10.setText('Show visualization')
        # self.btnLoad6.setStyleSheet("background-color : cyan;")
        self.btnLoad10.clicked.connect(self.calibrationTab_showviz)
        self.btnLoad10.setGeometry(QRect(570, 0, 150, 28))

        self.labelLoad10 = QLabel(self.tab2)
        self.labelLoad10.setObjectName('Pose')
        self.labelLoad10.setText('Pose')
        # self.btnLoad7.setStyleSheet("background-color : cyan;")
        self.labelLoad10.setStyleSheet("border: 1px solid black;")
        # self.labelLoad10.clicked.connect(self.calibrationTab_Load)
        self.labelLoad10.setGeometry(QRect(440, 0, 93, 28))

        # Grid for images
        self.tab2_gridLayoutWidget1 = QWidget(self.tab2)
        self.tab2_gridLayoutWidget1.setGeometry(QRect(0, 50, 1880, 500))
        self.tab2_gridLayoutWidget1.setObjectName("gridLayoutWidget")
        self.tab2_gridLayout1 = QGridLayout(self.tab2_gridLayoutWidget1)
        self.tab2_gridLayout1.setContentsMargins(0, 0, 0, 0)
        self.tab2_gridLayout1.setObjectName("gridLayout")

        # Grid for table
        self.tab2_gridLayoutWidget3 = QWidget(self.tab2)
        self.tab2_gridLayoutWidget3.setGeometry(QRect(0, 600, 700, 700))
        self.tab2_gridLayoutWidget3.setObjectName("gridLayoutWidget")
        self.tab2_gridLayout3 = QGridLayout(self.tab2_gridLayoutWidget3)
        self.tab2_gridLayout3.setContentsMargins(0, 0, 0, 0)
        self.tab2_gridLayout3.setObjectName("gridLayout")
        self.tab2_table = QTableWidget()
        self.tab2_gridLayout3.addWidget(self.tab2_table)

        self.tab2.setLayout(self.tab2.layout)


    def calibrationTab_showviz(self):
        camera = str(self.cam_num)
        group = str(self.cam_group)
        self.draw_viz(camera, group)


    def draw_viz(self, master_cam, group):
        masterBoard_angles = self.tab1_handEyeCamera[master_cam][group]['masterBoard_angle']
        slaveBoard_angles = self.tab1_handEyeCamera[master_cam][group]['slaveBoard_angle']
        masterBoard_error = self.tab1_handEyeCamera[master_cam][group]['masterBoard_error']
        slaveBoard_error = self.tab1_handEyeCamera[master_cam][group]['slaveBoard_error']
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
                name=nameM
            )
        )
        nameS = self.tab1_handEyeCamera[master_cam][group]['slave_cam']
        final_layout.add_trace(
            go.Scatter3d(
                x=slave_x,
                y=slave_y,
                z=slave_z,
                name=nameS
            )
        )
        final_layout.update_layout(scene = dict(
                    xaxis_title='Roll',
                    yaxis_title='Pitch',
                    zaxis_title='Re-projection error'))
        final_layout.show()
        pass


    def calibrationTab_loadNext(self):
        if self.calibrationTab_LastposeCount-1 > self.calibrationTab_poseCount >= 0:
            self.calibrationTab_poseCount += 1
            self.clearLayout(self.tab2_gridLayout1)
            # self.set_viewer(gridLayout=self.gridLayout1)
            self.selectionchange(self.group, self.calibrationTab_poseCount)
        else:
            self.clearLayout(self.tab2_gridLayout1)
            print("Pose end")
        # return 0

    def calibrationTab_loadPrevious(self):
        if self.calibrationTab_poseCount > 0:
            self.calibrationTab_poseCount -= 1
            self.clearLayout(self.tab2_gridLayout1)
            # self.set_viewer(gridLayout=self.tab2_gridLayout1)
            self.selectionchange(self.group, self.calibrationTab_poseCount)
        else:
            self.clearLayout(self.tab2_gridLayout1)
        # return 0


    def create_ImageViewer(self, folder_path = True):
        if folder_path:
            for c in self.tab1_cameras:
                self.viewer[c] = QtImageViewer()
                self.viewer[c].leftMouseButtonReleased.connect(self.handleLeftClick)
        pass

    def open_ImageViewer(self, gridLayout):
        for path, subdirs, files in os.walk(self.folder_path):
            if path == self.folder_path:
                for idx, dir in enumerate(subdirs):
                    v = os.path.join(self.folder_path, dir, self.tab1_images[self.pose_count])
                    self.viewer[dir].open(v)
                    gridLayout.addWidget(self.viewer[dir], 0, idx)
        self.clearLayout(self.gridLayout2)
        label1 = QLabel()
        label1.setText('Pose')
        label1.setFont(QFont("Times", 10, QFont.Bold))
        label1.setAlignment(Qt.AlignCenter)
        self.gridLayout2.addWidget(label1, 2, 0)

        label = QLabel()
        label.setText(self.tab1_images[self.pose_count])
        label.setAlignment(Qt.AlignCenter)
        self.gridLayout2.addWidget(label, 2, 1)
        self.gridLayout2.addWidget(label, 2, 1)

    def add_cameraLabel(self, gridLayout):
        for idx, cam in enumerate(self.tab1_cameras):
            btn = QPushButton(self.tab1)
            btn.setObjectName(cam)
            btn.setText(cam)
            btn.clicked.connect(lambda checked: self.show_CamImages())
            # btn.setGeometry(QRect(543, 0, 30, 28))
            gridLayout.addWidget(btn, 1, idx)

    def show_CamImages(self):
        if self.new_window is None:
            self.new_window = AnotherWindow(self.folder_path, self.tab1_workspace)
        self.new_window.resize(500, 500)
        self.new_window.show()

    def set_viewer(self, gridLayout, group_name="group01", live_view=False, cameraImgs={}):
        if self.folder_path:
            # # Create image viewer.
            self.create_ImageViewer()
            self.open_ImageViewer(gridLayout)
            self.add_cameraLabel(gridLayout)
            # self.add_3d_scatter()
            self.add_table_widget()

    def add_table_widget(self):
        self.table.setRowCount(len(self.tab1_boards))
        self.table.setColumnCount(len(self.tab1_cameras))
        self.table.setHorizontalHeaderLabels(self.tab1_cameras)
        self.table.setVerticalHeaderLabels(self.tab1_boards)

        for cam in self.tab1_cameras:
            camera_id = self.tab1_cameras.index(cam)
            for board in self.tab1_boards:
                board_id = self.tab1_boards.index(board)
                num_points = self.tab1_workspace.pose_table.num_points[camera_id][self.pose_count][board_id]
                repo_error = "{:.2f}".format(self.tab1_workspace.pose_table.reprojection_error[camera_id][self.pose_count][board_id])
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

    def group_decode(self, group):
        group_num = {}
        total = 0
        for cam_num, cam_group in self.tab1_handEyeCamera.items():
            group_num[cam_num] = np.arange(total, total+len(self.tab1_handEyeCamera[cam_num]))
            total += len(self.tab1_handEyeCamera[cam_num])
            if group in group_num[cam_num]:
                return cam_num, np.where(group_num[cam_num]==group)[0][0]
        pass

    def selectionchange(self, group, poseCount=0):
        if poseCount == 0:
            self.calibrationTab_poseCount = 0
        self.cam_num, self.cam_group = self.group_decode(group)

        self.group = group
        handEye_Cam = self.tab1_handEyeCamera[self.cam_num][str(self.cam_group)]
        self.calibrationTab_LastposeCount = len(handEye_Cam["image_list"])
        master_cam = handEye_Cam["master_cam"]
        master_board = handEye_Cam["master_board"]
        slave_cam = handEye_Cam["slave_cam"]
        slave_board = handEye_Cam["slave_board"]
        image_list = handEye_Cam["image_list"]
        master_path = os.path.join(self.folder_path, master_cam, image_list[poseCount])
        slave_path = os.path.join(self.folder_path, slave_cam, image_list[poseCount])
        pose = 'Pose '+ image_list[poseCount][1:-4]
        self.labelLoad10.setText(pose)
        imageLabel1 = self.image_load(master_path, master_cam, master_board, image_list[poseCount])
        imageLabel2 = self.image_load(slave_path, slave_cam, slave_board, image_list[poseCount])
        # self.clearLayout(layout)
        self.tab2_gridLayout1.addWidget(imageLabel1, 0, 0)
        self.tab2_gridLayout1.addWidget(imageLabel2, 0, 1)

        self.calibrationTab_Table()
        # print(self.group)
        pass

    def image_load(self, path, cam, board, image):
        frame = cv2.imread(path)
        cam_id = self.tab1_cameras.index(cam)
        board_id = self.tab1_boards.index(board)
        img_id = self.tab1_images.index(image)
        corners = self.tab1_detectedPoints[cam_id][img_id][board_id].corners

        cam_matrix = np.array(self.tab1_intrinsic["cameras"][cam]['K'])
        cam_dist = np.array(self.tab1_intrinsic["cameras"][cam]['dist'])
        rtvecs = from_matrix(np.array(self.tab1_workspace.pose_table.poses[cam_id][img_id][board_id]))
        marker_length = self.tab1_workspace.boards[board_id].marker_length
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

    def draw_corners(self, frame, corners):
        for c in corners:
            x = tuple(c.astype('int'))
            frame = cv2.circle(frame, x, radius=0, color=(0, 0, 255), thickness=30)
        return frame

    def calibrationTab_Table(self):
        self.tab2_table.setRowCount(7)
        self.tab2_table.setColumnCount(1)
        self.tab2_table.setHorizontalHeaderLabels(["Values"])
        self.tab2_table.setVerticalHeaderLabels(["Master Cam", "Master Board", "Slave Cam",
                                                 "Slave Board", "Num_poses", "Initial_Reprojection_Error", "Final_Reprojection_Error"])
        item1 = QTableWidgetItem()
        item2 = QTableWidgetItem()
        item3 = QTableWidgetItem()
        item4 = QTableWidgetItem()
        item5 = QTableWidgetItem()
        item6 = QTableWidgetItem()
        item7 = QTableWidgetItem()
        handEye_Cam = self.tab1_handEyeCamera[self.cam_num][str(self.cam_group)]
        item1.setText(str(handEye_Cam["master_cam"]))
        item2.setText(str(handEye_Cam["master_board"]))
        item3.setText(str(handEye_Cam["slave_cam"]))
        item4.setText(str(handEye_Cam["slave_board"]))
        item5.setText(str(len(handEye_Cam["image_list"])))
        item6.setText(str("{:.2f}".format(float(handEye_Cam["initial_reprojection_error"]))))
        item7.setText(str("{:.2f}".format(float(handEye_Cam["final_reprojection_error"]))))
        self.tab2_table.setItem(0, 0, item1)
        self.tab2_table.setItem(1, 0, item2)
        self.tab2_table.setItem(2, 0, item3)
        self.tab2_table.setItem(3, 0, item4)
        self.tab2_table.setItem(4, 0, item5)
        self.tab2_table.setItem(5, 0, item6)
        self.tab2_table.setItem(6, 0, item7)

    # def image_load(self, path):
    #     frame = cv2.imread(path)
    #     h, w, ch = frame.shape
    #     bytes_per_line = ch * w
    #     convert_to_Qt_format = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
    #     p = convert_to_Qt_format.scaled(700, 700, Qt.KeepAspectRatio)
    #     imageLabel = QLabel()
    #     x = QPixmap.fromImage(p)
    #     print(h, w, ch)
    #     imageLabel.setPixmap(x)
    #     return imageLabel

    def open_dir_dialog(self):
        dialog = QFileDialog()
        self.folder_path = dialog.getExistingDirectory(None, "Select Folder")
        # print(self.folder_path)
        self.workspace_load()
        self.calibration_tab()
        self.set_viewer(gridLayout=self.gridLayout1)
        return self.folder_path

    def workspace_load(self):
        for path, subdirs, files in os.walk((self.folder_path)):
            if path == self.folder_path:
                workspace_path = os.path.join(self.folder_path, [f for f in files if f == "workspace.pkl"][0])
                self.tab1_workspace = pickle.load(open( workspace_path, "rb"))
                self.tab1_cameras = self.tab1_workspace.names.camera
                self.tab1_images = self.tab1_workspace.names.image
                self.tab1_boards = self.tab1_workspace.names.board
                self.last_pose_count = len(self.tab1_images)
                for file in files:
                    if file == 'calibration.detections.pkl':
                        pickle_file = ws.Workspace.load(os.path.join(self.folder_path, 'calibration.detections.pkl'))
                        self.tab1_detectedPoints = pickle_file.detected_points
                    if file == "calibration.json":
                        intrinsic_path = os.path.join(self.folder_path, 'calibration.json')
                        self.tab1_intrinsic = json.load(open(intrinsic_path))
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.folder_path, "handEyeCamera.json")
                        self.tab1_handEyeCamera = json.load(open(handEye_path))

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

    def clearLayout(self, layout):
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def handleLeftClick(self, x, y):
        row = int(y)
        column = int(x)
        print("Clicked on image pixel (row=" + str(row) + ", column=" + str(column) + ")")

    def handleViewChange(self):
        print("viewChanged")

    def saveImage(self):
        print("Button clicked, Hello! Save Image")

    def nextPose(self):
        print("Button clicked, Next!")
        self.set_viewer(group_name="group02")
        self.show()

    def add_3d_scatter(self):
        point_cloud = np.random.random((100, 3))
        pdata = pv.PolyData(point_cloud)
        pdata['orig_sphere'] = np.arange(100)
        print('add scatter')
        # create many spheres from the point cloud
        sphere = pv.Sphere(radius=0.02, phi_resolution=10, theta_resolution=10)
        pc = pdata.glyph(scale=False, geom=sphere, orient=False)
        # pc.plot(cmap='Reds')
        # self.plotter.subplot(1, 1)
        self.plotter.add_mesh(pc, show_edges=True)
        self.plotter.reset_camera()


    def change_name(self):
        self.name_label.setText("Name Changed")


# def multiprocess_loading(viewer, folder_path, camera, pose, gridLayout):
def multiprocess_loading(viewer, folder_path, camera, pose):
    viewer = QtImageViewer()
    v = folder_path + '/' + camera + '/p' + str(pose) + '.png'
    viewer.open(v)
    print('multiprocess: ', pose)



def f(name):
    print('hello', name)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())
