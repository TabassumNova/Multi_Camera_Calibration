import numpy as np
from src.extrinsic2pyramid.camera_pose_visualizer import CameraPoseVisualizer
import plotly.graph_objects as go
import json
import os
from dash import Dash, dcc, html, Input, Output,callback
import plotly.io as pio
import io
from base64 import b64encode
from src.multical.transform.rtvec import *
from jupyter_dash import JupyterDash
from dash import dcc
from dash import html
from dash.dependencies import Input, Output
import pickle
from src.multical.transform import common, rtvec

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35_test"
'''
Single calibration.json visualization
'''

class Interactive_calibration():
    def __init__(self, base_path):
        self.base_path = base_path
        # self.workspace, self.handEye_group, self.campose2 = self.load_files()
        self.workspace = None
        self.handEye_group = None
        self.campose2 = None
        self.camera_pose_final = {}
        self.camera_pose_init = {}
        self.load_files()
        self.camera_color = {}
        self.set_Cam_color()
        self.draw_cameras()
        pass

    def set_Cam_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime']
        for idx, cam in enumerate(self.camera_pose_final.keys()):
            self.camera_color[cam] = colors[idx]

    def draw_cameras(self):
        visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
        final_layout = go.Figure()

        for cam, pose in self.camera_pose_final.items():
            data = visualizer.extrinsic2pyramid(pose, color=self.camera_color[cam],
                                                     focal_len_scaled=0.1, aspect_ratio=0.3, show_legend=False,
                                                     hover_template=cam+'_final')

            data1 = visualizer.extrinsic2pyramid(self.camera_pose_init[cam], color=self.camera_color[cam],
                                                focal_len_scaled=0.1, aspect_ratio=0.3, show_legend=False,
                                                hover_template=cam+'_init')
            final_layout.add_trace(data)
            final_layout.add_trace(data1)
        final_layout.show()
        pass

    def load_files(self):
        # workspace, handEye, campose2 = None, None, None
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                for file in files:
                    if file == "calibration.json":
                        calib_path = os.path.join(self.base_path, "calibration.json")
                        self.load_campose(calib_path, calib_type='final')
                    if file == "Calibration_handeye.json":
                        init_calib_path = os.path.join(self.base_path, "Calibration_handeye.json")
                        self.load_campose(init_calib_path, calib_type='initial')


    def load_campose(self, path, calib_type):
        calib = json.load(open(path))
        for k in calib['camera_poses']:
            if k in calib['cameras']:
                cam = k
            else:
                source, dest = k.split("_to_")
                cam = source
            R = np.array(calib['camera_poses'][k]['R'])
            t = np.array(calib['camera_poses'][k]['T'])
            if calib_type == 'final':
                # the final pose from multical masterCam_wrto_slaveCam
                self.camera_pose_final[cam] = np.linalg.inv(matrix.join(R, t))
            if calib_type == 'initial':
                # the pose from hand-eye calibration slaveCam_wrto_masterCam
                self.camera_pose_init[cam] = matrix.join(R, t)
        pass



if __name__ == '__main__':
    # argument : the minimum/maximum value of x, y, z
    v = Interactive_calibration(base_path)


