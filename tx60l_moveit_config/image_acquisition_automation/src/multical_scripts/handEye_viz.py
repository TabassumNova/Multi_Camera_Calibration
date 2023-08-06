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

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\isohedron\V31"
'''
for pairwise handeye board visualization
'''

class Interactive_Board():
    def __init__(self, base_path):
        self.base_path = base_path
        # self.workspace, self.handEye_group, self.campose2 = self.load_files()
        self.workspace = None
        self.handEye_group = None
        self.campose2 = None
        self.camera_pose = {}
        self.load_files()
        self.camera_color = {}
        self.set_Cam_color()
        # self.num_group = len(self.handEye)
        self.groups = {}
        # self.select_group()
        # self.draw_groups()
        pass

    def set_Cam_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime']
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]

    def draw_boards(self, master_cam, group):
        visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
        final_layout = go.Figure()
        handEye_group = self.handEye_group[master_cam][group]
        slave_cam = handEye_group['slave_cam']
        for idx in range(len(handEye_group['image_list'])):
            poseM_board = np.linalg.inv(handEye_group['masterCam_wrt_masterB'][idx])
            poseS_board = np.linalg.inv(handEye_group['slaveCam_wrto_slaveB'][idx])
            poseM_cam = self.camera_pose[master_cam]
            poseS_cam = self.camera_pose[slave_cam]

            name = handEye_group['image_list'][idx]

            data_camM = visualizer.extrinsic2pyramid(poseM_cam, color=self.camera_color[master_cam],
                                                focal_len_scaled=0.1, aspect_ratio=0.1, show_legend=False,
                                                hover_template=master_cam)
            data_camS = visualizer.extrinsic2pyramid(poseS_cam, color=self.camera_color[slave_cam],
                                                 focal_len_scaled=0.1, aspect_ratio=0.1,
                                                 hover_template=slave_cam, name=name)

            data_boardM = visualizer.extrinsic2Board(poseM_board, poseM_cam, color=self.camera_color[master_cam],
                                                focal_len_scaled=0.1, aspect_ratio=0.1,
                                                hover_template=master_cam, name=name)
            data_boardS = visualizer.extrinsic2Board(poseS_board, poseS_cam, color=self.camera_color[slave_cam],
                                                 focal_len_scaled=0.1, aspect_ratio=0.1,
                                                 hover_template=slave_cam, name=name)

            final_layout.add_trace(data_camM)
            final_layout.add_trace(data_camS)
            final_layout.add_trace(data_boardM)
            final_layout.add_trace(data_boardS)
        final_layout.show()
        pass

    def write_html(self, figure):
        buffer = io.StringIO()
        # pio.write_html(fig, file='Dataset_006.html', auto_open=True)
        figure.write_html(buffer)

        html_bytes = buffer.getvalue().encode()
        encoded = b64encode(html_bytes).decode()
        app = JupyterDash(__name__)
        app.layout = html.Div([
            dcc.Dropdown(['NYC', 'MTL', 'SF'], 'All', id='demo-dropdown', style={'width': '49%', 'display': 'inline-block', 'vertical-align': 'left'}),
            html.Div(id='dd-output-container'),
            dcc.Graph(id="graph1", figure=figure),
            html.A(
                html.Button("Download HTML"),
                id="download",
                href="data:text/html;base64," + encoded,
                download="plotly_graph.html"
            ),
        ])

        @callback(
            Output('dd-output-container', 'children'),
            Input('demo-dropdown', 'value')
        )
        def update_output(value):
            return f'You have selected {value}'
        app.run_server(debug=True)
        pass

    def layout(self, show_legend=False, w=1000, h=1000):
        axis = dict(showbackground=True, showline=False, zeroline=False, showgrid=True, showticklabels=False, title='')
        layout = go.Layout(title="Dataset", width=w,
                           height=h,
                           showlegend=show_legend,
                           scene=dict(xaxis=dict(axis),
                                      yaxis=dict(axis),
                                      zaxis=dict(axis)
                                      ),
                           margin=dict(t=100),
                           hovermode='closest')
        return layout

    def select_group(self):
        all_fig = []
        for cam_name, camera in self.handEye_group.items():
            self.groups[cam_name] = {}
            for group_num, value in camera.items():
                masterCam = value['master_cam']
                slaveCam = value['slave_cam']
                name = "M" + masterCam + '_S' + slaveCam
                if name not in self.groups[cam_name]:
                    self.groups[cam_name][name] = {}
                    self.groups[cam_name][name][group_num] = value
                else:
                    self.groups[cam_name][name][group_num] = value
        pass

    def load_files(self):
        # workspace, handEye, campose2 = None, None, None
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
                self.workspace = pickle.load(open(workspace_path, "rb"))
                for file in files:
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.base_path, "handEyeCamera.json")
                        self.handEye_group = json.load(open(handEye_path))
                    if file == "Calibration_handeye.json":
                        calib_path = os.path.join(self.base_path, "Calibration_handeye.json")
                        self.load_campose(calib_path)
                    if file == "campose2.json":
                        campose2_path = os.path.join(self.base_path, "campose2.json")
                        self.campose2 = json.load(open(campose2_path))

    def load_campose(self, path):
        calib = json.load(open(path))
        for k in calib['camera_poses']:
            if k in self.workspace.names.camera:
                cam = k
            else:
                source, dest = k.split("_to_")
                cam = source
            R = np.array(calib['camera_poses'][k]['R'])
            t = matrix.translation(np.array(calib['camera_poses'][k]['T']))
            self.camera_pose[cam] = matrix.join(R, t)
        pass



if __name__ == '__main__':
    # argument : the minimum/maximum value of x, y, z
    v = Interactive_Board(base_path)

    # visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
    #
    # # argument : extrinsic matrix, color, scaled focal length(z-axis length of frame body of camera
    # all_fig = []
    # data = visualizer.extrinsic2pyramid(np.eye(4), 'c', focal_len_scaled=0.1, aspect_ratio=0.3)
    # all_fig.append(data)
    # final_layout = go.Figure(data= all_fig)
    # # final_layout.add_trace(all_fig)
    # final_layout.show()
    # visualizer.show()
