import numpy as np
from src.extrinsic2pyramid.camera_pose_visualizer import CameraPoseVisualizer
import plotly.graph_objects as go
import json
import os
# from dash import Dash, dcc, html, Input, Output,callback
import plotly.io as pio
import io
from base64 import b64encode
from src.multical.transform.rtvec import *
# from jupyter_dash import JupyterDash
# from dash import dcc
# from dash import html
# from dash.dependencies import Input, Output
import pickle
from src.multical.transform import common, rtvec
from scipy import stats
from mayavi import mlab
import pandas as pd
import plotly.express as px

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V41"
'''
for camera extrinsic visualization
'''
class Interactive_Extrinsic():
    def __init__(self, base_path):
        self.base_path = base_path
        self.workspace = None
        self.handEye = None
        self.campose2 = None
        self.mean_cameras = None
        self.load_files()
        self.camera_color = {}
        self.set_Cam_color()
        # self.num_group = len(self.handEye)
        self.groups = {}
        self.select_group()
        # self.draw_heat_map()
        self.draw_groups()
        pass

    def set_Cam_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime', 'pink', 'teal', 'darkcyan', 'violet', 'brown', 'indigo']
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]

    def draw_heat_map(self):
        for cam_name, cam_value in self.groups.items():
            for key, group in cam_value.items():
                if len(group) > 2:
                    x = []
                    y = []
                    z = []
                    group_name = []
                    master_cam, slave_cam = key.split('_')
                    if master_cam[1:] != slave_cam[1:]:
                        for key2, value in group.items():
                            master_extrinsic = np.eye(4)
                            slave_extrinsic = np.array(value['slaveCam_wrto_masterCam'])
                            rvec, tvec = split(as_rtvec(slave_extrinsic))
                            x.append(tvec[0])
                            y.append(tvec[1])
                            z.append(tvec[2])
                            group_name.append(key2)
                        xyz = np.vstack([x, y, z])
                        kde = stats.gaussian_kde(xyz)
                        density = kde(xyz)
                        max_idx = np.argmax(density)
                        max_group = group_name[max_idx]
                        print(key, ' : ', max_group)
                        data = {'x': x, 'y': y, 'z': z, 'density': density}
                        df = pd.DataFrame(data)
                        fig = px.scatter_3d(df, x='x', y='y', z='z',
                                            color='density', title=key)
                        fig.show()
                        for idx, k in enumerate(self.groups[cam_name][key].keys()):
                            self.groups[cam_name][key][k]['density'] = density[idx]
                pass
        pass

    def draw_groups(self):
        all_fig = []
        visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])

        for cam_name, cam_value in self.groups.items():
            # # all_fig = []
            # add annotation
            final_layout = go.Figure()
            folder = self.base_path[-3:]
            final_layout.add_annotation(dict(font=dict(color='black', size=20),
                                    x=0,
                                    y=0.12,
                                    showarrow=False,
                                    text=folder + '-' + cam_name,
                                    textangle=0,
                                    xanchor='left',
                                    xref="paper",
                                    yref="paper"))
            mean_calculation = {}
            for key, group in cam_value.items():
                # all_fig = []
                for key2, value in group.items():
                    master_cam = value['master_cam']
                    slave_cam = value['slave_cam']
                    master_extrinsic = np.eye(4)
                    slave_extrinsic = np.array(value['slaveCam_wrto_masterCam'])
                    rvec, tvec = split(as_rtvec(slave_extrinsic))
                    # name = key + "_" + key2
                    name = "Master : " + master_cam + "\n" + "Slave: " + slave_cam + "\n" + "Group: " + "\n" + key2

                    data = visualizer.extrinsic2pyramid(master_extrinsic, color=self.camera_color[master_cam],
                                                        focal_len_scaled=0.1, aspect_ratio=0.3, show_legend=False, hover_template=master_cam)
                    data1 = visualizer.extrinsic2pyramid(slave_extrinsic, color=self.camera_color[slave_cam],
                                                         focal_len_scaled=0.1, aspect_ratio=0.3,
                                                         hover_template=slave_cam+ "_" + str(tvec), name=name)
                    # all_fig.append(data)
                    # all_fig.append(data1)
                    final_layout.add_trace(data)
                    final_layout.add_trace(data1)


            if self.mean_cameras:
                for slaveC in self.mean_cameras[cam_name]:
                    slave_extrinsic = np.array(self.mean_cameras[cam_name][slaveC]['extrinsic'])
                    d = visualizer.extrinsic2pyramid(slave_extrinsic, color='black', focal_len_scaled=0.1, aspect_ratio=0.3,
                                                                        hover_template="mean", name=slaveC)
                    final_layout.add_trace(d)

            final_layout.show()
        pass

    def show_cluster_mean(self, mean_calculation):
        mean_group_dict = {}
        for key, value in mean_calculation.items():
            x = rtvec.to_matrix(common.mean_robust(value))
            mean_group_dict[key] = x

        return mean_group_dict

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
        for cam_name, camera in self.handEye.items():
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
        workspace, handEye, campose2 = None, None, None
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
                self.workspace = pickle.load(open(workspace_path, "rb"))
                for file in files:
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.base_path, "handEyeCamera.json")
                        self.handEye = json.load(open(handEye_path))
                    if file == "campose2.json":
                        campose2_path = os.path.join(self.base_path, "campose2.json")
                        self.campose2 = json.load(open(campose2_path))
                    if file == 'meanCameras.json':
                        meanCam_path = os.path.join(self.base_path, 'meanCameras.json')
                        self.mean_cameras = json.load(open(meanCam_path))




if __name__ == '__main__':
    # argument : the minimum/maximum value of x, y, z
    v = Interactive_Extrinsic(base_path)

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
