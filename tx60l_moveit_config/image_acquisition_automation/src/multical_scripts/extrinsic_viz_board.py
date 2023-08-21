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
for camera extrinsic visualization
'''
class Interactive_Extrinsic_Board():
    def __init__(self, calib_obj, board_names):
        self.board_names = board_names
        self.calibObj = calib_obj
        self.board_color = {}
        self.set_board_color()
        # self.num_group = len(self.handEye)
        self.groups = {}
        # self.select_group()
        self.draw_groups()
        pass

    def set_board_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime',
                  'pink', 'teal', 'darkcyan', 'violet', 'brown',
                  'indigo', 'orange', 'orchid', 'chocolate', 'coral', ' gray', 'maroon']
        for board in self.calibObj.keys():
            id = self.board_names.index(board)
            self.board_color[board] = colors[id]

    def draw_groups(self):
        all_fig = []
        visualizer = CameraPoseVisualizer([-250, 250], [-250, 250], [-250, 250])
        final_layout = go.Figure()
        final_layout.add_annotation(dict(font=dict(color='black', size=20),
                                         x=0,
                                         y=0.12,
                                         showarrow=False,
                                         text='Boards',
                                         textangle=0,
                                         xanchor='left',
                                         xref="paper",
                                         yref="paper"))

        for board_key, board_value in self.calibObj.items():

            for pose in board_value['group']:
                data = visualizer.extrinsic2Board(pose, color=self.board_color[board_key],
                                                    focal_len_scaled=0.5, aspect_ratio=0.5, show_legend=True,
                                                    hover_template=board_key, name='board_'+str(board_key))

                final_layout.add_trace(data)
            data1 = visualizer.extrinsic2Board(board_value['mean'], color='black',
                                              focal_len_scaled=0.5, aspect_ratio=0.5, show_legend=True,
                                              hover_template=board_key, name='mean_'+str(board_key))

            final_layout.add_trace(data1)

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
        workspace, handEye, campose2 = None, None, None
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
                workspace = pickle.load(open(workspace_path, "rb"))
                for file in files:
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.base_path, "handEyeCamera.json")
                        handEye = json.load(open(handEye_path))
                    if file == "campose2.json":
                        campose2_path = os.path.join(self.base_path, "campose2.json")
                        campose2 = json.load(open(campose2_path))
        return workspace, handEye, campose2



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
