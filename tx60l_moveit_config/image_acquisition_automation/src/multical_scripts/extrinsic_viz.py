import numpy as np
from src.extrinsic2pyramid.util.camera_pose_visualizer import CameraPoseVisualizer
import plotly.graph_objects as go
import json
import os
from dash import Dash, dcc, html, Input, Output,callback
import plotly.io as pio
import io
from base64 import b64encode

from jupyter_dash import JupyterDash
from dash import dcc
from dash import html
from dash.dependencies import Input, Output
import pickle

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\isohedron\V31"

class Interactive_Extrinsic():
    def __init__(self, base_path):
        self.base_path = base_path
        self.workspace, self.handEye_group = self.load_files()
        self.camera_color = {}
        self.set_Cam_color()
        # self.num_group = len(self.handEye)
        self.groups = {}
        self.select_group()
        self.draw_groups()
        pass

    def set_Cam_color(self):
        colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime']
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]

    def draw_groups(self):
        all_fig = []
        visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
        final_layout = go.Figure()
        final_layout.layout.update(
            updatemenus=[
                go.layout.Updatemenu(
                    direction="down", buttons=list(
                        [
                            dict(args=["type", "Group1"], label="Group1", method="update"),
                        ]
                    )
                ),
            ]
        )
        # final_layout = self.layout(show_legend=True, w=1500, h=1500)
        for key, group in self.groups.items():
            all_fig = []
            for key2, value in group.items():
                master_cam = value['master_cam']
                slave_cam = value['slave_cam']
                master_extrinsic = np.eye(4)
                slave_extrinsic = np.array(value['slaveCam_wrto_masterCam'])
                name = key + "_" + key2
                data = visualizer.extrinsic2pyramid(master_extrinsic, color=self.camera_color[master_cam],
                                                    focal_len_scaled=0.1, aspect_ratio=0.3, show_legend=False, hover_template=master_cam)
                data1 = visualizer.extrinsic2pyramid(slave_extrinsic, color=self.camera_color[slave_cam],
                                                     focal_len_scaled=0.1, aspect_ratio=0.3, hover_template=slave_cam, name=name)
                all_fig.append(data)
                all_fig.append(data1)
                final_layout.add_trace(data)
                final_layout.add_trace(data1)
            pass

        # final_layout = go.Figure(data=all_fig)

        # fig = go.Figure(data=all_fig, layout=final_layout)
        # self.write_html(fig)

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
        for key, value in self.handEye_group.items():
            masterCam = value['master_cam']
            slaveCam = value['slave_cam']
            name = "M" + masterCam + '_S' + slaveCam
            if name not in self.groups:
                self.groups[name] = {}
                self.groups[name][key] = value
            else:
                self.groups[name][key] = value
        pass

    def load_files(self):
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
                workspace = pickle.load(open(workspace_path, "rb"))
                for file in files:
                    if file == "handEyeCamera.json":
                        handEye_path = os.path.join(self.base_path, "handEyeCamera.json")
                        handEye = json.load(open(handEye_path))
        return workspace, handEye



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
