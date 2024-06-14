import numpy as np
import matplotlib.pyplot as plt
from src.extrinsic2pyramid.camera_pose_visualizer import CameraPoseVisualizer
import plotly.graph_objects as go
import json
import os
# from dash import Dash, dcc, html, Input, Output,callback
import plotly.io as pio
import io
from base64 import b64encode
from src.multical.transform.rtvec import *
import networkx as nx
# from jupyter_dash import JupyterDash
# from dash import dcc
# from dash import html
# from dash.dependencies import Input, Output
import pickle
from src.multical.transform import common, rtvec
from scipy import stats
# from mayavi import mlab
import pandas as pd
import plotly.express as px

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
        self.board_names = []

        self.load_files()
        self.set_board_names()
        self.camera_color = {}
        self.set_Cam_color()
        # self.num_group = len(self.handEye)
        self.groups = {}
        self.select_group()
        self.draw_heat_map()
        self.draw_groups()
        pass

    def set_board_names(self):
        for i in range(len(self.workspace.names.board)):
            t = 'Board-'+str(i+1)
            self.board_names.append(t)
    def set_Cam_color(self):
        # colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime', 'pink', 'teal', 'darkcyan', 'violet', 'brown', 'indigo']
        # colors = ['blue', 'darkblue', 'green', 'darkgreen', 'olive', 'navy']
        colors = px.colors.sequential.Aggrnyl
        # print(x)
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]

    def draw_board_network(self):
        G = nx.Graph()
        G.add_edge(1, 2)
        G.add_edge(1, 3)
        G.add_edge(1, 5)
        G.add_edge(2, 3)
        G.add_edge(3, 4)
        G.add_edge(4, 5)

        # explicitly set positions
        pos = {1: (0, 0), 2: (-1, 0.3), 3: (2, 0.17), 4: (4, 0.255), 5: (5, 0.03)}

        options = {
            "font_size": 36,
            "node_size": 3000,
            "node_color": "white",
            "edgecolors": "black",
            "linewidths": 5,
            "width": 5,
        }
        nx.draw_networkx(G, pos, with_labels=False,  **options)

        # Set margins for the axes so that nodes aren't clipped
        ax = plt.gca()
        ax.margins(0.20)
        plt.axis("off")
        # plt.show()
        plt.savefig('net.png', bbox_inches='tight')

    def draw_heat_map(self):
        num_boards = len(self.workspace.names.board)
        for cam_name, cam_value in self.groups.items():
            board_map = np.zeros((num_boards, num_boards))
            data_list = []
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
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')

            for key, group in cam_value.items():
                if len(group) > 2:
                    i = 1
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
                            mb = self.workspace.names.board.index(value['master_board'])
                            sb = self.workspace.names.board.index(value['slave_board'])
                            board_map[mb, sb] = int(i)
                            i+=1

                        xyz = np.vstack([x, y, z])
                        kde = stats.gaussian_kde(xyz)
                        density = kde(xyz)
                        max_idx = np.argmax(density)
                        density = density/density.max()
                        for p in range(num_boards):
                            for q in range(num_boards):
                                if board_map[p][q]!=0:
                                    # x = board_map[p][q]-1
                                    # d = density[0]
                                    board_map[p][q] = density[int(board_map[p][q]-1)]

                        # save board_map
                        outfile = self.base_path + '/BoardMap_'+key
                        np.save(outfile, board_map)
                        self.draw_board_network()
                        # confusion matrix
                        fig = px.imshow(board_map, color_continuous_scale='Greens',
                                        labels=dict(x="Slave Boards", y="Master Boards"),
                                        x=self.board_names,
                                        y=self.board_names
                                        )

                        for i in range(num_boards):
                            fig.add_shape(type="line", x0=0.5 + i, y0=-0.5, x1=0.5 + i, y1=num_boards - 0.5,
                                          line=dict(color="white", width=2))

                        for i in range(num_boards):
                            fig.add_shape(type="line", x0=-0.5, y0=0.5 + i, x1=num_boards - 0.5, y1=0.5 + i,
                                          line=dict(color="white", width=2))
                        # fig.update_xaxes(side="top")
                        fig.add_shape(
                            type='rect',
                            x0=-0.5, x1=18 - 0.5, y0=-0.5, y1=18 - 0.5,
                            xref='x', yref='y',
                            line_color='black'
                        )
                        fig.update_xaxes(
                            tickangle=90,
                            title_font={"size": 20},
                            title_standoff=25)

                        fig.update_yaxes(
                            title_font={"size": 20},
                            title_standoff=25)
                        # fig.update_coloraxes(showscale=False)
                        fig.show()

                        max_group = group_name[max_idx]
                        print(key, ' : ', max_group)
                        data = {'x': x, 'y': y, 'z': z, 'density': density}
                        df = pd.DataFrame(data)
                        fig = px.scatter_3d(df, x='x', y='y', z='z',
                                            color='density', color_continuous_scale='Blugrn', title=key)
                        # fig.show()
                        name = "Master : " + master_cam + "\n" + "Slave: " + slave_cam + "\n" + "Group: "

                        data_list.extend([go.Scatter3d(
                            x=x,
                            y=y,
                            z=z,
                            mode='markers',
                            name= name,
                            marker=dict(
                                size=7,
                                color=density,
                                colorscale='Blugrn'
                            )
                        )])
                        ax.scatter(x, y, z, marker='o', s=20, c=density)

                        for idx, k in enumerate(self.groups[cam_name][key].keys()):
                            self.groups[cam_name][key][k]['density'] = density[idx]

                        fig2 = go.Figure(data=[go.Scatter3d(x=x, y=y, z=list(density))],
                                         layout_yaxis_range=[-2,2],
                                         layout_xaxis_range=[-2,2],
                                         # layout_zaxis_range=[-2,2]
                                         )
                        fig2.update_layout(title='PDF', autosize=False,
                                          # width=1000, height=1000,
                                          # margin=dict(l=65, r=50, b=65, t=90)
                                          )
                        fig2.show()
            data_list.extend([go.Scatter3d(
                x=[0],
                y=[0],
                z=[0],
                name = "Master : " + cam_name,
                mode='markers',
                marker=dict(
                    size=8,
                    color= 'Green',
                    # colorscale='Blugrn'
                )
            )])
            data_list.extend([go.Scatter3d(x=[None],
                                        y=[None], z=[None],
                                        mode='markers',
                                        marker=dict(
                                            # colorscale=red_blue,
                                            showscale=True,
                                            cmin=0,
                                            cmax=1,
                                            colorbar=dict(thickness=10, tickvals=[0, 1],
                                                          outlinewidth=0),
                                            colorscale='Blugrn',
                                        ),
                                        hoverinfo='none'
                                        )])

            ax.scatter([0], [0], [0], marker='o', s=20, c='yellow')
            # plt.show()
            fig1 = go.Figure(data=data_list)
            fig1.update_layout(
                scene=dict(
                    xaxis=dict(tickfont = dict(size=15), nticks=5,),
                    yaxis=dict(tickfont = dict(size=15), nticks=5, ),
                    zaxis=dict(tickfont = dict(size=15), nticks=5,),
                    xaxis_title="<b>X</b>",
                    yaxis_title="<b>Y</b>",
                    zaxis_title="<b>Z</b>",
                ),
                # plot_bgcolor='white',
                template='plotly_white',
                font=dict(
                    # family="Courier New, monospace",
                    size=20,
                    # color="RebeccaPurple"
                )
                # width=700,
                # margin=dict(r=20, l=10, b=10, t=10)
            )

            fig1.update_xaxes(
                mirror=True,
                ticks='outside',
                showline=True,
                gridwidth=5,
                linecolor='black',
                gridcolor='black'
            )
            fig1.update_yaxes(
                mirror=True,
                gridwidth=5,
                ticks='outside',
                showline=True,
                linecolor='black',
                gridcolor='black'
            )
            # fig1.update_layout(yaxis = dict(tickfont = dict(size=100)), xaxis = dict(tickfont = dict(size=100)))
            fig1.show()

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
                    data1 = visualizer.extrinsic2pyramid(slave_extrinsic, color=self.camera_color[slave_cam], show_legend=False,
                                                         focal_len_scaled=0.1, aspect_ratio=0.3,
                                                         hover_template=slave_cam+ "_" + str(tvec), name=name)
                    # data2 = visualizer.extrinsic2pyramid(slave_extrinsic, color=self.groups[cam_name][key][key2]['density'],
                    #                                      focal_len_scaled=0.1, aspect_ratio=0.3,
                    #                                      hover_template=slave_cam+ "_" + str(tvec), name=name)
                    # all_fig.append(data)
                    # all_fig.append(data1)
                    final_layout.add_trace(data)
                    final_layout.add_trace(data1)


            if self.mean_cameras:
                for slaveC in self.mean_cameras[cam_name]:
                    slave_extrinsic = np.array(self.mean_cameras[cam_name][slaveC]['extrinsic'])
                    d = visualizer.extrinsic2pyramid(slave_extrinsic, color='blue', focal_len_scaled=0.2, aspect_ratio=0.3,
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
                    # if file == "campose2.json":
                    #     campose2_path = os.path.join(self.base_path, "campose2.json")
                    #     self.campose2 = json.load(open(campose2_path))
                    if file == 'meanCameras.json':
                        meanCam_path = os.path.join(self.base_path, 'meanCameras.json')
                        self.mean_cameras = json.load(open(meanCam_path))




if __name__ == '__main__':
    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35"
    v = Interactive_Extrinsic(base_path)

