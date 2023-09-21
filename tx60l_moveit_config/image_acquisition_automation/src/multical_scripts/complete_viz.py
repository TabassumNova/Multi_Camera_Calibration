import math

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
from matplotlib import pyplot as plt
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


'''
for camera extrinsic visualization
'''
class Complete_Viz():
    def __init__(self, base_path):
        self.base_path = base_path
        self.workspace = None
        self.cameras = None
        self.all_images = None
        self.error_dict = {}
        self.boards = None
        self.reprojected_points = {}
        self.inlier_mask = {}
        self.camera_intrinsics = {}
        self.camera_extrinsics = {}
        self.handEye = None
        self.campose2 = None
        self.mean_cameras = None
        self.load_files()
        self.camera_color = {}
        self.set_Cam_color()

        self.draw_cameras()
        pass

    def set_Cam_color(self):
        # colors = ['red', 'green', 'blue', 'cyan', 'magenta', 'lime', 'pink', 'teal', 'darkcyan', 'violet', 'brown', 'indigo']
        colors = px.colors.sequential.Aggrnyl
        for idx, cam in enumerate(self.workspace.names.camera):
            self.camera_color[cam] = colors[idx]


        pass
    def calculate_area(self, board_id, ids):
        adjusted_points_x = [self.workspace.boards[board_id].adjusted_points[i][0] for i in ids]
        adjusted_points_y = [self.workspace.boards[board_id].adjusted_points[i][1] for i in ids]
        x_min, x_max = min(adjusted_points_x), max(adjusted_points_x)
        y_min, y_max = min(adjusted_points_y), max(adjusted_points_y)
        area = (x_max-x_min)*(y_max-y_min)
        return area

    def draw_cameras(self):
        df = {}
        folder = self.base_path[-3:]
        excel_path = os.path.join(self.base_path, folder + '-M' + '-all' + '-finalInlier.xlsx')
        writer = pd.ExcelWriter(excel_path, engine="xlsxwriter")
        visualizer = CameraPoseVisualizer([-2000, 2000], [-2000, 2000], [-2000, 2000])
        for cam in self.camera_extrinsics.keys():
            self.error_dict[cam] = {}
            final_layout = go.Figure()
            folder = self.base_path[-3:]
            final_layout.add_annotation(dict(font=dict(color='black', size=20),
                                             x=0,
                                             y=0.12,
                                             showarrow=False,
                                             text=folder + '-' + cam,
                                             textangle=0,
                                             xanchor='left',
                                             xref="paper",
                                             yref="paper"))
            for camS in self.camera_extrinsics[cam].keys():
                pose = np.linalg.inv(self.camera_extrinsics[cam][camS])
                d = visualizer.extrinsic2pyramid(pose, color=self.camera_color[camS], focal_len_scaled=0.15, aspect_ratio=0.3,
                                                 hover_template="mean", name=camS)
                final_layout.add_trace(d)

            ## Draw boards
            self.error_dict[cam]['num_inliers_point'] = {}
            self.error_dict[cam]['num_good_inliers_point'] = {}
            self.error_dict[cam]['num_inlier_poses'] = {}
            self.error_dict[cam]['num_good_inlier_poses'] = {}
            self.error_dict[cam]['inlier_point_error'] = {}
            self.error_dict[cam]['inlier_pose_error'] = {}
            self.error_dict[cam]['inlier_poses'] = {}
            self.error_dict[cam]['good_inlier_poses'] = {}
            self.error_dict[cam]['inlier_poses_std'] = {}
            self.error_dict[cam]['inlier_poses_mean'] = {}
            self.error_dict[cam]['inlier_poses_variation'] = {}
            self.error_dict[cam]['inlier_area'] = {}
            # if good_pose:
            self.error_dict[cam]['good_inlier_poses_std'] = {}
            self.error_dict[cam]['good_inlier_poses_mean'] = {}
            self.error_dict[cam]['good_inlier_poses_variation'] = {}
            for cam_id, camS in enumerate(self.workspace.names.camera):
                # self.error_dict[cam][camS] = {}
                point_error = []
                pose_error = []
                poses = []
                good_inlier = 0
                good_pose = 0
                good_poses = []
                test_pose = []
                total_area = 0
                for img_id, img in enumerate(self.workspace.names.image):
                    for board_id, board in enumerate(self.workspace.names.board):
                        inlier_mask = self.inlier_mask[cam]
                        ids = np.flatnonzero(inlier_mask[cam_id][img_id][board_id])
                        if len(ids) and self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            ## for inliers only
                            # inlier_mask = self.inlier_mask[cam]
                            area = self.calculate_area(board_id, ids)
                            total_area += area
                            real_points = self.workspace.point_table.points[cam_id][img_id][board_id]
                            reprojected_points = self.reprojected_points[cam].points[cam_id][img_id][board_id]
                            # ids = np.flatnonzero(inlier_mask[cam_id][img_id][board_id])
                            err0 = real_points - reprojected_points
                            point_err1 = [np.linalg.norm(err0[i]) for i in ids]
                            pose_err = np.sqrt(np.square(point_err1).mean())
                            p, _ = rtvec.split(rtvec.from_matrix(self.workspace.pose_table.poses[cam_id][img_id][board_id]))
                            pose0 = np.linalg.norm([p[0], p[1]]) * 180.0 / math.pi
                            point_error.extend(point_err1)
                            pose_error.append(pose_err)
                            poses.append(pose0)
                            # if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            if len(ids)>10 and pose_err<1.0:
                                ## for good inliers
                                good_inlier += len(ids)
                                good_pose += 1
                                board_pose = (self.workspace.pose_table.poses[cam_id][img_id][board_id])
                                camS_to_camM = (self.camera_extrinsics[cam][camS])
                                pose = (camS_to_camM @ board_pose)
                                d = visualizer.extrinsic2Board(pose, color=self.camera_color[camS], focal_len_scaled=0.15,
                                                                 aspect_ratio=0.3,
                                                                 hover_template="mean", name=camS)
                                final_layout.add_trace(d)
                                good_poses.append(pose0)
                                test_pose.append(np.linalg.norm([pose[0], pose[1]]) * 180.0 / math.pi)
                            # else:
                            #     board_pose = (self.workspace.pose_table.poses[cam_id][img_id][board_id])
                            #     camS_to_camM = (self.camera_extrinsics[cam][camS])
                            #     pose = (camS_to_camM @ board_pose)
                            #     t = np.linalg.norm([pose[0], pose[1]]) * 180.0 / math.pi
                            #     if t not in test_pose:
                            #         d = visualizer.extrinsic2Board(pose, color='red',
                            #                                        focal_len_scaled=0.15,
                            #                                        aspect_ratio=0.3,
                            #                                        hover_template="mean", name=camS)
                            #         final_layout.add_trace(d)


                self.error_dict[cam]['num_inliers_point'][camS] = len(point_error)
                self.error_dict[cam]['num_good_inliers_point'][camS] = good_inlier
                self.error_dict[cam]['num_inlier_poses'][camS] = len(poses)
                self.error_dict[cam]['num_good_inlier_poses'][camS] = good_pose
                self.error_dict[cam]['inlier_point_error'][camS] = point_error
                self.error_dict[cam]['inlier_pose_error'][camS] = pose_error
                self.error_dict[cam]['inlier_poses'][camS] = poses
                self.error_dict[cam]['good_inlier_poses'][camS] = good_poses
                self.error_dict[cam]['inlier_poses_std'][camS] = np.std(poses)
                self.error_dict[cam]['inlier_poses_mean'][camS] = np.mean(poses)
                if len(poses):
                    self.error_dict[cam]['inlier_area'][camS] = total_area
                    self.error_dict[cam]['inlier_poses_variation'][camS] = max(poses) - min(poses)
                if good_pose:
                    self.error_dict[cam]['good_inlier_poses_std'][camS] = np.std(good_poses)
                    self.error_dict[cam]['good_inlier_poses_mean'][camS] = np.mean(good_poses)
                    self.error_dict[cam]['good_inlier_poses_variation'][camS] = max(good_poses) - min(good_poses)

            # final_layout.show()
            self.graph_analysis(cam)
            df[cam] = pd.DataFrame(self.error_dict[cam])
            df[cam].to_excel(writer, sheet_name=cam)
        writer.close()


    def graph_analysis(self, masterCam):
        # plt.figure(figsize=(10, 10))
        fig, axs = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        fig1, axs1 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        fig2, axs2 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        fig3, axs3 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        for idx, camS in enumerate(self.workspace.names.camera):
            point_error = self.error_dict[masterCam]['inlier_point_error'][camS]
            pose_error = self.error_dict[masterCam]['inlier_pose_error'][camS]
            poses = self.error_dict[masterCam]['inlier_poses'][camS]
            good_poses = self.error_dict[masterCam]['good_inlier_poses'][camS]
            bin = np.arange(0, 2, .1)
            bin2 = np.arange(0, 120, 5)
            if idx < math.ceil(self.workspace.sizes.camera / 2):
                axs[0, idx].hist(point_error, bin, edgecolor='black')
                axs[0, idx].set_title('Cam-' + camS)
                axs1[0, idx].hist(pose_error, bin, edgecolor='black')
                axs1[0, idx].set_title('Cam-' + camS)
                axs2[0, idx].scatter(poses, pose_error)
                axs2[0, idx].set_title('Cam-' + camS)
                if len(good_poses):
                    axs3[0, idx].hist(poses, bin2, edgecolor='black')
                    axs3[0, idx].set_title('Cam-' + camS)
            else:
                i = idx - math.ceil(self.workspace.sizes.camera / 2)
                axs[1, i].hist(point_error, bin, edgecolor='black')
                axs[1, i].set_title('Cam-' + camS)
                axs1[1, i].hist(pose_error, bin, edgecolor='black')
                axs1[1, i].set_title('Cam-' + camS)
                axs2[1, i].scatter(poses, pose_error)
                axs2[1, i].set_title('Cam-' + camS)
                if len(good_poses):
                    axs3[1, i].hist(poses, bin2, edgecolor='black')
                    axs3[1, i].set_title('Cam-' + camS)
            for ax, ax1, ax2, ax3 in zip(axs.flat, axs1.flat, axs2.flat, axs3.flat):
                ax.set_xlabel('Re-projection error \n Per point', fontsize=15)
                ax.set_ylabel('Number of Points', fontsize=15)
                # ax.set(xlabel='Re-projection error \n Per point', ylabel='Number of Points')
                ax1.set_xlabel('Re-projection error \n Per view', fontsize=15)
                ax1.set_ylabel('Number of Views', fontsize=15)
                # ax1.set(xlabel='Re-projection error \n Per view', ylabel='Number of Views')
                ax2.set(xlabel='Views', ylabel='Re-projection Error')
                ax3.set(xlabel='View angle (Degrees)', ylabel='Number of Views')

            for ax, ax1, ax2, ax3 in zip(axs.flat, axs1.flat, axs2.flat, axs3.flat):
                ax.label_outer()
                ax1.label_outer()
                ax2.label_outer()
                ax3.label_outer()
        # ax.set_title('Histogram of Inlier Points Re-projection Error: Master-Camera{}'.format(masterCam),
        #              # fontproperties=prop,
        #              y=-0.2, fontsize=14)
        folder = self.base_path[-3:]
        path = os.path.join(self.base_path, folder + '-finalPointError.png')

        # plt.savefig(path)
        # plt.figure(figsize=(10, 10))
        plt.show()

        # df = pd.DataFrame(self.error_dict[masterCam])
        # excel_path = os.path.join(self.base_path, folder + '-M' + masterCam+'-finalInlier.xlsx')
        # df.to_excel(excel_path, sheet_name=masterCam)
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


    def collect_inlier_dataset(self):
        for cam in self.cameras:
            file = os.path.join(self.base_path, 'calibration_'+ cam + '.pkl')
            if os.path.exists(file):
                data = pickle.load(open(file, "rb"))
                self.inlier_mask[cam] = data.calibrations['calibration'].inlier_mask
                self.reprojected_points[cam] = data.calibrations['calibration'].reprojected
        pass

    def load_files(self):
        workspace, handEye, campose2 = None, None, None
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace_test.pkl"][0])
                self.workspace = pickle.load(open(workspace_path, "rb"))
                self.cameras = self.workspace.names.camera
                self.all_images = self.workspace.names.image
                self.boards = self.workspace.names.board
                self.collect_inlier_dataset()
                for name in files:
                    if "calibration_" in name:
                        n = name.split('calibration_')[0]
                        if n == '' and name.endswith('.json'):
                            cam1 = self.workspace.names.camera[0]
                            master_cam = name.split('calibration_')[1].split('.')[0]
                            if master_cam in self.workspace.names.camera:
                                # self.camera_intrinsics[master_cam] = {}
                                self.camera_extrinsics[master_cam] = {}
                                data = json.load(open(os.path.join(self.base_path, name)))
                                for cam_id, cam in enumerate(self.workspace.names.camera):
                                    if cam == cam1:
                                        self.camera_extrinsics[master_cam][cam] = np.eye(4)
                                    else:
                                        name = cam + '_to_' + cam1
                                        if name in data['camera_poses']:
                                            R, T = np.array(data['camera_poses'][name]['R']), np.array(data['camera_poses'][name]['T'])
                                            self.camera_extrinsics[master_cam][cam] = matrix.join(R,T)
                                        else:
                                            self.camera_extrinsics[master_cam][cam] = np.eye(4)




if __name__ == '__main__':
    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V30"
    v = Complete_Viz(base_path)

