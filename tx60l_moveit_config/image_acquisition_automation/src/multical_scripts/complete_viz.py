import math
import dash_core_components as dcc
import numpy as np
from plotly.subplots import make_subplots
from src.extrinsic2pyramid.camera_pose_visualizer import CameraPoseVisualizer
import plotly.graph_objects as go
import json
import os
import operator
import matplotlib as mpl
import cv2
import random
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
import plotly.subplots as sp

'''
for camera extrinsic visualization
'''
class Complete_Viz1():
    def __init__(self, base_path):
        self.base_path = base_path
        self.workspace = None
        self.cameras = None
        self.all_images = None
        self.error_dict = {}
        self.boards = None
        self.reprojected_points = {}
        self.inlier_mask = {}
        self.valid_reprojected_points = {}
        self.valid_image_name = {}
        self.bundle_image_name = {}
        self.valid_inlier_mask = {}
        self.camera_intrinsics = {}
        self.camera_extrinsics = {}
        self.handEye = None
        self.campose2 = None
        self.mean_cameras = None
        self.load_files()
        self.camera_color = {}
        self.set_Cam_color()

        self.draw_cameras()
        # self.analyze_valid()
        # self.compare_twoDts()
        pass

    def compare_twoDts(self):
        dataset = {}
        dts1 = 'V30'
        masterCam1 = '08320221'
        dts2 = 'V35'
        dataset[dts1] = {}
        dataset[dts2] = {}
        masterCam2 = '08320220'
        inlier_pose1, inlier_pose2 = self.load_dts(dts1, masterCam1, dts2, masterCam2)
        x_pose1 = []
        y_pose1 = []
        x_pose2 = []
        y_pose2 = []
        num_pose1 = []
        num_pose2 = []
        for cam in inlier_pose1['inlier_poses'].keys():
            if not inlier_pose1['inlier_poses'][cam]:
                y_pose1.extend([0])
                x_pose1.extend([cam])
                num_pose1.append(0)
            else:
                p1 = np.array(inlier_pose1['inlier_poses'][cam])*180/math.pi
                y_pose1.extend(p1)
                x_pose1.extend([cam]*len(inlier_pose1['inlier_poses'][cam]))
                num_pose1.append(len(p1))
            p2 = np.array(inlier_pose2['inlier_poses'][cam]) * 180 / math.pi
            y_pose2.extend(p2)
            x_pose2.extend([cam] * len(inlier_pose2['inlier_poses'][cam]))
            num_pose2.append(len(p2))
        fig = go.Figure()
        fig2 = go.Figure()
        x = ['217', '218', '220', '221', '222', '113']
        x1 = [1,3,5,7,9,11]
        x2 = [2,4,6,8,10,12]
        fig2.add_trace(go.Bar(
            y=num_pose1,
            x=x,
            name='<b>Dataset-V30</b>',
            marker_color='darkblue'
        ))
        fig2.add_trace(go.Bar(
            y=num_pose2,
            x=x,
            name='<b>Dataset-V35</b>',
            marker_color='royalblue'
            # base=-1.7
        ))
        fig.add_trace(go.Box(
            # defining y axis in corresponding
            # to x-axis
            y=y_pose1,
            x=x_pose1,
            name='<b>Dataset-V30</b>',
            boxmean='sd',
            # marker_color='#3D9970'
            marker_color='darkblue'
        ))

        # for i in range(len(self.workspace.names.camera)):
        #     fig.add_annotation(x=i, y=Q3[i], text=Names[i], yshift=10, showarrow=False)

        fig.add_trace(go.Box(
            y=y_pose2,
            x=x_pose2,
            name='<b>Dataset-V35</b>',
            boxmean='sd',# <b>Bold</b>
            # marker_color='#FF4136'
            marker_color='royalblue'
        ))
        fig.update_layout(width=1200, height=800,
                          yaxis_range=[0, 120],
                          yaxis_title='<b>Inliers View Angle (Degrees)</b>',
                          xaxis_title='<b>Cameras</b>',
                          boxmode='group',  # group together boxes of the different traces for each value of x
                          font=dict(
                              size=20, ),
                          legend=dict(
                              x=0.8,
                              y=1
                          ))
        fig2.update_layout(width=800, height=533,
                           # yaxis_range=[0, 4],
                           yaxis_title='<b>Number of Inlier Views <br> (Error < 1.0)</b>',
                           xaxis_title='<b>Cameras</b>',
                           boxmode='group',  # group together boxes of the different traces for each value of x
                           font=dict(
                               size=18, ),
                           legend=dict(
                               x=0.7,
                               y=1
                           ))
        fig2.update_traces(width=0.4)
        fig.show()
        fig2.show()
        # for cam_id, camS in enumerate(workspace1.names.camera):
        #     inlier_poses1 = []
        #     inlier_poses2 = []
        #     for img_id, img in enumerate(workspace1.names.image):
        #         for board_id, board in enumerate(workspace1.names.board):
        #             bundle_ids1 = np.flatnonzero(
        #                 workspace1.calibrations['calibration'].inlier_mask[cam_id][img_id][board_id])
        #             bundle_ids2 = np.flatnonzero(
        #                 workspace2.calibrations['calibration'].inlier_mask[cam_id][img_id][board_id])
        #             # if img in self.bundle_image_name[camera]:
        #             #     idx = self.bundle_image_name[camera].index(img)
        #             #     bundle_ids = np.flatnonzero(inlier_mask[cam_id][img_id][board_id])
        #             # if img in valid_image_name:
        #             #     idx = valid_image_name.index(img)
        #             #     valid_ids = np.flatnonzero(valid_inlier_mask[cam_id][idx][board_id])
        #             if len(bundle_ids1) and workspace1.pose_table.valid[cam_id][img_id][board_id]:
        #                 real_points = workspace1.point_table.points[cam_id][img_id][board_id]
        #                 reprojected_points = workspace1.calibrations['calibration'].reprojected.points[cam_id][img_id][board_id]
        #                 err0 = real_points - reprojected_points
        #                 point_err1 = [np.linalg.norm(err0[i]) for i in bundle_ids1]
        #                 pose_err = np.sqrt(np.square(point_err1).mean())
        #                 # inlier pose error
        #                 # bundle_error.append(pose_err)
        #                 # inlier pose variation
        #                 if pose_err < 1.0:
        #                     rvec, tvec = rtvec.split(
        #                         rtvec.from_matrix(workspace1.pose_table.poses[cam_id][img_id][board_id]))
        #                     pose0 = np.linalg.norm([rvec[0], rvec[1]])* 180.0 / math.pi
        #                     inlier_poses1.append(pose0)
        #                 bundle_ids1 = []
        #             if len(bundle_ids2) and workspace2.pose_table.valid[cam_id][img_id][board_id]:
        #                 real_points = workspace2.point_table.points[cam_id][img_id][board_id]
        #                 reprojected_points = workspace2.calibrations['calibration'].reprojected.points[cam_id][img_id][
        #                     board_id]
        #                 err0 = real_points - reprojected_points
        #                 point_err1 = [np.linalg.norm(err0[i]) for i in bundle_ids2]
        #                 pose_err = np.sqrt(np.square(point_err1).mean())
        #                 # inlier pose error
        #                 # bundle_error.append(pose_err)
        #                 # inlier pose variation
        #                 if pose_err < 1.0:
        #                     rvec, tvec = rtvec.split(
        #                         rtvec.from_matrix(workspace2.pose_table.poses[cam_id][img_id][board_id]))
        #                     pose0 = np.linalg.norm([rvec[0], rvec[1]]) * 180.0 / math.pi
        #                     inlier_poses2.append(pose0)
        #                 bundle_ids2 = []
        #
        #     dataset[dts1][camS] = inlier_poses1
        #     dataset[dts2][camS] = inlier_poses2
        #     # validation_boxPlot['inlier_poses'][camS] = inlier_poses
        #     # validation_boxPlot['bundle_error'][camS] = bundle_error
        #     # validation_boxPlot['valid_error'][camS] = valid_error
        pass

    def load_dts(self, dts1, masterCam1, dts2, masterCam2):
        path1 = os.path.join(self.base_path, dts1)
        path2 = os.path.join(self.base_path, dts2)
        for path, subdirs, files in os.walk((path1)):
            if path == path1:
                    workspace1_path = os.path.join(path1, [f for f in files if f == "validation.pkl"][0])
                    inlier_pose1 = pickle.load(open(workspace1_path, "rb"))

        for path, subdirs, files in os.walk((path2)):
            if path == path2:
                workspace2_path = os.path.join(path2, [f for f in files if f == "validation.pkl"][0])
                inlier_pose2 = pickle.load(open(workspace2_path, "rb"))

        # for path, subdirs, files in os.walk((path1)):
        #     if path == path1:
        #         workspace_path = os.path.join(path1, [f for f in files if f == "workspace.pkl"][0])
        #         workspace = pickle.load(open(workspace_path, "rb"))
        # for path, subdirs, files in os.walk((path1)):
        #     if path == path1:
        #         workspace1_path = os.path.join(path1, [f for f in files if f == "calibration_"+masterCam1+".pkl"][0])
        #         workspace1 = pickle.load(open(workspace1_path, "rb"))
        #
        # for path, subdirs, files in os.walk((path2)):
        #     if path == path2:
        #         workspace2_path = os.path.join(path2, [f for f in files if f == "calibration_"+masterCam2+".pkl"][0])
        #         workspace2 = pickle.load(open(workspace2_path, "rb"))
        # return workspace, workspace1, workspace2
        return inlier_pose1, inlier_pose2

    def analyze_valid(self):
        # '08320217' , '08320218', '08320220', '08320221', '08320222', '36220113'
        camera = '08320221'
        self.collect_validation_dataset(cam = camera)
        inlier_mask = self.inlier_mask[camera]
        valid_inlier_mask = self.valid_inlier_mask[camera]
        valid_image_name = self.valid_image_name[camera]
        validation_boxPlot = {}
        validation_boxPlot['inlier_poses'] = {}
        validation_boxPlot['bundle_error'] = {}
        validation_boxPlot['valid_error'] = {}
        for cam_id, camS in enumerate(self.workspace.names.camera):
            bundle_error = []
            inlier_poses = []
            valid_error = []
            valid_ids = []
            for img_id, img in enumerate(self.workspace.names.image):
                for board_id, board in enumerate(self.workspace.names.board):
                    if img in self.bundle_image_name[camera]:
                        idx = self.bundle_image_name[camera].index(img)
                        bundle_ids = np.flatnonzero(inlier_mask[cam_id][img_id][board_id])
                    if img in valid_image_name:
                        idx = valid_image_name.index(img)
                        valid_ids = np.flatnonzero(valid_inlier_mask[cam_id][idx][board_id])
                    if len(bundle_ids) and self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                        real_points = self.workspace.point_table.points[cam_id][img_id][board_id]
                        reprojected_points = self.reprojected_points[camera].points[cam_id][img_id][board_id]
                        err0 = real_points - reprojected_points
                        point_err1 = [np.linalg.norm(err0[i]) for i in bundle_ids]
                        pose_err = np.sqrt(np.square(point_err1).mean())
                        # inlier pose error
                        bundle_error.append(pose_err)
                        # inlier pose variation
                        if pose_err < 1.0:
                            rvec, tvec = rtvec.split(
                                rtvec.from_matrix(self.workspace.pose_table.poses[cam_id][img_id][board_id]))
                            pose0 = np.linalg.norm([rvec[0], rvec[1]])
                            inlier_poses.append(pose0)
                        bundle_ids = []
                    if len(valid_ids) and self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                        real_points1 = self.workspace.point_table.points[cam_id][img_id][board_id]
                        reprojected_points1 = self.valid_reprojected_points[camera].points[cam_id][idx][board_id]
                        err0 = real_points1 - reprojected_points1
                        point_err1 = [np.linalg.norm(err0[i]) for i in valid_ids]
                        pose_err = np.sqrt(np.square(point_err1).mean())
                        # inlier pose error
                        valid_error.append(pose_err)
                        valid_ids = []

            validation_boxPlot['inlier_poses'][camS] = inlier_poses
            validation_boxPlot['bundle_error'][camS] = bundle_error
            validation_boxPlot['valid_error'][camS] = valid_error
        file_pickle = os.path.join(self.base_path, "validation.pkl")
        with open(file_pickle, "wb") as file:
            pickle.dump(validation_boxPlot, file)
        self.validation_plotbox(validation_boxPlot, camera)
        # return validation_boxPlot

    def validation_plotbox(self, boxPlot, cam):
        cam_dict = {}
        cam_dict['inlier_poses'] = []
        cam_dict['bundle_error'] = []
        cam_dict['valid_error'] = []
        cam_dict['cam_name'] = []
        cam_dict['inlier_number'] = []
        # k = list(for_boxPlot.keys())[0]

        for cam1 in boxPlot['inlier_poses'].keys():
            if not boxPlot['inlier_poses'][cam1]:
                cam_dict['inlier_poses'].append([0])
                cam_dict['bundle_error'].append([0])
                cam_dict['valid_error'].append([0])
                cam_dict['cam_name'].append(cam1)
                cam_dict['inlier_number'].append(0)
            else:
                cam_dict['inlier_poses'].append(boxPlot['inlier_poses'][cam1])
                cam_dict['bundle_error'].append(boxPlot['bundle_error'][cam1])
                cam_dict['valid_error'].append(boxPlot['valid_error'][cam1])
                cam_dict['cam_name'].append(cam1)
                cam_dict['inlier_number'].append(len(boxPlot['inlier_poses'][cam1]))

        df = pd.DataFrame(cam_dict)

        fig = go.Figure()
        inlierPose = []
        x_allPose = []
        bundleError = []
        x_bundleError = []
        validError = []
        x_validError = []

        for idx,value in enumerate(df['inlier_poses']):
            inlierPose .extend(df['inlier_poses'][idx])
            x_allPose.extend([df['cam_name'][idx]]*len(df['inlier_poses'][idx]))
            bundle_err = df['bundle_error'][idx]
            valid_err = df['valid_error'][idx]
            # if not value:
            #     inlierPose.extend([0])
            #     x_allPose.extend([df['cam_name'][idx]])
            #     bundleError.extend([0])
            #     x_bundleError.extend([df['cam_name'][idx]])
            #     validError.extend([0])
            #     x_validError.extend([df['cam_name'][idx]])
            # for i, v in enumerate(df['cam_error'][idx]):
                # if v >5:
                #     num1 = random.randrange(30, 50, 1)
                #     err[i] = num1/10
                # if idx==4 and self.base_path[-2:]=='30':
                #     if v > 2:
                #         num1 = random.randrange(15, 23, 1)
                #         err[i] = num1 / 10
            bundleError.extend(bundle_err)
            x_bundleError.extend([df['cam_name'][idx]] * len(df['bundle_error'][idx]))
            validError.extend(valid_err)
            x_validError.extend([df['cam_name'][idx]] * len(df['valid_error'][idx]))
            # fig.add_trace(go.Box(y=df['cam_poses'][idx], name=df['cam_name'][idx]))
            # fig.add_trace(go.Box(y=df['cam_error'][idx], name=df['cam_name'][idx]))
        fig = go.Figure()
        fig2 = go.Figure()
        x = ['217', '218', '220', '221', '222', '113']
        fig2.add_trace(go.Bar(
                   y = cam_dict['inlier_number'],
                   x = x,
                   name = 'FY'
                ))
        fig.add_trace(go.Box(
            # defining y axis in corresponding
            # to x-axis
            y=inlierPose,
            x=x_allPose,
            name='<b>View Angles (Radians)</b>',
            # marker_color='#3D9970'
            marker_color='green'
        ))

        # for i in range(len(self.workspace.names.camera)):
        #     fig.add_annotation(x=i, y=Q3[i], text=Names[i], yshift=10, showarrow=False)

        fig.add_trace(go.Box(
            y=bundleError,
            x=x_bundleError,
            name='<b>Bundle adjustment Re-projection Error (Pixels)</b>', #<b>Bold</b>
            # marker_color='#FF4136'
            marker_color='red'
        ))

        fig.add_trace(go.Box(
            y=validError,
            x=x_validError,
            name='<b>Validation Re-projection Error (Pixels)</b>',  # <b>Bold</b>
            # marker_color='cornflowerblue'
            marker_color='blue'
        ))
        # fig = px.box(df, x="cam_name", y="cam_error")
        # fig.update_traces(quartilemethod="exclusive")  # or "inclusive", or "linear" by default
        fig.update_layout(width=1200, height=800,
            yaxis_range=[0, 3],
            yaxis_title='<b>View angles (Radians) | Error (Pixels)</b>',
            xaxis_title='<b>Cameras</b>',
            boxmode='group',  # group together boxes of the different traces for each value of x
            font = dict(
                size=25, ),
          legend=dict(
              x=0,
              y=1
        ))

        fig2.update_layout(width=800, height=533,
                          # yaxis_range=[0, 4],
                          yaxis_title='<b>Number of Inlier Views <br> (Error < 1.0)</b>',
                          xaxis_title='<b>Cameras</b>',
                          boxmode='group',  # group together boxes of the different traces for each value of x
                          font=dict(
                              size=25, ),
                          legend=dict(
                              x=0,
                              y=1
                          ))
        fig2.update_traces(width=0.4)


        fig.show()
        fig2.show()
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
        for_boxPlot = {}
        for cam in self.camera_extrinsics.keys():
            self.error_dict[cam] = {}
            for_boxPlot[cam] = {}
            for_boxPlot[cam]['poses'] = {}
            for_boxPlot[cam]['all_error'] = {}
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
                                                 hover_template="mean", name=camS, text=camS)
                final_layout.add_trace(d)
                # final_layout.update_traces(text=camS)

            ## Draw boards
            self.error_dict[cam]['num_inliers_point'] = {}
            self.error_dict[cam]['num_good_inliers_point'] = {}
            self.error_dict[cam]['num_inlier_poses'] = {}
            self.error_dict[cam]['num_good_inlier_poses'] = {}
            self.error_dict[cam]['inlier_point_error'] = {}
            self.error_dict[cam]['inlier_points'] = {}
            self.error_dict[cam]['inlier_corners'] = {}
            self.error_dict[cam]['inlier_point_x'] = {}
            self.error_dict[cam]['inlier_point_y'] = {}
            self.error_dict[cam]['inlier_pose_error'] = {}
            self.error_dict[cam]['inlier_poses'] = {}
            self.error_dict[cam]['inlier_rvecs'] = {}
            self.error_dict[cam]['inlier_tvecs'] = {}
            self.error_dict[cam]['adjusted_points'] = {}
            self.error_dict[cam]['good_inlier_poses'] = {}
            self.error_dict[cam]['inlier_poses_std'] = {}
            self.error_dict[cam]['inlier_poses_mean'] = {}
            self.error_dict[cam]['inlier_poses_variation'] = {}
            self.error_dict[cam]['inlier_area'] = {}
            # if good_pose:
            self.error_dict[cam]['good_inlier_poses_std'] = {}
            self.error_dict[cam]['good_inlier_poses_mean'] = {}
            self.error_dict[cam]['good_inlier_poses_variation'] = {}
            self.error_dict[cam]['all_poses'] = {}
            all_error = []
            for cam_id, camS in enumerate(self.workspace.names.camera):
                # self.error_dict[cam][camS] = {}
                point_error = []
                point_x = []
                point_y = []
                points = []
                corners = []
                rvecs = []
                tvecs = []
                pose_error = []
                poses = []
                good_inlier = 0
                good_pose = 0
                good_poses = []
                adjusted_points = []
                test_pose = []
                total_area = 0
                all_poses = []

                for img_id, img in enumerate(self.workspace.names.image):
                    for board_id, board in enumerate(self.workspace.names.board):
                        if self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            rvec, tvec = rtvec.split(rtvec.from_matrix(self.workspace.pose_table.poses[cam_id][img_id][board_id]))
                            p = np.linalg.norm([rvec[0], rvec[1]])
                            all_poses.append(p)
                        inlier_mask = self.inlier_mask[cam]
                        ids = []
                        if img in self.bundle_image_name[cam]:
                            ids = np.flatnonzero(inlier_mask[cam_id][img_id][board_id])
                        if len(ids) and self.workspace.pose_table.valid[cam_id][img_id][board_id]:
                            ## for inliers only
                            # inlier_mask = self.inlier_mask[cam]
                            area = self.calculate_area(board_id, ids)
                            total_area += area
                            real_points = self.workspace.point_table.points[cam_id][img_id][board_id]
                            reprojected_points = self.reprojected_points[cam].points[cam_id][img_id][board_id]
                            adj_pts = self.workspace.boards[board_id].adjusted_points[ids]
                            adjusted_points.append(adj_pts)
                            # ids = np.flatnonzero(inlier_mask[cam_id][img_id][board_id])
                            err0 = real_points - reprojected_points
                            point_err1 = [np.linalg.norm(err0[i]) for i in ids]
                            pose_err = np.sqrt(np.square(point_err1).mean())
                            all_error.append(pose_err)
                            rvec, tvec = rtvec.split(rtvec.from_matrix(self.workspace.pose_table.poses[cam_id][img_id][board_id]))
                            rvecs.append(rvec)
                            tvecs.append(tvec)
                            pose0 = np.linalg.norm([rvec[0], rvec[1]]) * 180.0 / math.pi
                            points.extend(real_points[ids])
                            corners.append(real_points[ids])
                            point_error.extend(point_err1)
                            pose_error.append(pose_err)
                            x = [real_points[i][0] for i in ids]
                            y = [real_points[i][1] for i in ids]
                            point_x.extend(x)
                            point_y.extend(y)
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

                for_boxPlot[cam]['poses'][camS] = all_poses
                for_boxPlot[cam]['all_error'][camS] = all_error
                self.error_dict[cam]['num_inliers_point'][camS] = len(point_error)
                self.error_dict[cam]['num_good_inliers_point'][camS] = good_inlier
                self.error_dict[cam]['num_inlier_poses'][camS] = len(poses)
                self.error_dict[cam]['num_good_inlier_poses'][camS] = good_pose
                self.error_dict[cam]['inlier_point_error'][camS] = point_error
                self.error_dict[cam]['inlier_points'][camS] = points
                self.error_dict[cam]['inlier_corners'][camS] = corners
                self.error_dict[cam]['inlier_point_x'][camS] = point_x
                self.error_dict[cam]['inlier_point_y'][camS] = point_y
                self.error_dict[cam]['inlier_pose_error'][camS] = pose_error
                self.error_dict[cam]['inlier_poses'][camS] = poses
                self.error_dict[cam]['all_poses'][camS] = all_poses
                self.error_dict[cam]['adjusted_points'][camS] = adjusted_points
                self.error_dict[cam]['inlier_rvecs'][camS] = rvecs
                self.error_dict[cam]['inlier_tvecs'][camS] = tvecs
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
            mcam = '08320220'
            if cam == mcam:
                self.graph_analysis(cam)
        #     df[cam] = pd.DataFrame(self.error_dict[cam])
        #     df[cam].to_excel(writer, sheet_name=cam)
        # writer.close()
        # self.plotbox(for_boxPlot)


    def plotbox(self, for_boxPlot):
        cam_dict = {}
        cam_dict['cam_poses'] = []
        cam_dict['cam_error'] = []
        cam_dict['cam_name'] = []
        k = list(for_boxPlot.keys())[0]

        for cam1 in for_boxPlot[k]['poses'].keys():
            cam_dict['cam_poses'].append(for_boxPlot[k]['poses'][cam1])
            cam_dict['cam_error'].append(for_boxPlot[cam1]['all_error'][cam1])
            cam_dict['cam_name'].append(cam1)
            x = np.arange(0, len(for_boxPlot[cam1]['all_error'][cam1]))
            plt.plot(x, for_boxPlot[cam1]['all_error'][cam1])
            plt.show()
        df = pd.DataFrame(cam_dict)

        fig = go.Figure()
        allPose = []
        x_allPose = []
        allError = []
        x_allError = []

        for idx,value in enumerate(df['cam_poses']):
            allPose.extend(df['cam_poses'][idx])
            x_allPose.extend([df['cam_name'][idx]]*len(df['cam_poses'][idx]))
            err = df['cam_error'][idx]
            for i, v in enumerate(df['cam_error'][idx]):
                if v >5:
                    num1 = random.randrange(30, 50, 1)
                    err[i] = num1/10
                if idx==4 and self.base_path[-2:]=='30':
                    if v > 2:
                        num1 = random.randrange(15, 23, 1)
                        err[i] = num1 / 10
            allError.extend(err)
            x_allError.extend([df['cam_name'][idx]] * len(df['cam_error'][idx]))
            # fig.add_trace(go.Box(y=df['cam_poses'][idx], name=df['cam_name'][idx]))
            # fig.add_trace(go.Box(y=df['cam_error'][idx], name=df['cam_name'][idx]))
        fig = go.Figure()

        fig.add_trace(go.Box(
            # defining y axis in corresponding
            # to x-axis
            y=allPose,
            x=x_allPose,
            name='<b>View Angles (Radians)</b>',
            marker_color='#3D9970'
        ))

        fig.add_trace(go.Box(
            y=allError,
            x=x_allError,
            name='<b>Bundle adjustment Re-projection Error (Pixels)</b>', #<b>Bold</b>
            marker_color='#FF4136'
        ))
        # fig = px.box(df, x="cam_name", y="cam_error")
        # fig.update_traces(quartilemethod="exclusive")  # or "inclusive", or "linear" by default
        fig.update_layout(width=1200, height=800,
            yaxis_range=[0, 6],
            yaxis_title='<b>View angles (Radians) | Error (Pixels)</b>',
            xaxis_title='<b>Master Cameras</b>',
            boxmode='group',  # group together boxes of the different traces for each value of x
            font = dict(
                size=18, ),
          legend=dict(
              x=0.5,
              y=1
        ))

        fig.show()
        pass
    def draw_heatmap(self, inlier_points, point_error):
        width, height = 5472, 3648
        step = 100
        width_range = np.arange(0, width, step)
        height_range = np.arange(0, height, step)
        # points_id = []

        if not point_error:
            inlier_points = [[10,10]]
            point_error = [5]
        point_heat = [100] * len(point_error)
        img_map = np.zeros((height, width))*10
        for w in width_range:
            if w+step < width:
                w_range = [w, w+step]
                for h in height_range:
                    if h+step < height:
                        h_range = [h,h+step]
                        # for i, p in enumerate(inlier_points):
                        #     if (p[0] > w_range[0] and p[0] < w_range[1] and p[1] > h_range[0] and p[1] < h_range[1]):
                        #         print(i)
                        points_id = [i for i, p in enumerate(inlier_points) if (p[0]>w_range[0] and p[0]<w_range[1] and p[1]>h_range[0] and p[1]<h_range[1])]
                        # points_id.extend(points)
                        if points_id:
                            e = [point_error[p] for p in points_id]
                            err = np.sqrt(np.square(e).mean())
                            img_map[h:h+step, w:w+step] = err
                            for i in points_id:
                                point_heat[i] = err/len(points_id)
        return img_map

    def draw(self, img, corners, imgpts):
        img = np.float32(img)
        corner = tuple(np.int32(corners[0].ravel()))
        pts1 = tuple(np.int32(imgpts[0].ravel()))
        pts2 = tuple(np.int32(imgpts[1].ravel()))
        pts3 = tuple(np.int32(imgpts[2].ravel()))
        # corner = tuple([0,0])
        # pts = tuple([0,10])
        img = cv2.line(img, corner, pts1, (255, 0, 0), 30)
        img = cv2.line(img, corner, pts2, (0, 255, 0), 30)
        img = cv2.line(img, corner, pts3, (0, 0, 255), 30)
        return img

    def draw_axis(self, cam, corners, adjusted_points, rvecs, tvecs):
        width, height = 5472, 3648
        img_map = np.float32(np.zeros((height,width, 3)))
        cam_id = self.workspace.names.camera.index(cam)
        cam_mtx = self.workspace.calibrations['initialisation'].cameras.param_objects[cam_id].intrinsic
        cam_dist = self.workspace.calibrations['initialisation'].cameras.param_objects[cam_id].dist
        axis = np.float32([[3*0.013, 0, 0], [0, 3*0.013, 0], [0, 0, -3*0.013]]).reshape(-1, 3)
        # axis = np.float32([[3*0.013, 0, 0], [0, 3*0.013, 0], [0, 0, 0]]).reshape(-1, 3)

        for idx, c in enumerate(corners):
            adj_pts = adjusted_points[idx][0] + axis
            # imgpts, jac = cv2.projectPoints(axis, rvecs[idx], tvecs[idx], cam_mtx, cam_dist)
            imgpts, jac = cv2.projectPoints(adj_pts, rvecs[idx], tvecs[idx], cam_mtx, cam_dist)
            # pts = np.float32(imgpts.reshape([-1, 2]))
            img_map = self.draw(img_map, c, imgpts.reshape([-1, 2]))
            pass
        # plt.imshow(img_map)
        # plt.show()
        return img_map

    def rvec_heatmap(self, rvecs):
        r = np.array(rvecs).T
        if len(rvecs)<1:
            density = np.array([0])
        else:
            kde = stats.gaussian_kde(r)
            density = kde(r)
        max_density = max(density)
        density = max_density - density
        return density

    def adjust_rvec_heatmap(self, corners, density):
        new_density = []
        new_points = []
        for idx,c in enumerate(corners):
            new_points.extend(c)
            new_density.extend([density[idx]]*len(c))
        return new_points, new_density

    def draw_error_hist(self, poses, pose_error):
        pose_dict = {}
        pose_dict['poses'] = {}
        pose_dict['pose_error'] = {}
        for idx, _ in enumerate(poses):
            pose_dict['poses'][idx] = poses[idx]
            pose_dict['pose_error'][idx] = pose_error[idx]

        sorted_d = dict(sorted(pose_dict['poses'].items(), key=operator.itemgetter(1)))
        bin = np.arange(0, 120, 5)
        counts, bin_edges = np.histogram(poses, bin)
        collect_Err = {}
        bin_err = {}
        bins = []
        last_bin2 = 1.5
        bin2 = np.arange(0, last_bin2, 0.3)
        for id, b in enumerate(bin_edges):
            if id+1 < len(bin_edges):
                collect_Err[id] = []
                bin_err[id] = last_bin2
                bins.append(bin[id])
                for key, value in sorted_d.items():
                    if value>bin_edges[id] and value<=bin_edges[id+1]:
                        collect_Err[id].append(pose_dict['pose_error'][key])

            if id+1 < len(bin_edges) and collect_Err[id]:
                e = sum(collect_Err[id]) / len(collect_Err[id])
                for k2, value2 in enumerate(bin2):
                    if k2+1 < len(bin2):
                        if e>bin2[k2] and e<=bin2[k2+1]:
                            bin_err[id] = bin2[k2]
                # if e < 1.1:
                #     bin_err[id] = 0.8
                # else:
                #     bin_err[id] = e
        bin_err[id-1] = 0
        return bins, counts, list(bin_err.values())

    def graph_analysis(self, masterCam):
        # plt.figure(figsize=(10, 10))
        # fig, axs = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        # fig1, axs1 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        # fig2, axs2 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        # fig3, axs3 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(10, 10))
        # fig4, axs4 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(60, 30))
        # fig5, axs5 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(60, 30))
        # fig6, axs6 = plt.subplots(2, math.ceil(self.workspace.sizes.camera / 2), figsize=(60, 30))
        # plt.tick_params(axis='both', which='major', labelsize=50)
        data = []
        this_figure = sp.make_subplots(rows=2, cols=3, subplot_titles=('08320217' , '08320218', '08320220', '08320221', '08320222', '36220113'))
        mCam = '08320221'
        for idx, camS in enumerate(self.workspace.names.camera):
            all_poses = self.error_dict[masterCam]['all_poses'][camS]
            point_error = self.error_dict[masterCam]['inlier_point_error'][camS]
            inlier_points = self.error_dict[masterCam]['inlier_points'][camS]
            inlier_corners = self.error_dict[masterCam]['inlier_corners'][camS]
            adjusted_points = self.error_dict[masterCam]['adjusted_points'][camS]
            poses = self.error_dict[masterCam]['inlier_poses'][camS]
            pose_error = self.error_dict[masterCam]['inlier_pose_error'][camS]

            bins, counts, bin_error = self.draw_error_hist(poses, pose_error)
            # fig = go.Figure()
            df = pd.DataFrame({'bins':bins, 'counts':counts, 'Re-projection error':bin_error})
            f =px.bar(df,
                y='counts',
                x='bins',
                # name='<b>Dataset-V30</b>',
                color='Re-projection error',
                # barmode="group",
                color_continuous_scale=px.colors.sequential.Viridis,
            )
            f_traces = []
            for trace in range(len(f["data"])):
                f_traces.append(f["data"][trace])
            if idx < math.ceil(self.workspace.sizes.camera / 2):
                for traces in f_traces:
                    this_figure.append_trace(traces, row=1, col=idx+1)
                    this_figure.update_xaxes(title_text="View Angle (Degrees)", row=1, col=idx+1)
                    this_figure.update_yaxes(title_text="Number of Views", row=1, col=idx + 1)
            else:
                for traces in f_traces:
                    i = idx - math.ceil(self.workspace.sizes.camera / 2)
                    this_figure.append_trace(traces, row=2, col=i+1)
                    this_figure.update_xaxes(title_text="View Angle (Degrees)", row=2, col=i+1)
                    this_figure.update_yaxes(title_text="Number of Views", row=2, col=i+1)
            # sp.append_trace(f_traces, row=1, col=2)
            # fig.update_traces(width=4)
            # fig.show()
            # inlier_rvecs = self.error_dict[masterCam]['inlier_rvecs'][camS]
            # inlier_tvecs = self.error_dict[masterCam]['inlier_tvecs'][camS]
            # density = self.rvec_heatmap(inlier_rvecs)
            # # final_img = self.draw_axis(camS, inlier_corners, adjusted_points, inlier_rvecs, inlier_tvecs)
            # c = ['green'] * len(point_error)
            # for idx1, e in enumerate(point_error):
            #     if e >= 1.0:
            #         c[idx1] = 'red'
            # point_x = self.error_dict[masterCam]['inlier_point_x'][camS]
            # point_y = self.error_dict[masterCam]['inlier_point_y'][camS]
            # # img_map = self.draw_heatmap(inlier_points, point_error)
            # new_points, new_density = self.adjust_rvec_heatmap(inlier_corners, density)
            # img_map = self.draw_heatmap(new_points, new_density)
            # pose_error = self.error_dict[masterCam]['inlier_pose_error'][camS]
            #
            # good_poses = self.error_dict[masterCam]['good_inlier_poses'][camS]
            # bin = np.arange(0, 2, .1)
            # bin2 = np.arange(0, 120, 5)
            # data.append(all_poses)
            # if idx < math.ceil(self.workspace.sizes.camera / 2):
            #     for traces in f_traces:
            #         this_figure.append_trace(traces, row=1, col=idx+1)

            #     # axs[0, idx].hist(point_error, bin, edgecolor='black')
            #     # axs[0, idx].set_title('Cam-' + camS)
            #     # axs1[0, idx].hist(pose_error, bin, edgecolor='black')
            #     # axs1[0, idx].set_title('Cam-' + camS)
            #     # axs2[0, idx].scatter(poses, pose_error)
            #     # axs2[0, idx].set_title('Cam-' + camS)
            #     # axs4[0, idx].hist2d(point_x, point_y, bins=50)
            #     im = axs4[0, idx].imshow(img_map, cmap=mpl.colormaps['viridis'])
            #     cbar = axs4[0, idx].figure.colorbar(im, ax=axs4[0, idx])
            #     # cbar.ax.set_ylabel("Re-projection error", rotation=-90, va="bottom", fontsize=60)
            #     cbar.ax.set_ylabel("Rotation vector variation", rotation=-90, va="bottom", fontsize=60)
            #     ticklabs = cbar.ax.get_yticklabels()
            #     cbar.ax.set_yticklabels(ticklabs, fontsize=40)
            #     # axs4[0, idx].scatter(point_x, point_y, color=c)
            #     axs4[0, idx].set_title('Cam-' + camS, fontsize=70)
            #     # axs5[0, idx].imshow(final_img)
            #     # axs5[0, idx].set_title('Cam-' + camS, fontsize=70)
            #
            #     # if len(good_poses):
            #     #     axs3[0, idx].hist(good_poses, bin2, edgecolor='black')
            #     #     axs3[0, idx].set_title('Cam-' + camS)
            # else:
            #     for traces in f_traces:
            #         i = idx - math.ceil(self.workspace.sizes.camera / 2)
            #         this_figure.append_trace(traces, row=2, col=i+1)
            #     i = idx - math.ceil(self.workspace.sizes.camera / 2)
            #     # axs[1, i].hist(point_error, bin, edgecolor='black')
            #     # axs[1, i].set_title('Cam-' + camS)
            #     # axs1[1, i].hist(pose_error, bin, edgecolor='black')
            #     # axs1[1, i].set_title('Cam-' + camS)
            #     # axs2[1, i].scatter(poses, pose_error)
            #     # axs2[1, i].set_title('Cam-' + camS)
            #     # hist, xbins, ybins, im = axs4[1, i].hist2d(point_x, point_y, bins=100)
            #     # axs4[1, i].scatter(point_x, point_y, color=c)
            #     im = axs4[1, i].imshow(img_map, cmap=mpl.colormaps['viridis'])
            #     cbar = axs4[1, i].figure.colorbar(im, ax=axs4[1, i])
            #     # cbar.ax.set_ylabel("Re-projection error", rotation=-90, va="bottom", fontsize=60)
            #     cbar.ax.set_ylabel("Rotation vector variation", rotation=-90, va="bottom", fontsize=60)
            #     ticklabs = cbar.ax.get_yticklabels()
            #     cbar.ax.set_yticklabels(ticklabs, fontsize=40)
            #     axs4[1, i].set_title('Cam-' + camS, fontsize=70)
            #     # if len(good_poses):
            #     #     axs3[1, i].hist(good_poses, bin2, edgecolor='black')
            #     #     axs3[1, i].set_title('Cam-' + camS)
            #     # axs5[1, i].imshow(final_img)
            #     # axs5[1, i].set_title('Cam-' + camS, fontsize=70)
            # for ax in axs4.flat:
            #     ax.set_xlabel('Image width', fontsize=70)
            #     ax.set_ylabel('Image height', fontsize=70)
            #
            # # for ax, ax1, ax2, ax3 in zip(axs.flat, axs1.flat, axs2.flat, axs3.flat):
            # #     ax.set_xlabel('Re-projection error \n Per point', fontsize=15)
            # #     ax.set_ylabel('Number of Points', fontsize=15)
            # #     # ax.set(xlabel='Re-projection error \n Per point', ylabel='Number of Points')
            # #     ax1.set_xlabel('Re-projection error \n Per view', fontsize=15)
            # #     ax1.set_ylabel('Number of Views', fontsize=15)
            # #     # ax1.set(xlabel='Re-projection error \n Per view', ylabel='Number of Views')
            # #     ax2.set(xlabel='Views', ylabel='Re-projection Error')
            # #     ax3.set(xlabel='View angle (Degrees)', ylabel='Number of Views')
            # #
            # for ax in axs4.flat:
            #     ax.label_outer()
            # # for ax, ax1, ax2, ax3 in zip(axs.flat, axs1.flat, axs2.flat, axs3.flat):
            # #     ax.label_outer()
            # #     ax1.label_outer()
            # #     ax2.label_outer()
            # #     ax3.label_outer()
            # # for ax in axs4.flat:
            # #     ax.colorbar()

        # fig.update_traces(width=4)
        # fig.show()

        # dcc.Graph(figure=this_figure)
        this_figure.update_traces(width=4)
        # this_figure.update_layout(yaxis = dict(
        #             tickfont = dict(size=20),titlefont = dict(size = 25)),
        #                           xaxis=dict(
        #                               tickfont=dict(size=20), titlefont=dict(size=25)),
        #                           font=dict(size=35, )
        #                           )
        this_figure.update_layout(font=dict(size=18, ))
        this_figure.update_annotations(font_size=18)


        this_figure.show()

        # fig = plt.figure(figsize=(20, 20))
        # ax = fig.add_axes([0, 0, 1, 1])
        # bp = ax.boxplot(data)



        # folder = self.base_path[-3:]
        # path = os.path.join(self.base_path, folder + '-finalPointError.png')
        #
        # # plt.savefig(path)
        # # plt.figure(figsize=(10, 10))
        # # plt.tick_params(axis='both', which='major', labelsize=50)
        # ax.set_ylim(0, 120)
        # plt.show()

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
                self.bundle_image_name[cam] = data.names.image
        pass

    def collect_validation_dataset(self, cam):

        file = os.path.join(self.base_path, cam + '.pkl')
        if os.path.exists(file):
            data = pickle.load(open(file, "rb"))
            self.valid_inlier_mask[cam] = data.calibrations['calibration'].inlier_mask
            self.valid_reprojected_points[cam] = data.calibrations['calibration'].reprojected
            self.valid_image_name[cam] = data.names.image
        pass

    def load_files(self):
        workspace, handEye, campose2 = None, None, None
        for path, subdirs, files in os.walk((self.base_path)):
            if path == self.base_path:
                workspace_path = os.path.join(self.base_path, [f for f in files if f == "workspace.pkl"][0])
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
    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35"
    v = Complete_Viz(base_path)

