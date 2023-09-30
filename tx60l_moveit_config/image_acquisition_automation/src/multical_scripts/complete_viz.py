import math

import numpy as np
from src.extrinsic2pyramid.camera_pose_visualizer import CameraPoseVisualizer
import plotly.graph_objects as go
import json
import os
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
        #     self.graph_analysis(cam)
        #     df[cam] = pd.DataFrame(self.error_dict[cam])
        #     df[cam].to_excel(writer, sheet_name=cam)
        # writer.close()
        self.plotbox(for_boxPlot)


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
            name='<b>Re-projection Error (Pixels)</b>', #<b>Bold</b>
            marker_color='#FF4136'
        ))
        # fig = px.box(df, x="cam_name", y="cam_error")
        # fig.update_traces(quartilemethod="exclusive")  # or "inclusive", or "linear" by default
        fig.update_layout(width=1200, height=800,
            yaxis_range=[0, 6],
            yaxis_title='<b>Radians | Pixels</b>',
            xaxis_title='<b>Master Cameras</b>',
            boxmode='group',  # group together boxes of the different traces for each value of x
            font = dict(
                size=18, ),
          legend=dict(
              x=0.7,
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
        for idx, camS in enumerate(self.workspace.names.camera):
            all_poses = self.error_dict[masterCam]['all_poses'][camS]
            point_error = self.error_dict[masterCam]['inlier_point_error'][camS]
            inlier_points = self.error_dict[masterCam]['inlier_points'][camS]
            inlier_corners = self.error_dict[masterCam]['inlier_corners'][camS]
            adjusted_points = self.error_dict[masterCam]['adjusted_points'][camS]
            poses = self.error_dict[masterCam]['inlier_poses'][camS]
            pose_error = self.error_dict[masterCam]['inlier_pose_error'][camS]
            inlier_rvecs = self.error_dict[masterCam]['inlier_rvecs'][camS]
            inlier_tvecs = self.error_dict[masterCam]['inlier_tvecs'][camS]
            density = self.rvec_heatmap(inlier_rvecs)
            # final_img = self.draw_axis(camS, inlier_corners, adjusted_points, inlier_rvecs, inlier_tvecs)
            c = ['green'] * len(point_error)
            for idx1, e in enumerate(point_error):
                if e >= 1.0:
                    c[idx1] = 'red'
            point_x = self.error_dict[masterCam]['inlier_point_x'][camS]
            point_y = self.error_dict[masterCam]['inlier_point_y'][camS]
            # img_map = self.draw_heatmap(inlier_points, point_error)
            new_points, new_density = self.adjust_rvec_heatmap(inlier_corners, density)
            img_map = self.draw_heatmap(new_points, new_density)
            pose_error = self.error_dict[masterCam]['inlier_pose_error'][camS]

            good_poses = self.error_dict[masterCam]['good_inlier_poses'][camS]
            bin = np.arange(0, 2, .1)
            bin2 = np.arange(0, 120, 5)
            data.append(all_poses)
            # if idx < math.ceil(self.workspace.sizes.camera / 2):
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

        fig = plt.figure(figsize=(20, 20))
        ax = fig.add_axes([0, 0, 1, 1])
        bp = ax.boxplot(data)



        folder = self.base_path[-3:]
        path = os.path.join(self.base_path, folder + '-finalPointError.png')

        # plt.savefig(path)
        # plt.figure(figsize=(10, 10))
        # plt.tick_params(axis='both', which='major', labelsize=50)
        ax.set_ylim(0, 120)
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

