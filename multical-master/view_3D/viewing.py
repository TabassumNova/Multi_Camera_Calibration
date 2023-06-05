import json
from multical.transform.matrix import *
import plotly.graph_objects as go
import numpy as np
from scipy.spatial import distance

f = open('D:\MY_DRIVE_N\Masters_thesis\Dataset/view_plan\V2\calibration.json')

data = json.load(f)
cameras = data['cameras']
camera_poses = {}
for k in data['camera_poses']:
    camera_poses[k[0:8]] = data['camera_poses'][k]

camera_dict = {}
camx = []
camy = []
camz = []

for camera in data['cameras']:
    camera_dict[camera] = {}
    R = np.array(camera_poses[camera]['R'])
    t = np.array(camera_poses[camera]['T'])
    rt_matrix = join(R, t)
    camera_dict[camera]['rt_matrix'] = rt_matrix
    center = -R.T@t
    camera_dict[camera]['center'] = center
    camx.append(center[0])
    camy.append(center[1])
    camz.append(center[2])

fig = go.Scatter3d(
        x=camx,
        y=camy,
        z=camz,
        mode='markers',
        marker=dict(
            size=6,
            color='rgb(0,255,0)'
        ),
        text= [k for k in data['cameras']]
    )
final_layout = go.Figure(fig)
final_layout.show()

for camera1 in data['cameras']:
    center1 = camera_dict[camera1]['center']
    for camera2 in data['cameras']:
        center2 = camera_dict[camera2]['center']
        dst = distance.euclidean(center1, center2)
        print('Distance between [', camera1, '] & [', camera2, '] ', dst)



