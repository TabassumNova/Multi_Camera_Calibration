import pickle
import json
import plotly.graph_objects as go
import numpy as np
import plotly.express as px
import pandas as pd

pickle_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V30\workspace.pkl"

def visualizer_data(ws):
    '''
    :return: x -> view angle x
             y -> view angle y
             z -> re-projection error
    '''
    x = []
    y = []
    z = []
    name = []
    for cam in range(ws.sizes.camera):
        for img in range(ws.sizes.image):
            for board in range(ws.sizes.board):
                if ws.pose_table.valid[cam][img][board]:
                    x.append(ws.pose_table.view_angles[cam][img][board][0])
                    y. append(ws.pose_table.view_angles[cam][img][board][1])
                    z.append(ws.pose_table.reprojection_error[cam][img][board])
                    text = ws.names.camera[cam] + "/" + ws.names.image[img] + '/' + ws.names.board[board]
                    name.append(text)
    data = {'view_angle X': x, 'view_angle Y': y, 'reprojection error': z, 'name': name}

    return data


def visualizer(data):
    df = pd.DataFrame(data)
    text = df['name']
    fig = px.scatter_3d(df, x="view_angle X", y="view_angle Y", z='reprojection error', hover_name=text)
    fig.show()
    pass

if __name__ == '__main__':
    workspace = pickle.load(open(pickle_path, "rb"))
    data = visualizer_data(workspace)
    visualizer(data)
    pass
