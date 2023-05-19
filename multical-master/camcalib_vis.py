import multical.app.vis as vis

if __name__ == '__main__':
    pkl_file = "D:\MY_DRIVE_N\Masters_thesis\Dataset\docker_V11\input\data\calibration.pkl"
    v = vis.Vis(workspace_file=pkl_file)
    v.execute()
