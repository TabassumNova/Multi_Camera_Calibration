# Workspace
- Dataset Link (https://1drv.ms/f/s!At7UW7zuEoCCivxe6B8zsCaGNpZ2qA?e=DtCZi4)

## Dataset Description
- boards.yaml : Boards configuration file
- calibration.detections.pkl : Contains all the detected points in pixel
- intrinsic_26June.json : Contains Intrinsic Parameters for all the cameras
- poses_29June_1.json : Robot poses
- workspace.pkl : Workspace

## workspace.pkl file
- detected_points : Contains all the detected points in pixel
- boards[board_num].adjusted_points : 3D points of the boards
- pose_table.poses : Extrinsic parameters for Board to cameras

## How to open pickel file?
```
workspace = pickle.load(open('workspace.pkl_path', "rb"))
```
