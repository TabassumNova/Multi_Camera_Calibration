import src.multical.app.calibrate as calibrate
import src.multical.config.arguments as args
import src.multical.config.workspace as workspace
import os
import plotly.graph_objects as go
import matplotlib.pyplot as plt

class Icosahedron_vs_Cube():
    def __init__(self, path):
        self.path = path
        self.intrinsic_path = None
        self.icosahedron_path = None
        self.cube_path = None
        self.ico_boardPath = None
        self.cube_boardpath = None
        self.ico_image_path = None
        self.cube_image_path = None
        self.ico_workspace = None
        self.cube_workspace = None
        self.load_dataset()
        self.board_detection()
        self.draw_viz()

    def load_dataset(self):
        for path, subdirs, files in os.walk(self.path):
            for f in files:
                if f == 'intrinsic.json':
                    self.intrinsic_path = os.path.join(self.path, f)
            for dirs in subdirs:
                if dirs == 'icosahedron':
                    self.ico_image_path = os.path.join(self.path, dirs)
                    for path, subdirs, files in os.walk(self.ico_image_path):
                        for name in files:
                            if name == 'boards.yaml':
                                self.ico_boardPath = os.path.join(path, name)
                if dirs == 'cube':
                    self.cube_image_path = os.path.join(self.path, dirs)
                    for path, subdirs, files in os.walk(self.cube_image_path):
                        for name in files:
                            if name == 'boards.yaml':
                                self.cube_boardPath = os.path.join(path, name)

    def board_detection(self):
        pathO = args.PathOpts(image_path=self.ico_image_path)
        cam = args.CameraOpts(intrinsic_error_limit=1.0,
                              calibration=self.intrinsic_path)
        pose_estimation_method = "solvePnPGeneric"
        runt = args.RuntimeOpts(pose_estimation=pose_estimation_method)
        opt = args.OptimizerOpts(outlier_threshold=0.5, fix_intrinsic=True, fix_camera_poses=True, iter=2)

        c = calibrate.Calibrate(paths=pathO, camera=cam, runtime=runt, optimizer=opt)
        self.ico_workspace = c.execute_board()

        path1 = args.PathOpts(image_path=self.cube_image_path)
        cam1 = args.CameraOpts(intrinsic_error_limit=1.0, calibration=self.intrinsic_path)
        runt1 = args.RuntimeOpts(pose_estimation=pose_estimation_method)
        c1 = calibrate.Calibrate(paths=path1, camera=cam1, runtime=runt1, optimizer=opt)
        self.cube_workspace = c1.execute_board()
        pass

    def draw_viz(self):
        # for icosahedron
        ico_x = []
        ico_y = []
        ico_z = []
        cube_x = []
        cube_y = []
        cube_z = []
        for cam in range(self.ico_workspace.sizes.camera):
            for img in range(self.ico_workspace.sizes.image):
                for board in range(self.ico_workspace.sizes.board):
                    if self.ico_workspace.pose_table.valid[cam][img][board]:
                        angles = self.ico_workspace.pose_table.view_angles[cam][img][board]
                        ico_x.append(angles[0])
                        ico_y.append(angles[1])
                        ico_z.append(angles[2])

        for cam in range(self.cube_workspace.sizes.camera):
            for img in range(self.cube_workspace.sizes.image):
                for board in range(self.cube_workspace.sizes.board):
                    if self.cube_workspace.pose_table.valid[cam][img][board]:
                        angles = self.cube_workspace.pose_table.view_angles[cam][img][board]
                        cube_x.append(angles[0])
                        cube_y.append(angles[1])
                        cube_z.append(angles[2])

        # final_layout = go.Figure()
        # final_layout.add_trace(
        #     go.Scatter3d(
        #         x=ico_x,
        #         y=ico_y,
        #         z=ico_z,
        #         mode='markers',textposition="bottom center",
        #         name='Icosahedron'
        #     )
        # )
        # final_layout.add_trace(
        #     go.Scatter3d(
        #         x=cube_x,
        #         y=cube_y,
        #         z=cube_z,
        #         mode='markers', textposition="bottom center",
        #         name='Cube'
        #     )
        # )
        # final_layout.show()

        # matplotlib
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(ico_x, ico_y, ico_z, marker='o', label='Icosahedron')
        # plt.show()

        # ax1 = fig.add_subplot(projection='3d')
        ax.scatter(cube_x, cube_y, cube_z, marker='^', label='Cube')
        ax.set_xlabel('Roll', fontweight='bold')
        ax.set_ylabel('Pitch', fontweight='bold')
        ax.set_zlabel('Yaw', fontweight='bold')
        ax.legend()
        plt.show()
        fig.savefig('D:\MY_DRIVE_N\Masters_thesis/final report\Report_topics\Blender Images/final_img/foo.png')

if __name__ == '__main__':
    '''
    icosahedron_path : contains one cam image and boards.yaml
    cube_path : same as icosahedron
    '''
    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\icosahedron_cube"
    comparison = Icosahedron_vs_Cube(base_path)

    pass