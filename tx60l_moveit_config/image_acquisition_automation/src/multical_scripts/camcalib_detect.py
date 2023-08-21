import src.multical.app.boards as boards
from src.multical.image.detect import *
from src.multical.board import load_config as board_load_config
import json
from src.multical_scripts.board_angle import *

def detect_img(board_file, detected_img, write_path=None):
    b = boards.Boards(boards=board_file, detect=detected_img, pixels_mm=10, show_image=False,
                      write_detect_image=write_path)
    board_config, detection = b.execute()
    return board_config, detection

if __name__ == '__main__':

    # print(cv2.__version__)

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38/boards.yaml"
    img_file = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38/08320217\p3.png"
    cam_intrinsic_file = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38\calibration.json"
    img = load_image(img_file)
    cam_intrinsic = json.load(open(cam_intrinsic_file))
    board_config0 = board_load_config(board_yaml)
    board_config, detection = detect_img(board_config0, img)
    board_list = [b for b in board_config.keys()]
    cam_matrix, cam_dist = np.array(cam_intrinsic['cameras']['08320220']['K'],dtype=np.float32) , np.array(cam_intrinsic['cameras']['08320220']['dist'],dtype=np.float32)
    for board_num, b in enumerate(detection):
        if b.corners.size != 0:
            ids = detection[board_num].ids
            corners = np.array(detection[board_num].corners, dtype=np.float32).reshape(-1, 2)
            print(type(corners))
            undistorted = cv2.undistortPoints(corners, cam_matrix, cam_dist, P=cam_matrix).reshape(-1, 2)
            detected_board = board_list[board_num]
            adjusted_points = board_config[detected_board].adjusted_points
            # objpoints = adjusted_points[ids].astype('float32')
            objpoints = np.array([adjusted_points[a] for a in ids], dtype=np.float32).reshape((-1, 3))
            
            ret, rvecs, tvecs, euler_deg, view_angle = board_pose(objpoints,
                                                undistorted, ids, cam_matrix, cam_dist, method="solvePnP")
            pass
