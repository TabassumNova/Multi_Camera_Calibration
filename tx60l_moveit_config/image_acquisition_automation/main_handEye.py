from src.multical_scripts.handEye_final import *
import cv2
if __name__ == '__main__':
    base_path = "/home/nova/Desktop/Nova/Calibration_paper/datasets/V35"
    print(cv2.__version__)
    # main1(base_path, limit_image=10)
    main4(base_path, limit_images=6, limit_board_image=6, calculate_handeye=True, check_cluster=True)
    pass