from src.multical_scripts.handEye_final import *

if __name__ == '__main__':
    base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35"
    # main1(base_path, limit_image=10)
    main4(base_path, limit_images=6, limit_board_image=6, calculate_handeye=True, check_cluster=True)
    pass