import src.multical.app.boards as boards
from src.multical.image.detect import *
from src.multical.board import load_config as board_load_config

def detect_img(board_file, detected_img, write_path=None):
    b = boards.Boards(boards=board_file, detect=detected_img, pixels_mm=10, show_image=False,
                      write_detect_image=write_path)
    board_config, detection = b.execute()
    return board_config, detection

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38/boards.yaml"
    img_file = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V38/08320217\p1.png"
    img = load_image(img_file)
    board_config0 = board_load_config(board_yaml)
    board_config, detection = detect_img(board_config0, img)
    for b in detection:
        if b.corners.size != 0:
            pass
