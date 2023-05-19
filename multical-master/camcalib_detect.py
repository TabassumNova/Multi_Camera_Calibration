import multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/train/boards.yaml"
    detect_img = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V10_debug\data/train/08320217/1_p114.png"
    b = boards.Boards(boards=board_yaml, detect=detect_img)

    b.execute()

