import multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:/MY_DRIVE_N/Masters_thesis/Dataset/Latest Pattern/board_images/boards.yaml"
    detect_img = "D:/MY_DRIVE_N/Masters_thesis/Dataset/Latest Pattern/board_images/b108.png"
    b = boards.Boards(boards=board_yaml, detect=detect_img, pixels_mm=10)
    # b = boards.Boards(boards=board_yaml, detect=detect_img)

    b.execute()

