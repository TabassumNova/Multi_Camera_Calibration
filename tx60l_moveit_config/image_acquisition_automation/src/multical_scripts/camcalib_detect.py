import src.multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V23\debug/boards.yaml"
    detect_img = "D:\MY_DRIVE_N\Masters_thesis\Dataset\geometrical_method/08320217/p13.png"
    b = boards.Boards(boards=board_yaml, detect=detect_img, pixels_mm=10, show_image=True)
    # b = boards.Boards(boards=board_yaml, detect=detect_img)

    detection = b.execute()
    print('end')

