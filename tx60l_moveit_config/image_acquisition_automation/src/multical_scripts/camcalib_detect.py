import src.multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset\isohedron/boards.yaml"
    detect_img = "D:\MY_DRIVE_N\Masters_thesis\Dataset\isohedron/p4.png"
    b = boards.Boards(boards=board_yaml, detect=detect_img, pixels_mm=10, show_image=True, write_detect_image="D:\MY_DRIVE_N\Masters_thesis\Dataset\isohedron")
    # b = boards.Boards(boards=board_yaml, detect=detect_img)

    detection = b.execute()
    print('end')

