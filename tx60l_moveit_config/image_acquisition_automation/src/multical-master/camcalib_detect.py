import multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\Dataset/view_plan\V2/boards.yaml"
    detect_img = "D:\MY_DRIVE_N\Masters_thesis\Dataset/view_plan\V3/08320218/p1.png"
    b = boards.Boards(boards=board_yaml, detect=detect_img, pixels_mm=10, show_image=True)
    # b = boards.Boards(boards=board_yaml, detect=detect_img)

    detection = b.execute()
    print('end')

