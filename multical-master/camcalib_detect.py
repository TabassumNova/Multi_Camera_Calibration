import multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis\mymy\camnodes_pkg\camnodes_pkg\multical-master\detect_test/t2.yaml"
    detect_img = "D:\MY_DRIVE_N\Masters_thesis\mymy\camnodes_pkg\camnodes_pkg\multical-master\detect_test/p2.bmp"
    b = boards.Boards(boards=board_yaml, detect=detect_img)

    b.execute()
