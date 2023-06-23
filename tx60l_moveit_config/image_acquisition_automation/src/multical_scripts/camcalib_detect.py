import src.multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/view_plan/viewPlan2/boards.yaml"
    detect_img = "/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/08320220/p36.png"
    b = boards.Boards(boards=board_yaml, detect=detect_img, pixels_mm=10, show_image=True)
    # b = boards.Boards(boards=board_yaml, detect=detect_img)

    detection = b.execute()
    print('end')

