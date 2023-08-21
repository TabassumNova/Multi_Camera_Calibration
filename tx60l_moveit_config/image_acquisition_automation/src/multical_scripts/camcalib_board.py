import src.multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:\MY_DRIVE_N\Masters_thesis/final report\Report_topics\Blender Images/boards.yaml"
    write_img = "D:\MY_DRIVE_N\Masters_thesis/final report\Report_topics\Blender Images/"
    b = boards.Boards(boards=board_yaml, write=write_img, paper_size='A4', pixels_mm=10)

    b.execute()