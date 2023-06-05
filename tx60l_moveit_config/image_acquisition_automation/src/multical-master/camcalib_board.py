import multical.app.boards as boards

if __name__ == '__main__':

    board_yaml = "D:/MY_DRIVE_N/Masters_thesis/Dataset/Latest Pattern/boards.yaml"
    write_img = "D:/MY_DRIVE_N/Masters_thesis/Dataset/Latest Pattern/board_images"
    b = boards.Boards(boards=board_yaml, write=write_img, paper_size='A4', pixels_mm=10)

    b.execute()