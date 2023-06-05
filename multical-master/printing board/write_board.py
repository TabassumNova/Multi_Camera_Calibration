import numpy as np
import os
import cv2, PIL, os
from cv2 import aruco
import operator
from scipy.spatial.transform import Rotation as R
import glob

def board_config(dict_id, offset, width, height, square_length, marker_length):
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    aruco_dict.bytesList = aruco_dict.bytesList[offset:]
    board = cv2.aruco.CharucoBoard_create(width, height, square_length, marker_length, aruco_dict)

    return board

if __name__ == '__main__':
    # set board params
    num_board = 5
    dict_id = 11
    width, height = (12, 9)
    square_length, marker_length = 0.013, 0.009
    board_offset = [0, 54, 108, 162, 216]

    output_path = "D:/MY_DRIVE_N/Masters_thesis/Dataset/Latest Pattern/board_images/"

    for offset in board_offset:
        board = board_config(dict_id, offset, width, height, square_length, marker_length)
        img = board.draw((1000, 1000), 10, 50)
        img_path = output_path + 'Board_' + str(offset) + '.png'
        cv2.imwrite(img_path, img)

