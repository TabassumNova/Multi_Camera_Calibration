import numpy as np
import os
import cv2

directory = 'D:\MY_DRIVE_N\Masters_thesis\Dataset\V23/'
new_directory = 'D:\MY_DRIVE_N\Masters_thesis\Dataset/V23_sorted/'

for subdir, dirs, files in os.walk(directory):
    for camera in dirs:
        os.mkdir(os.path.join(new_directory, camera))
        for image in os.listdir(directory+camera):
            image_path = os.path.join(directory, camera, image)
            new_image_path = os.path.join(new_directory, camera, image)
            frame = cv2.imread(image_path)
            cv2.imwrite(new_image_path, frame[:,:,0])