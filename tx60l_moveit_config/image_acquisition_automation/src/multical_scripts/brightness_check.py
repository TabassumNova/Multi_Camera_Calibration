import os
import cv2
from PIL import Image, ImageStat
import numpy as np
from skimage.exposure import is_low_contrast

def check_brightness1(folder):

    for path, subdirs, files in os.walk(folder):
        mean_dict = {}
        rms_dict = {}
        light_dict = {}
        for name in files:
            image = cv2.imread(os.path.join(path, name))
            im_gray = image[:, :, 0]
            ret, thresh = cv2.threshold(im_gray, 127, 255, 0)
            contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
            out_path = path + '/out_' + name
            cv2.imwrite(out_path, image)
            # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            # cv2.resizeWindow('image', 600, 600)
            # cv2.imshow('image', im_gray)
            #
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            im_pil = Image.open(os.path.join(path, name))
            stat = ImageStat.Stat(im_pil)
            mean = stat.mean[0]
            rms = stat.rms[0]
            mean_dict[name] = mean
            rms_dict[name] = rms
    pass

def check_brightness2(folder):
    for path, subdirs, files in os.walk(folder):
        light_dict = {}
        for name in files:
            im_pil = Image.open(os.path.join(path, name))
            histogram = im_pil.histogram()
            pixels = sum(histogram)
            brightness = scale = len(histogram)

            for index in range(0, scale):
                ratio = histogram[index] / pixels
                brightness += ratio * (-scale + index)

            light_dict[name] = 1 if brightness == 255 else brightness / scale
    pass

if __name__ == '__main__':
    folder = "D:\MY_DRIVE_N\Masters_thesis\Dataset/brightness_check_method"
    check_brightness1(folder)
    pass