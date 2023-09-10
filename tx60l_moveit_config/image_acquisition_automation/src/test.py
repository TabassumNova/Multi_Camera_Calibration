import os

base_path = "D:\MY_DRIVE_N\Masters_thesis\Dataset\V35_test"
for path, subdirs, files in os.walk(base_path):
    for name in files:
        if "calibration" in name:
            name1 = name.split('calibration')
            pass