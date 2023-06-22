import cv2
import cvb
import ast
# import helpers.make_directory
from .helpers import make_directory


def discoverDevices():
    """
    List information about all devices. Available cameras are being returned.
    """
    discover = cvb.DeviceFactory.discover_from_root()
    num_of_devices = len(discover)
    print("number of devices: ", num_of_devices)

    cameras_access_token = []
    cameras_info = []
    serialnum_lst = []
    i = 0
    for device in discover:
        device_info_tmp = ast.literal_eval(device.access_token)
        device_info_key = list(device_info_tmp)[0]
        device_info = device_info_tmp[device_info_key][0]

        # check if listed device is a camera
        if "ifaces" in device_info:
            device_dict = ast.literal_eval(device.access_token)
            # only use api from vendor Stemmer Imaging, not from Matrix Vision
            vendor = device_dict["tls"][0]["infos"][1]["value"]
            if vendor == "STEMMER IMAGING":
                serialnum = device_dict["tls"][0]["ifaces"][0]["devices"][0]["infos"][3]["value"]
                print("###################### camera ", serialnum, " ######################")
                print("")

                cameras_access_token.append(device.access_token)
                cameras_info.append(device_info)
                serialnum_lst.append(serialnum)
                i = i + 1
    print("number of cameras: ", i)

    return cameras_access_token, cameras_info, serialnum_lst

class CamStreamer:
    def __init__(self, idx=-1):
        """
        To use every detected camera use idx = -1. If more cameras are attached than being used here,
        the ones discovered first will be the ones used.
        """
        # get camera meta data
        self.camidx = idx

        # discover all available cameras
        self.access_token_lst, self.cameras_info, self.serialnum_lst = discoverDevices()

        self.num_cams = len(self.access_token_lst)
        self.device_lst = []
        self.stream_lst = []

        # Create list containing the current image of each camera
        self.images = [None] * self.num_cams

        # get resources for all cameras
        for d in range(self.num_cams):
            # get device
            device = cvb.DeviceFactory.open(self.access_token_lst[d], port=0)
            # get stream
            stream = device.stream()

            # add to list
            self.device_lst.append(device)
            self.stream_lst.append(stream)

    def start_cvb_image_acquisition(self, pose):
        """
        Acquires new images from cameras (one after another).
        Convert image to numpy array and save it in self.images[cam_idx]
        """
        while True:
            # multi cam: jump from camera to camera
            base_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/'
            # pose = 1
            for cam_idx in range(self.num_cams):
                self.stream_lst[cam_idx].start()
                try:
                    image, status = self.stream_lst[cam_idx].wait_for(1000)
                    if status == cvb.WaitStatus.Ok:
                        # Convert image into numpy array
                        self.images[cam_idx] = cvb.as_array(image, copy=True)
                        serialnum = str(self.serialnum_lst[cam_idx])
                        new_path = base_path + serialnum + '/'
                        make_directory(new_path)
                        saved_path = new_path + '/p' + str(pose) + '.png'
                        cv2.imwrite(saved_path, self.images[cam_idx])

                    else:
                        raise RuntimeError("[ERROR] timeout during wait" if status == cvb.WaitStatus.Timeout
                                           else "acquisition aborted")
                except:
                    print("[ERROR] Aborting publishing")
                    self.stream_lst[cam_idx].try_abort()
                    exit(-1)

                self.stream_lst[cam_idx].try_abort()

            if cam_idx == self.num_cams - 1:
                break

