import cv2
from .helpers import make_directory
from .TIS_Cam import TIS_list_devices, print_device, TIS

def start_tis_image_acquisition(self, pose):
    """
    Acquires new images from cameras (one after another).
    Convert image to numpy array and save it in self.images[cam_idx]
    """
    camera_list = TIS_list_devices()
    num_cams = len(camera_list)
    print('num_cam: ',num_cams)
    tis_camera = []
    camera_serial = []
    base_path = '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/image/'
    new_path = []

    for cam_idx in range(num_cams):
        camera_serial.append(camera_list[cam_idx].get_properties().get_string("serial"))
        tis_camera.append(TIS(camera_serial[cam_idx], 1920, 1080, 15, 1, False))
        new_path.append(base_path + camera_serial[cam_idx] + '/')
        make_directory(new_path[cam_idx])
        tis_camera[cam_idx].Start_pipeline()

    while True:
        # multi cam: jump from camera to camera
        for cam_idx in range(num_cams):
            try:
                image = tis_camera[cam_idx].Get_image()
                if image.all() != None:
                    error = 0
                    saved_path = new_path[cam_idx] + '/p' + str(pose) + '.png'
                    cv2.imwrite(saved_path, image)
                else:
                    print("No image reveived ")
                    error = error + 1
            except:
                print("[ERROR] Aborting publishing")
                exit(-1)
            tis_camera[cam_idx].Stop_pipeline()
        if cam_idx == num_cams - 1:
            break



