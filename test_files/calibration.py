# source ROS2 
import sys 
import os 
# os.system('bash /opt/ros/dashing/setup.sh') # needs to be done in terminal before starting python 
import cv2 
import numpy as np 
import glob
import json 
import time 
import utils 


sys.path.append(os.path.abspath(os.path.abspath(__file__)+"/../../../../..")) 
currentscript_path = os.path.abspath(os.path.abspath(__file__)+"/../")   
# import stemmer 



def main(args=None):
    """
    # calibration best practices 
    https://calib.io/blogs/knowledge-base/calibration-best-practices 

    opencv calibration tutorials 
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html 
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_table_of_contents_calib3d/py_table_of_contents_calib3d.html
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
    https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html 
    https://www.learnopencv.com/camera-calibration-using-opencv/ 
    https://medium.com/@aliyasineser/opencv-camera-calibration-e9a48bdd1844 
    https://medium.com/analytics-vidhya/camera-calibration-with-opencv-f324679c6eb7 
    https://nikatsanka.github.io/camera-calibration-using-opencv-and-python.html 
    https://medium.com/@aliyasineser/opencv-camera-calibration-e9a48bdd1844 

    circle grid 
    https://longervision.github.io/2017/03/18/ComputerVision/OpenCV/opencv-internal-calibration-circle-grid/ 
    https://answers.opencv.org/question/88457/chessboard-and-circle-calibration-provide-radically-different-results/ 
    https://stackoverflow.com/questions/39272510/camera-calibration-with-circular-pattern/39328596

    create own pattern https://docs.opencv.org/master/da/d0d/tutorial_camera_calibration_pattern.html 

    camera configuration https://www.emva.org/wp-content/uploads/GenICam_SFNC_2_3.pdf 

    detection  
    https://longervision.github.io/categories/Computer-Vision/OpenCV/ 
    https://www.learnopencv.com/blob-detection-using-opencv-python-c/ 
    """
    calibrator = Calibrator() 
    calibrator.calibrate() 

    return 0 

################################################################################
# 
################################################################################
class Calibrator(): 

    def __init__(self): 
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 

        #TODO get Stemmer calibration data 
        # cvb.foundation.IntrinsicCorrectionModel #doesn't work docs doesn't give more info 
        

    def calibrate(self, image=None, save=True, show=False): 
        """
        adapted from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration 
        """
        # calibration data directory 
        calibration_time = str(time.time()).replace(".","_") 
        # calibration_dir = currentscript_path+'/images/calibration/calib_'+calibration_time+'/'
        calibration_dir = 'C:/Users/FAPS/Desktop/Nova/camera_calib_nova/5MP_Cameras/cam742_images/'
        utils.ensure_dir(calibration_dir) 

        #TODO insert the real distances from the calibration plate .descr-file 
        #TODO check units of the calibration info: try to use mm  
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((7*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:7].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # images = glob.glob(currentscript_path+'/images/calibration/*.png') 
        images = glob.glob('C:/Users/FAPS/Desktop/Nova/camera_calib_nova/5MP_Cameras/cam742_images/*.bmp')

        for fname in images:
            img = cv2.imread(fname)
            print(np.array(img))
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            # ret, corners = cv2.findCirclesGrid(gray, (3,3),None) 
            ret, corners = cv2.findChessboardCorners(gray, (3,3), None)
            print(np.array(ret))

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),self.criteria)
                imgpoints.append(corners2)

                # Draw and display the corners 
                img = cv2.drawChessboardCorners(img, (3,3), corners2,ret) 
                if(True): 
                    cv2.namedWindow('image',cv2.WINDOW_NORMAL) 
                    cv2.resizeWindow('image', (600,600)) 
                    cv2.imshow('img',img)
                    cv2.waitKey(50000) 

            # save marked image 
            if(save): 
                fname_calibrated = fname.replace(".png", "_calibrated.png").split("/")[-1] 
                fname_calibrated_dir = calibration_dir+fname_calibrated 
                cv2.imwrite( fname_calibrated_dir, cv2.cvtColor(img, cv2.COLOR_BGR2RGB) ) 
                print("Saved image to", fname_calibrated_dir) 

        cv2.destroyAllWindows() 

        # calibrate 
        #TODO: use cv2.calibrateCameraRO() instead, to improve accuracy 
        # https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga11eeb16e5a458e1ed382fb27f585b753
        # meaning of ret https://stackoverflow.com/questions/29628445/meaning-of-the-retval-return-value-in-cv2-calibratecamera 
        # ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None) 

        #TODO: refine camera matrix 
        # meaning of getOptimalNewCameraMatrix 
        # https://answers.opencv.org/question/101398/what-does-the-getoptimalnewcameramatrix-function-does/
        # https://stackoverflow.com/questions/39432322/what-does-the-getoptimalnewcameramatrix-does-in-opencv
        # img = cv2.imread('left12.jpg')
        # h,  w = img.shape[:2]
        # newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h)) 

        # Re-projection Error 
        # Re-projection error gives a good estimation of just how exact is the found parameters. 
        # This should be as close to zero as possible. 
        """
        mean_error = 0.0 
        tot_error = 0.0  
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            tot_error += error 

        print ("total error: ", mean_error/len(objpoints)) 

        # save calibration data 
        calibration_data = {} 
        calibration_data["ret"] = ret 
        calibration_data["mtx"] = mtx.tolist()  
        calibration_data["dist"] = dist.tolist()  
        calibration_data["rvecs"] = [r.tolist() for r in rvecs]   
        calibration_data["tvecs"] = [t.tolist() for t in rvecs]      
        intrinsic_matrix_path = calibration_dir + 'intrinsic_matrix_'+calibration_time+'.json' 
        with open(intrinsic_matrix_path, 'w', encoding='utf-8') as f: 
            json.dump(calibration_data, f, ensure_ascii=False)#, indent=4) 

        """
        return 0 

    def loadIntrinsicMatrix(self, path): 
        
        calibration_data = {} 
        with open(path) as f:
            calibration_data = json.load(f) 

        ret = calibration_data["ret"] 
        mtx = calibration_data["mtx"]   
        dist = calibration_data["dist"]   
        rvecs = calibration_data["rvecs"]    
        tvecs = calibration_data["tvecs"]     

        return ret, mtx, dist, rvecs, tvecs 


################################################################################
# 
################################################################################
if __name__ == '__main__':
    main()
