import os
import xlrd, xlsxwriter
import numpy as np
import plotly.graph_objects as go
from re import X
import numpy as np 
from scipy.spatial.transform import Rotation as ScyRot 
import cv2


def make_directory(path):
  isExist = os.path.exists(path)
  if not isExist:

    # Create a new directory because it does not exist
    os.makedirs(path)
    print("The new directory is created!")

# Input row as int
def read_from_xlsx(row):
  # Open the Workbook
  workbook = xlrd.open_workbook('/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/joint_values.xlsx')

  # Open the worksheet
  worksheet = workbook.sheet_by_index(0)

  joint = []
  for j in range(1, 7):
    joint.append(worksheet.cell_value(row, j))
  pose = int(worksheet.cell_value(row, 0))
  print('joint values of Pose ', pose, ' : ', joint)

  return pose, joint


def write_cartesian_position(row, position):
  workbook = xlsxwriter.Workbook(
            '/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/joint_values.xlsx')
  worksheet = workbook.add_worksheet()
  worksheet.write(row, 11, (str(position[0])+','+str(position[1])+','+str(position[2])))
  # workbook.close()

def create_points(pointx, pointy, pointz, planNum):
  xBound = np.arange(pointx-0.1, pointx+0.1, 0.02)
  yBound = np.arange(pointy-0.1, pointy+0.1, 0.02)
  zBound = np.arange(pointz-0.1, pointz+0.1, 0.02)

  i = planNum
  x = []
  y = []
  z = []
  while (i != 0):
    print('i: ', i)
    px = np.random.choice(xBound, 1)[0]
    py = np.random.choice(yBound, 1)[0]
    pz = np.random.choice(zBound, 1)[0]
    x.append(px)
    y.append(py)
    z.append(pz)
    i -= 1

  return x, y, z

def view3D(new_x, new_y, new_z, newtext):
  # Helix equation
  # t = np.linspace(0, 10, 50)
  pointx = np.array((0.096))
  pointy = np.array((-0.25))
  pointz = np.array((0.95))
  # x, y, z = np.cos(t), np.sin(t), t

  all_fig = []
  fig1 = go.Scatter3d(
    x=np.array((0)),
    y=np.array((0)),
    z=np.array((0)),
    mode='markers',
    marker=dict(
      size=8,
      color='rgb(255,0,0)'
    )
  )
  all_fig.append(fig1)
  fig2 = go.Scatter3d(
    x=pointx,
    y=pointy,
    z=pointz,
    mode='markers',
    marker=dict(
      size=4,
      color='rgb(0,0,255)'
    ),
    text=['Cam233', 'Cam643', 'Cam241', 'Cam237', 'Cam218', 'Cam642']
  )
  all_fig.append(fig2)
  camx = np.array((1.2, 1.2, 1.2, -0.2, -0.2, 0.8))
  camy = np.array((0.14, -1.2, 0.14, -1.4, -1.4, -1.4))
  camz = np.array((1, 1, 0.1, 1.1, 0.1, 0.1))
  fig3 = go.Scatter3d(
    x=camx,
    y=camy,
    z=camz,
    mode='markers',
    marker=dict(
      size=6,
      color='rgb(0,255,0)'
    ),
    text=['Cam237', 'Cam241', 'Cam642', 'Cam643', 'Cam218', 'Cam233']
  )
  all_fig.append(fig3)

  # new_x, new_y, new_z = create_points()
  fig4 = go.Scatter3d(
    x=np.array(new_x),
    y=np.array(new_y),
    z=np.array(new_z),
    mode='markers',
    marker=dict(
      size=4,
      color='rgb(0,255,255)'
    ),
    text=list(newtext)
  )
  all_fig.append(fig4)

  final_layout = go.Figure(all_fig)

  final_layout.show()

  print("end")



def euler2T(translation, angles, extrinsic=True): 
    """
    Takes XYZ-euler angles/quaternion/rotation matrix and the translation and 
    returns a 4x4 homogeneous tranformation matrix. 
    """
    # list to np
    if(type(translation)==type([])):
        translation = np.array(translation, dtype="float") 
    if(type(angles)==type([])):
        angles = np.array(angles, dtype="float") 


    # init
    T = np.zeros((4,4), dtype="float") 
    T[3,3] = 1.0 
    # rotation 
    if(len(angles) == 4): # quaternion
        T[:3,:3] = ScyRot.from_quat(angles).as_matrix()
    elif(angles.shape == (3,3)): # matrix 
        T[:3,:3] = angles 
    elif(len(angles) == 3): # euler 
        eulertype = "xyz" if extrinsic==True else "XYZ" 
        T[:3,:3] = ScyRot.from_euler(eulertype, angles, degrees=True).as_matrix()
    else: 
        raise ValueError("Wrong input")
    # translatory 
    # (n, 1) -> (n,)
    if len(translation.shape) > 1:
        translation = translation.flatten() 
    T[:3,3] = translation

    return T 


def T_mm2m(T):
    T[:3,3] = T[:3,3]/1000.0
    return T

def T_m2mm(T): 
    T[:3,3] = T[:3,3]*1000.0
    return T 


def unit_transform(): 
    """
    Returns a unit homogeneous transformation that does 
    not change the rotation or the translation. 
    """
    T = np.zeros((4,4), dtype="float") 
    T[0,0] = 1.0
    T[1,1] = 1.0 
    T[2,2] = 1.0
    T[3,3] = 1.0 

    return T 

