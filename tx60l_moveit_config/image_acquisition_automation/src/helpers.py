import os
import xlrd, xlsxwriter


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