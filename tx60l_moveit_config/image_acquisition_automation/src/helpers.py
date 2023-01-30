import os
import xlrd


def make_directory(path):
  isExist = os.path.exists(path)
  if not isExist:

    # Create a new directory because it does not exist
    os.makedirs(path)
    print("The new directory is created!")


def read_from_xlsx(row):
  # Open the Workbook
  workbook = xlrd.open_workbook('/home/raptor/tx60_moveit/src/tx60l_moveit_config/python_program/joint_values.xlsx')

  # Open the worksheet
  worksheet = workbook.sheet_by_index(0)
  joint = []
  for j in range(1, 7):
    joint.append(worksheet.cell_value(row, j))
  print('joint values of row ', row, ' : ', joint)
  return joint



