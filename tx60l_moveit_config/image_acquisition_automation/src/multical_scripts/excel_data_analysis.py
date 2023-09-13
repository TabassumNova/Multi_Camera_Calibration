import pandas as pd
import matplotlib.pyplot as plt

xls = pd.ExcelFile(r"D:\MY_DRIVE_N\Masters_thesis\final report\Report_topics\Blender Images\final_img\analyze_all\analysis.xlsx") # use r before absolute file path

sheetX = xls.parse(1)

data = sheetX.values
std_list = []
mean_list = []
error_list = []
for row in range(data.shape[0]):
    if data[row][6]<10:
        std_list.append(data[row][3])
        mean_list.append(data[row][5])
        error_list.append(data[row][6])

plt.scatter(std_list, error_list)
plt.show()
pass