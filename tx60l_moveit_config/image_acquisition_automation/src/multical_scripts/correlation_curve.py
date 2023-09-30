import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def main1():

    # fig, ax = plt.subplots()
    fig, axs = plt.subplots(1, 3)

    obj = ['V24(cube)', 'V30(cube)', 'V35(ico.)', 'V43(ico.)']
    counts1 = [7, 19.8, 41.2, 33.2]
    bar_labels = ['V24(cube)', 'V30(cube)', 'V35(ico.)', 'V43(ico.)']
    bar_colors = ['tab:blue', 'tab:blue', 'tab:green', 'tab:green']

    axs[0].bar(obj, counts1, label=bar_labels, color=bar_colors, width=0.5)
    fig.set_figwidth(20)
    fig.set_figheight(8)

    axs[0].set_ylabel('Number of views', fontsize="15",)
    # fig.xticks(fontsize=30)
    axs[0].set_xticklabels(bar_labels, fontsize=12)
    axs[0].set_title('Number of Inlier views \n (Re-projection error < 1.0) per camera', fontsize="15",)
    # axs[0].legend(title='Calibration object', fontsize="12")

    counts2 = [16.9, 16.41, 21.41, 19.98]
    axs[1].bar(obj, counts2, label=bar_labels, color=bar_colors, width=0.5)
    fig.set_figwidth(20)
    fig.set_figheight(8)
    axs[1].set_ylabel('Standard deviation of view angle (Degrees)', fontsize="15",)
    # fig.xticks(fontsize=30)
    axs[1].set_xticklabels(bar_labels, fontsize=12)
    axs[1].set_title('Standard deviation \n of inlier poses view angle per camera', fontsize="15",)
    # axs[0].legend(title='Calibration object', fontsize="12")

    counts3 = [49.8, 54.09, 78.78, 80.55]
    axs[2].bar(obj, counts2, label=bar_labels, color=bar_colors, width=0.5)
    fig.set_figwidth(20)
    fig.set_figheight(8)
    axs[2].set_ylabel('Range of view angle (Degrees)', fontsize="15",)
    # fig.xticks(fontsize=30)
    axs[2].set_xticklabels(bar_labels, fontsize=12)
    axs[2].set_title('Variation of inlier poses \n view angle per camera', fontsize="15",)
    # axs[0].legend(title='Calibration object', fontsize="12")

    plt.show()



def main2():
    dataframe1 = pd.read_excel("D:\MY_DRIVE_N\Masters_thesis/final_report\Report_topics\Blender_Images/final_img/final_analysis2.xlsx")

    values = dataframe1.values
    dataset = {}
    dataset['V24(cube)'] = {}
    dataset['V30(cube)'] = {}
    dataset['V35(Icosahedron)'] = {}
    dataset['V43(Icosahedron)'] = {}

    for x in values:
        name = x[0]
        if name == 'V24(cube)' or name == 'V30(cube)' or name == 'V35(Icosahedron)' or name == 'V43(Icosahedron)':
            if 'num' not in dataset[name]:
                dataset[name]['num'] = []
                dataset[name]['std'] = []
                dataset[name]['range'] = []
            dataset[name]['num'].append(x[6])
            dataset[name]['std'].append(x[10])
            dataset[name]['range'].append(x[12])
        pass
    # print(dataframe1)

    # create data
    fig, axs = plt.subplots(1, 3)

    ## num inliers
    x = np.arange(6)
    y1 = dataset['V24(cube)']['num']
    y2 = dataset['V30(cube)']['num']
    y3 = dataset['V35(Icosahedron)']['num']
    y4 = dataset['V43(Icosahedron)']['num']
    width = 0.2
    fig.set_figwidth(25)
    fig.set_figheight(8)
    # plot data in grouped manner of bar type
    axs[0].bar(x - 0.2, y1, width, color='red')
    axs[0].bar(x, y2, width, color='orange')
    axs[0].bar(x + 0.2, y3, width, color='blue')
    axs[0].bar(x + 0.4, y4, width, color='navy')
    axs[0].set_xticklabels(['Cam-217', 'Cam-217', 'Cam-218', 'Cam-220', 'Cam-221', 'Cam-222', 'Cam-113'])
    axs[0].set_xlabel("Cameras")
    axs[0].set_ylabel("Number of inlier views")
    axs[0].legend(["V24(cube)", "V30(cube)", "V35(Ico.)", "V43(Ico.)"])
    axs[0].set_title('Number of Inlier views (Re-projection error < 1.0)\n per camera', fontsize="15",)

    ## std
    x = np.arange(6)
    y1 = dataset['V24(cube)']['std']
    y2 = dataset['V30(cube)']['std']
    y3 = dataset['V35(Icosahedron)']['std']
    y4 = dataset['V43(Icosahedron)']['std']
    width = 0.2
    fig.set_figwidth(25)
    fig.set_figheight(8)
    # plot data in grouped manner of bar type
    axs[1].bar(x - 0.2, y1, width, color='red')
    axs[1].bar(x, y2, width, color='orange')
    axs[1].bar(x + 0.2, y3, width, color='blue')
    axs[1].bar(x + 0.4, y4, width, color='navy')
    axs[1].set_xticklabels(['Cam-217','Cam-217',  'Cam-218', 'Cam-220', 'Cam-221', 'Cam-222', 'Cam-113'])
    axs[1].set_xlabel("Datasets")
    axs[1].set_ylabel('Standard deviation of view angle (Degrees)', fontsize="15")
    axs[1].legend(["V24(cube)", "V30(cube)", "V35(Ico.)", "V43(Ico.)"])
    axs[1].set_title('Standard deviation of inlier poses (Re-projection error < 1.0)\n view angle per camera', fontsize="15")

    ## range
    x = np.arange(6)
    y1 = dataset['V24(cube)']['range']
    y2 = dataset['V30(cube)']['range']
    y3 = dataset['V35(Icosahedron)']['range']
    y4 = dataset['V43(Icosahedron)']['range']
    width = 0.2
    fig.set_figwidth(25)
    fig.set_figheight(8)
    # plot data in grouped manner of bar type
    axs[2].bar(x - 0.2, y1, width, color='red')
    axs[2].bar(x, y2, width, color='orange')
    axs[2].bar(x + 0.2, y3, width, color='blue')
    axs[2].bar(x + 0.4, y4, width, color='navy')
    axs[2].set_xticklabels(['Cam-217','Cam-217',  'Cam-218', 'Cam-220', 'Cam-221', 'Cam-222', 'Cam-113'])
    axs[2].set_xlabel("Cameras")
    axs[2].set_ylabel('Range of view angle (Degrees)', fontsize="15",)
    axs[2].legend(["V24(cube)", "V30(cube)", "V35(Ico.)", "V43(Ico.)"])
    axs[2].set_title('Variation of inlier poses (Re-projection error < 1.0) \n view angle per camera', fontsize="15",)
    plt.show()

def main3():
    dataframe1 = pd.read_excel(
        "D:\MY_DRIVE_N\Masters_thesis/final_report\FAPS_LaTeX_Template_English_v2022-05-16\Bilder\Blender_Images/final_img/analyze_all/analysis2.xlsx")

    values = dataframe1.values

    error = values[0:6,11]
    range_std_mean = values[0:6, 13]
    x = ['08320217' , '08320218', '08320220', '08320221', '08320222', '36220113']
    plt.plot(x, error, marker ='o', label="%Re-projection error")
    # plt.xlim(2, 4)
    # plt.ylim(-1, 1)
    # plt.show()
    plt.grid()
    plt.xlabel("Cameras")
    # plt.ylabel("Calorie Burnage")
    plt.plot(x, range_std_mean, marker ='o', label="View Angle Variation")
    # plt.xlim(2, 4)
    # plt.ylim(-1, 1)
    plt.legend(loc="upper left")
    plt.show()

    pass

def main4():
    x = ['V24', 'V30', 'V35', 'V43']
    error = [0.54, 0.17, 0.12, 0.16]
    angle_variation = [0.08, 0.23, 0.36, 0.33]
    plt.plot(x, error, marker='o', label="%Validation Re-projection error")
    plt.grid()
    plt.xlabel("Datasets")
    plt.ylabel("Percentages")
    plt.plot(x, angle_variation, marker='o', label="%Inlier View Variation")

    plt.legend(loc="upper right")
    plt.show()
    pass

if __name__ == '__main__':
    main4()
    pass