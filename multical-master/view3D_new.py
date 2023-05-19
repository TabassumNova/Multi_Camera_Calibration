import plotly.graph_objects as go
import numpy as np

def view3D():
    # Helix equation
    # t = np.linspace(0, 10, 50)
    pointx = np.array((0.096, 2, -2))
    pointy = np.array((-0.25, 2, -2))
    pointz = np.array((0.95, 2, -2))
    # x, y, z = np.cos(t), np.sin(t), t

    all_fig = []
    # fig1 = go.Scatter3d(
    #     x=np.array((0)),
    #     y=np.array((0)),
    #     z=np.array((0)),
    #     mode='markers',
    #     marker=dict(
    #         size=8,
    #         color='rgb(255,0,0)'
    #     )
    # )
    # all_fig.append(fig1)
    fig2 = go.Scatter3d(
        x=pointx,
        y=pointy,
        z=pointz,
        mode='markers',
        marker=dict(
            size=4,
            color='rgb(0,0,255)'  
        )
    )
    # all_fig.append(fig2)

    cam217 = [
        0.0,
        0.0,
        0.0
        # 23.3, 8.24, 998.3
      ]
    cam218 = [
        0.95, 0.47, 0.09
        # 0,0,0
        # -0.89, -0.12, 0.71
        # 0.81, 0.39, 0.043
        # 3.3794577823826923,
        # 0.9690751872978102,
        # 6.596593801548761
        # 0.9569066659280587,
        # 0.46877755769072393,
        # 0.08966121101677928
        # -239.37, -7.35, 1421.53
      ]
    cam220 = [
        1.06, 0.79, 1.79
        # 0,0,0
        # -0.68, 0.77, 2.02
        # 0.9, 0.68, 1.48
        # 10.364688731582001,
        # 2.8528211434391215,
        # 1.3599010481129343
        # 1.0645776109639138,
        # 0.7969159850696835,
        # 1.7956446937907211
        # -104.18, 82.91, 1547.15
      ]
    cam221 = [
        1.05, -0.11, 1.84
        # 0,0,0
        # -0.8, 0.25, 2.18
        # 0.89, -0.09, 1.52
        # 4.405003752702106,
        # -3.0574558188932186,
        # 5.33891211033096
        # 1.0571976157451903,
        # -0.11240637340093819,
        # 1.8400679990944397
        # -45.84, 44.18, 1347.24
      ]
    cam222 = [
        -0.27, -0.43, 0
        # 0,0,0
        # 0.16, 0.44, 0.18
        # -0.23, -0.36, -0.03
        # -0.03283577055158265,
        # -0.5035229359581769,
        # -0.4924782735496166
        # - 0.27747623592116577,
        # -0.42647551289056657,
        # 0.003887695271044547
        # -29.63, 115.463, 1185.59
      ]
    cam113 = [
        1.18,-0.24,0.89
        # -1.21, -0.13, 0.99
        # 1, -0.2, 0.72
        # 6.947918493978645,
        # -3.548271503473395,
        # 5.164481136951818
        # 1.1853238145214062,
        # -0.24161026725382237,
        # 0.8911889696422884
        # 23.3, 8.24, 998.3
      ]

    camx = np.array((cam217[0], cam218[0], cam220[0], cam221[0], cam222[0], cam113[0]))
    camy = np.array((cam217[1], cam218[1], cam220[1], cam221[1], cam222[1], cam113[1]))
    camz = np.array((cam217[2], cam218[2], cam220[2], cam221[2], cam222[2], cam113[2]))


    fig3 = go.Scatter3d(
        x=camx,
        y=camy,
        z=camz,
        mode='markers',
        marker=dict(
            size=6,
            color='rgb(0,255,0)'  
        ),
        text= ['Cam217', 'Cam218', 'Cam220', 'Cam221', 'Cam222', 'Cam113']
    )
    all_fig.append(fig3)

    ## new
    cam217 = [
        0.0,
        0.0,
        0.0
    ]
    cam218 = [
        0.95, 0.47, 0.09
    ]
    cam220 = [
        # 1.06, 0.79, 1.79
        0.92, 1.44, 1.46
    ]
    cam221 = [
        # 1.05, -0.11, 1.84
        1.26, 0.56, 1.95
    ]
    cam222 = [
        # -0.27, -0.43, 0
        -0.47, -0.61, 0.11
    ]
    cam113 = [
        # 1.18,-0.24,0.89
        1.47, -0.28, 1.12
    ]

    camx = np.array((cam217[0], cam218[0], cam220[0], cam221[0], cam222[0], cam113[0]))
    camy = np.array((cam217[1], cam218[1], cam220[1], cam221[1], cam222[1], cam113[1]))
    camz = np.array((cam217[2], cam218[2], cam220[2], cam221[2], cam222[2], cam113[2]))

    fig4 = go.Scatter3d(
        x=camx,
        y=camy,
        z=camz,
        mode='markers',
        marker=dict(
            size=6,
            color='rgb(0,0,255)'
        ),
        text=['Cam217', 'Cam218', 'Cam220', 'Cam221', 'Cam222', 'Cam113']
    )
    all_fig.append(fig4)

    cam217 = [
        0.0,
        0.0,
        0.0
    ]
    cam218 = [
        # 0.95, 0.47, 0.09
        0.7, -0.2, 0.24
    ]
    cam220 = [
        1.06, 0.79, 1.79
        # 0.92, 1.44, 1.46
    ]
    cam221 = [
        # 1.05, -0.11, 1.84
        # 1.26, 0.56, 1.95
        1.21, 0.58, 1.38
    ]
    cam222 = [
        # -0.27, -0.43, 0
        # -0.47, -0.61, 0.11
        -0.58, 0.19, 0.20
    ]
    cam113 = [
        # 1.18,-0.24,0.89
        # 1.47, -0.28, 1.12
        1.11, -0.35, 0.45
    ]

    camx = np.array((cam217[0], cam218[0], cam220[0], cam221[0], cam222[0], cam113[0]))
    camy = np.array((cam217[1], cam218[1], cam220[1], cam221[1], cam222[1], cam113[1]))
    camz = np.array((cam217[2], cam218[2], cam220[2], cam221[2], cam222[2], cam113[2]))

    fig5 = go.Scatter3d(
        x=camx,
        y=camy,
        z=camz,
        mode='markers',
        marker=dict(
            size=6,
            color='rgb(255,0,0)'
        ),
        text=['Cam217', 'Cam218', 'Cam220', 'Cam221', 'Cam222', 'Cam113']
    )
    all_fig.append(fig5)

    cam217 = [
        0.0,
        0.0,
        0.0
    ]
    cam218 = [
        0.0,
        0.0,
        0.0
    ]
    cam220 = [
        # 1.06, 0.79, 1.79
        - 189.772,
        944.483,
        - 69.393
    ]
    cam221 = [
        # 1.05, -0.11, 1.84
        -664.74,
        757.84,
        772.07
    ]
    cam222 = [
        # -0.27, -0.43, 0
        0,0,0
    ]
    cam113 = [
        # 1.18,-0.24,0.89
        72.543,
        248.681,
        891.033
    ]

    camx = np.array((cam217[0], cam218[0], cam220[0], cam221[0], cam222[0], cam113[0]))
    camy = np.array((cam217[1], cam218[1], cam220[1], cam221[1], cam222[1], cam113[1]))
    camz = np.array((cam217[2], cam218[2], cam220[2], cam221[2], cam222[2], cam113[2]))

    fig5 = go.Scatter3d(
        x=camx,
        y=camy,
        z=camz,
        mode='markers',
        marker=dict(
            size=6,
            color='rgb(0,0,255)'
        ),
        text=['Cam217', 'Cam218', 'Cam220', 'Cam221', 'Cam222', 'Cam113']
    )
    all_fig.append(fig5)
    ## new

    edge_x = [camx[0],camx[4], None, camx[2], camx[3], None, camx[2], camx[1], None, camx[3], camx[5], None]
    edge_y = [camy[0],camy[4], None, camy[2], camy[3], None, camy[2], camy[1], None, camy[3], camy[5], None]
    edge_z = [camz[0],camz[4], None, camz[2], camz[3], None, camz[2], camz[1], None, camz[3], camz[5], None]

    trace_edges = go.Scatter3d(x=edge_x,
                               y=edge_y,
                               z=edge_z,
                               mode='lines',
                               line=dict(color='red', width=5),
                               hoverinfo='text')
    all_fig.append(trace_edges)

    new_x, new_y, new_z = create_points()
    fig4 = go.Scatter3d(
        x=np.array(new_x),
        y=np.array(new_y),
        z=np.array(new_z),
        mode='markers',
        marker=dict(
            size=4,
            color='rgb(0,255,255)'  
        )
    )
    # all_fig.append(fig4)

    final_layout = go.Figure(all_fig)

    final_layout.show()

    print("end")

def create_points():
    xBound = np.arange(0, 0.2, 0.02)
    yBound = np.arange(-0.3, -0.1, 0.02)
    zBound = np.arange(0.8, 1, 0.02)

    i = 50
    x = []
    y = []
    z = []
    while (i != 0):
        # print('i: ', i)
        px = np.random.choice(xBound, 1)[0]
        py = np.random.choice(yBound, 1)[0]
        pz = np.random.choice(zBound, 1)[0]
        x.append(px)
        y.append(py)
        z.append(pz)
        i -= 1

    return x, y, z


if __name__ == '__main__':
    view3D()