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
    all_fig.append(fig2)

    camx = np.array((0, 0.59, 0.22, 0.72, -0.06, 0.34))
    camy = np.array((0, 0.99, 1.13, 0.63, 1.13, 1.23))
    camz = np.array((0, 1.67, 0.62, 0.57, 1.81, 1.01))
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
    all_fig.append(fig4)

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
        print('i: ', i)
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