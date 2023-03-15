import plotly.graph_objects as go
import numpy as np

def view3D():
    # Helix equation
    # t = np.linspace(0, 10, 50)
    pointx = np.array((-0.02, 0.11, 0.11, 0, 0.096, -0.12))
    pointy = np.array((-0.019,- 0.31, -0.17, -0.3, -0.25, -0.25))
    pointz = np.array((1.09, 0.94, 0.98, 0.95, 0.95, 0.78))
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
        text= ['Cam233', 'Cam643', 'Cam241', 'Cam237', 'Cam218', 'Cam642']
    )
    all_fig.append(fig2)
    camx = np.array((1.2, 1, 1.2, 1.2, 1.2, -0.1))
    camy = np.array((0.14, -1.4, .14, -1.2, 0.14, -1.4))
    camz = np.array((0.62, 0.67, 1, 1, 0.1, 0.67))
    fig3 = go.Scatter3d(
        x=camx,
        y=camy,
        z=camz,
        mode='markers',
        marker=dict(
            size=6,
            color='rgb(0,255,0)'  
        ),
        text= ['Cam218', 'Cam233', 'Cam237', 'Cam241', 'Cam642', 'Cam643']
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

def create_points():
    xBound = np.arange(-0.15, 0.11, 0.02)
    yBound = np.arange(-0.31, 0, 0.02)
    zBound = np.arange(0.75, 1.1, 0.02)

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