import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import plotly.graph_objects as go

board_map = np.load("/home/nova/Desktop/Nova/Calibration_paper/datasets/V35/BoardMap_M08320217_S08320218.npy")
num_board = board_map.shape[0]
board_dict = {}
x = []
y = []
text = []
for i in range(num_board):
    t = 'B-'+str(i+1)
    text.extend([t,t])
    x.extend([3,3.3])
    y.extend([num_board - i, num_board - i])
    # ts = 'SBoard-' + str(i + 1)
    # board_dict[i] = (3, num_board-i)
    # board_dict[i+num_board] = (5, num_board - i)

fig = go.Figure()
fig.add_trace(go.Scatter(x=x,
                  y=y,
                  mode='markers+text',
                  textfont={'color':'#ffffff'},
                  marker={'symbol': 'square', 'size': 30},
                  text=text)
              )
fig.add_trace(go.Scatter(x=[2.7, 3.6],
                  y=[9, 9],
                  mode='markers+text',
                  textfont={'color':'#ffffff'},
                  marker={'symbol': 'square', 'size': 30},
                  text=['C-1', 'C-2'])
              )
# fig.add_trace(go.Scatter(x=[1.06,3-.06], y=[12,9],
#             marker= dict(size=10,symbol= "arrow-bar-up", angleref="previous")))
for r in range(num_board):
    for c in range(num_board):
        if board_map[r][c]:
            fig.add_trace(go.Scatter(x=[2.71,3-.01], y=[9,r+1],
                        marker= dict(size=10,symbol= "arrow-bar-up", angleref="previous")))
            fig.add_trace(go.Scatter(x=[3+0.01, 3.3 - .01], y=[r+1, c+1],
                                     marker=dict(size=10, symbol="arrow-bar-up", angleref="previous")))

fig.show()
# iplot([data])

