import matplotlib.pyplot as plt
import matplotlib as mpl
import plotly.graph_objects as go



# fig = plt.figure()
# ax = fig.add_axes([0.05, 0.80, 0.9, 0.05])
#
# colormap = plt.cm.get_cmap('viridis') # 'plasma' or 'viridis'
# norm = mpl.colors.Normalize(vmin=0, vmax=1)
# cb = mpl.colorbar.ColorbarBase(ax, orientation='horizontal', norm=norm, cmap=colormap)
#
# # plt.imshow()
# plt.savefig('colorbar', bbox_inches='tight')


#### using plotly
final_layout = go.Figure()
data_list = [go.Scatter3d(x=[None], y=[None], z=[None],
                                    mode='markers',
                                    marker=dict(
                                        # colorscale=red_blue,
                                        showscale=True,
                                        cmin=0,
                                        cmax=1,
                                        colorbar=dict(thickness=20, tickvals=[0.0,0.2,0.4,0.6,0.8, 1.0], tickfont=dict(size=18),
                                                      outlinewidth=0)
                                    ),
                                    hoverinfo='none'
                                    )]

fig1 = go.Figure(data=data_list)
fig1.update_layout(
    margin=dict(r=150),
    # ^^ making a bit of space for the annotation
    annotations=[
        dict(
            text="<b>Density</b>",
            font_size=20,
            # font_family='arial',
            # font_color='red',
            textangle=90,
            showarrow=False,
            # ^^ appearance
            xref="paper",
            yref="paper",
            x=1.08,
            y=0.5,
            # ^^ position
        )
    ]
)
fig1.show()