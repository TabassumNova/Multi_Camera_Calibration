import plotly.graph_objs as go

fig = go.Figure()
fig.add_trace(go.Box(y=[900, 350, 650, 750, 350, 1350, 250, 1000, 850, 690, 720]))
fig.layout.update(
    updatemenus=[
        go.layout.Updatemenu(
            type="buttons", direction="left", buttons=list(
                [
                    dict(args=["type", "box"], label="Box", method="restyle"),
                    dict(args=["type", "violin"], label="Violin", method="restyle")
                ]
            ),
            pad={"r": 2, "t": 2},
            showactive=True,
            x=0.11,
            xanchor="left",
            y=1.1,
            yanchor="top"
        ),
    ]
)

fig.show()