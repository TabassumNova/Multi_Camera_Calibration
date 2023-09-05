import numpy as np
from scipy import stats
from mayavi import mlab
import pandas as pd
import plotly.express as px

mu, sigma = 0, 0.1 
x = 10*np.random.normal(mu, sigma, 5000)
y = 10*np.random.normal(mu, sigma, 5000)
z = 10*np.random.normal(mu, sigma, 5000)

xyz = np.vstack([x,y,z])
kde = stats.gaussian_kde(xyz)
density = kde(xyz)

# # Plot scatter with mayavi
# figure = mlab.figure('DensityPlot')
# pts = mlab.points3d(x, y, z, density, scale_mode='none', scale_factor=0.07)
# mlab.axes()
# mlab.show()

data = {'x':x, 'y':y, 'z':z, 'density':density}
df = pd.DataFrame(data)
fig = px.scatter_3d(df, x='x', y='y', z='z',
              color='density')
fig.show()

