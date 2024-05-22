import matplotlib.pyplot as plt
import matplotlib as mpl

fig = plt.figure()
ax = fig.add_axes([0.05, 0.80, 0.9, 0.05])

colormap = plt.cm.get_cmap('viridis') # 'plasma' or 'viridis'
norm = mpl.colors.Normalize(vmin=0, vmax=2)
cb = mpl.colorbar.ColorbarBase(ax, orientation='horizontal', norm=norm, cmap=colormap)

# plt.imshow()
plt.savefig('colorbar', bbox_inches='tight')