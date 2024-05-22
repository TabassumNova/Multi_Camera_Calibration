import matplotlib.pyplot as plt
import matplotlib as mpl

fig = plt.figure()
ax = fig.add_axes([0.05, 0.80, 0.9, 0.1])

colormap = plt.cm.get_cmap('viridis') # 'plasma' or 'viridis'
cb = mpl.colorbar.ColorbarBase(ax, orientation='horizontal', cmap=colormap)

# plt.imshow()
plt.savefig('just_colorbar', bbox_inches='tight')