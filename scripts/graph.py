#!/usr/bin/env python
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

path_vel = input('csv velocity file path:')
df_vel = pd.read_csv(path_vel)
path_force_smooth = input('csv smooth force file path:')
df_force_smooth = pd.read_csv(path_force_smooth)
path_force_transform = input('csv transform force file path:')
df_force_transform = pd.read_csv(path_force_transform)

df_vel.set_index('time', inplace = True)
df_force_smooth.set_index('time', inplace = True)
df_force_transform.set_index('time', inplace = True)

fig = plt.figure(figsize=(15, 10))
plt.rcParams['font.family'] = "DejaVu Serif"
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams["font.size"] = 22

ax1 = fig.add_subplot(2, 1, 1)
df_force_smooth.plot.line(ax=ax1, style=['r', 'g'])
df_force_transform.plot.line(ax=ax1, style=['r:', 'g:'])
ax2 = fig.add_subplot(2, 1, 2)
df_vel.plot.line(ax=ax2, style=['r', 'g'])

ax1.set_xlim(0, 12)
ax2.set_xlim(0, 12)
ax1.set_ylabel('force')
ax2.set_ylabel('velocity')
ax2.set_xlabel('time / s')

ax1.set_xticks([0,2,4,6,8,10,12])
ax2.set_xticks([0,2,4,6,8,10,12])
ax1.label_outer()
ax2.label_outer()

# ax1.legend([r'$F_x$', r'$F_y$'], loc='upper left', bbox_to_anchor=(1, 1))
# ax2.legend([r'$v_x$', r'$v_y$'], loc='upper left', bbox_to_anchor=(1, 1))
ax1.legend([r'$F_x$', r'$F_y$'], loc='lower right')
ax2.legend([r'$v_x$', r'$v_y$'], loc='lower right')

ax1.yaxis.set_label_coords(-0.07, 0.5)
ax2.yaxis.set_label_coords(-0.07, 0.5)
fig.tight_layout()

plt.savefig('output.png')
plt.close('all')
