#!/usr/bin/env python
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

path_vel = input('csv velocity file path:')
df_vel = pd.read_csv(path_vel)
path_angle = input('csv angle file path:')
df_angle = pd.read_csv(path_angle)

df_vel.set_index('time', inplace = True)
df_angle.set_index('time', inplace = True)

fig = plt.figure(figsize=(15, 10))
plt.rcParams['font.family'] = "DejaVu Serif"
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams["font.size"] = 22

ax1 = fig.add_subplot(2, 1, 1)
df_angle.plot.line(ax=ax1)

ax2 = fig.add_subplot(2, 1, 2)
df_vel.plot.line(ax=ax2)

# ax1.axhline(y=-2.5,color='r',linestyle=':')
# ax1.axhline(y=2.5,color='r',linestyle=':')
# ax1.axhline(y=-2.0,color='g',linestyle=':')
# ax1.axhline(y=1.6,color='g',linestyle=':')



ax1.set_xlim(0, 10)
ax2.set_xlim(0, 10)
ax1.set_ylabel('joint angle')
ax2.set_ylabel('velocity rad/s')
ax2.set_xlabel(r'time /s')

# ax1.set_xticks([0,2,4,6,8,10,12])
# ax2.set_xticks([0,2,4,6,8,10,12])
ax1.label_outer()
ax2.label_outer()

# ax1.legend([r'$F_x$', r'$F_y$'], loc='upper left', bbox_to_anchor=(1, 1))
# ax2.legend([r'$v_x$', r'$v_y$'], loc='upper left', bbox_to_anchor=(1, 1))
# ax1.legend([r'$F_x$', r'$F_y$'], loc='lower right')
# ax2.legend([r'$v_x$', r'$v_y$'], loc='lower right')
ax1.get_legend().set_visible(False)
ax2.get_legend().set_visible(False)
ax1.yaxis.set_label_coords(-0.07, 0.5)
ax2.yaxis.set_label_coords(-0.07, 0.5)
fig.tight_layout()

plt.savefig('rotate.png')
plt.close('all')
