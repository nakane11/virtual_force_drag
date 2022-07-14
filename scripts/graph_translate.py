#!/usr/bin/env python

import argparse

import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt


parser = argparse.ArgumentParser(description='rosbag to video')
parser.add_argument('--out', '-o', default='output.png',
                    help='output directory path or filename. '
                    'If more than one --image-topic are specified, '
                    'this will be interpreted as a directory name. '
                    'Otherwise this is the file name.')
parser.add_argument('--input-velocity', '-v', required=True)
# parser.add_argument('--input-transform-force', '--tf', required=True)
parser.add_argument('--input-smooth-force', '--sf', required=True)
args = parser.parse_args()

path_vel = args.input_velocity
df_vel = pd.read_csv(path_vel)
df_vel.set_index('time', inplace = True)

path_force_smooth = args.input_smooth_force
df_force_smooth = pd.read_csv(path_force_smooth)
df_force_smooth.set_index('time', inplace = True)

# path_force_transform = args.input_transform_force
# df_force_transform = pd.read_csv(path_force_transform)
# df_force_transform.set_index('time', inplace = True)

fig = plt.figure(figsize=(14, 7))
plt.rcParams['font.family'] = "DejaVu Serif"
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams["font.size"] = 22

grid = plt.GridSpec(3, 1,hspace=0.2)
ax1 = fig.add_subplot(grid[0:2,0])
df_force_smooth.plot.line(ax=ax1, lw=2,color=['r','g'])
# df_force_transform.plot.line(ax=ax1, lw=1.8, color=[(205/255.0,110/255.0,110/255.0),(110/255.0,205/255.0,110/255.0)], style=':')
ax2 = fig.add_subplot(grid[2,0])
df_vel.plot.line(ax=ax2, lw=2,color=['r', 'g'])

tmp = ax1.axhline(y=-2.5,color='r',linestyle=':')
ax1.axhline(y=2.5,color='r',linestyle=':')
ax1.axhline(y=-2.0,color='g',linestyle=':')
ax1.axhline(y=1.6,color='g',linestyle=':')

ax1.set_xlim(0, 12)
ax1.set_ylim(-4, 4)
ax2.set_xlim(0, 12)
ax1.set_ylabel('force [N]')
ax2.set_ylabel('velocity [m/s]')
ax2.set_xlabel(r'time [s]')

ax1.set_xticks([0,2,4,6,8,10,12])
ax2.set_xticks([0,2,4,6,8,10,12])
ax1.label_outer()
ax2.label_outer()

ax1.legend([r'$F_x$', r'$F_y$'], loc='upper right',fontsize='24')
ax2.legend([r'$v_x$', r'$v_y$'], loc='upper right',fontsize='24')
ax1.text(x=0.7, y=2.7, s=r'$T_{up x}$', color='black', ha='center', fontsize='22')
ax1.text(x=0.7, y=1.8, s=r'$T_{up y}$', color='black', ha='center', fontsize='22')
ax1.text(x=0.7, y=-2.4, s=r'$T_{low x}$', color='black', ha='center', fontsize='22')
ax1.text(x=0.7, y=-1.8, s=r'$T_{low y}$', color='black', ha='center', fontsize='22')

ax1.yaxis.set_label_coords(-0.07, 0.5)
ax2.yaxis.set_label_coords(-0.07, 0.5)
# fig.tight_layout()

plt.savefig(args.out)
plt.close('all')
