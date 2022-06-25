#!/usr/bin/env python
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

path = input('csv file path:')
df = pd.read_csv(path)
df.set_index('time', inplace = True)

plt.figure()
plt.rcParams['font.family'] = "DejaVu Serif"
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams["font.size"] = 18

axes = df[['force_x', 'vel_x']].plot.line(subplots=True, layout=(2, 1), style=['r','r'], figsize=(10, 8))
df[['force_y']].plot.line(ax=axes[0][0], style=['g'])
df[['vel_y']].plot.line(ax=axes[1][0], style=['g'])

axes[0][0].set_ylabel('force')
axes[1][0].set_ylabel("velocity")
axes[1][0].set_xlabel('time / s')
axes[0][0].set_yticks([])
axes[1][0].set_yticks([])

axes[0][0].legend([r'$F_x$', r'$F_y$'], loc='upper right')
axes[1][0].legend([r'$v_x$', r'$v_y$'], loc='upper right')

plt.savefig('output.png')
plt.close('all')
