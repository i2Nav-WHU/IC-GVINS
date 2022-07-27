# -*- coding: utf-8 -*-
"""
@File    : show_statistics.py
@Author  : hailiang
@Contact : thl@whu.edu.cn
"""

import numpy as np
import matplotlib.pyplot as plt

stat = np.loadtxt('path/statistics.txt')

stamp = stat[:, 0]

fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

ax1.plot(stamp, stat[:, 1], color='#d62728', label='Length [s]')
ax2.plot(stamp, stat[:, 2], color='#2ca02c', label='Counts')
ax1.set_xlabel('Time [s]')
ax1.grid()
plt.title('Average length %0.2lfs, counts %0.1lf' % (np.average(stat[:, 1]), np.average(stat[:, 2])))
ax1.legend(loc='upper left')
ax2.legend(loc='upper right')
plt.tight_layout()

plt.figure('feature')
plt.plot(stamp, stat[:, 3])
plt.grid()
plt.title('Average feature %0.1lf' % np.average(stat[:, 3]))
plt.xlabel('Time [s]')
plt.ylabel('Counts')
plt.tight_layout()

plt.figure('reprojection')
min_error = np.min(stat[:, 4])
max_error = np.max(stat[:, 5])
mean_error = np.mean(stat[:, 6])
rms_error = np.sqrt(np.average(stat[:, 7]**2))
plt.plot(stamp, stat[:, 4], label='MIN %0.3lf' % min_error)
plt.plot(stamp, stat[:, 5], label='MAX %0.3lf' % max_error)
plt.plot(stamp, stat[:, 6], label='MEAN %0.3lf' % mean_error)
plt.plot(stamp, stat[:, 7], label='RMS %0.3lf' % rms_error)

plt.grid()
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Error [pixel]')

plt.figure('iteratios')
plt.plot(stamp, stat[:, 8], label='#1 %0.1lf' % np.average(stat[:, 8]))
plt.plot(stamp, stat[:, 9], label='#2 %0.1lf' % np.average(stat[:, 9]))
plt.grid()
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Iterations')
plt.tight_layout()

plt.figure('timecost')

cost1 = np.average(stat[:, 10])
cost2 = np.average(stat[:, 11])
cost3 = np.average(stat[:, 12])
plt.plot(stamp, stat[:, 10], label='Optimization #1 %0.1lf ms' % cost1)
plt.plot(stamp, stat[:, 11], label='Optimization #2 %0.1lf ms' % cost2)
plt.plot(stamp, stat[:, 12], label='Marginalization %0.1lf ms' % cost3)
plt.title("Optimization cost %0.1lf ms" % (cost1 + cost2 + cost3))
plt.grid()
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Costs [ms]')
plt.tight_layout()

plt.figure('outlier')
plt.plot(stamp, stat[:, 13], label='Mappoints #1 %0.1lf' % np.average(stat[:, 13]))
plt.plot(stamp, stat[:, 14], label='Features #1 %0.1lf' % np.average(stat[:, 14]))
plt.grid()
plt.legend()
plt.xlabel('Time [s]')
plt.ylabel('Counts')
plt.tight_layout()

plt.show()
