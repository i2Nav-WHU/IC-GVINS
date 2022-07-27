# -*- coding: utf-8 -*-
"""
@File    : show_tracking.py
@Author  : hailiang
@Contact : thl@whu.edu.cn
"""

import numpy as np
import matplotlib.pyplot as plt

log = np.loadtxt('path/tracking.txt')

plt.figure('interval')
plt.plot(log[:, 0], log[:, 1])
plt.grid()
plt.title('Average %0.2lf s' % np.average(log[:, 1]))
plt.tight_layout()

plt.figure('parallax')
plt.plot(log[:, 0], log[:, 2])
plt.grid()
plt.title('Average %0.2lf pixel' % np.average(log[:, 2]))
plt.tight_layout()

plt.figure('translation')
plt.plot(log[:, 0], log[:, 3])
plt.grid()
plt.title('Average %0.2lf m' % np.average(log[:, 3]))
plt.tight_layout()

plt.figure('rotation')
plt.plot(log[:, 0], log[:, 4])
plt.grid()
plt.title('Average %0.2lf deg' % np.average(log[:, 4]))
plt.tight_layout()

plt.figure('mappoint')
plt.plot(log[:, 0], log[:, 5])
plt.grid()
plt.title('Average %0.2lf' % np.average(log[:, 5]))
plt.tight_layout()

plt.figure('tiemcost')
plt.plot(log[:, 0], log[:, 6])
plt.grid()
plt.title('Average %0.2lf ms' % np.average(log[:, 6]))
plt.tight_layout()

plt.show()
