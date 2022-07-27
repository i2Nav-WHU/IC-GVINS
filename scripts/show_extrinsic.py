# -*- coding: utf-8 -*-
"""
@File    : show_extrinsics.py
@Author  : hailiang
@Contact : thl@whu.edu.cn
"""

import numpy as np
import matplotlib.pyplot as plt

extrinsic = np.loadtxt('path/extrinsic.txt')

tx = extrinsic[:, 0]

trans = extrinsic[:, 1:4]
rota = extrinsic[:, 4:7]

plt.figure('translation')
plt.plot(tx, trans[:, 0], label='X')
plt.plot(tx, trans[:, 1], label='Y')
plt.plot(tx, trans[:, 2], label='Z')
plt.grid()
plt.legend()
plt.tight_layout()

plt.figure('rotation')
plt.plot(tx, rota[:, 0] - 90, label='X')
plt.plot(tx, rota[:, 1], label='Y')
plt.plot(tx, rota[:, 2] - 90, label='Z')
plt.grid()
plt.legend()
plt.tight_layout()

plt.figure('td')
plt.plot(tx, extrinsic[:, 7])
plt.grid()
plt.tight_layout()

plt.show()
