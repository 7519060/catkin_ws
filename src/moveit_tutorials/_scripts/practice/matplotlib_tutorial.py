#!/usr/bin/env python
#coding: UTF-8
import numpy as np
import matplotlib.pyplot as plt

x1 = np.arange(-3, 3, 0.1)
x2 = np.arange(-3, 5, 0.1)
y1 = np.sin(x1)
y2 = x2
y3 = x2 ** 2
y4 = x1 ** 3

# fig, ax = plt.subplots(2,2)
# # fig, ax = plt.subplots(2, 2, sharex=True, sharey=False)

# ax[0,0].plot(x1, y1)
# ax[0,1].plot(x2, y2)
# ax[1,0].plot(x2, y3)
# ax[1,1].plot(x1, y4)
# plt.show()

fig = plt.figure(figsize = (10,10), facecolor = 'lightblue') ###tate yoko nojun
ax1 = fig.add_subplot(3,2,1)
ax2 = fig.add_subplot(3,2,2)
ax3 = fig.add_subplot(3,2,3)
ax4 = fig.add_subplot(3,2,4)
ax5 = fig.add_subplot(3,2,5)
ax6 = fig.add_subplot(3,2,6)

ax1.plot(x1, y1)
ax2.plot(x2, y2)
ax3.plot(x2, y3)
ax4.plot(x1, y4)
ax5.plot(x2, y3)
ax6.plot(x1, y4)
plt.tight_layout()
plt.show()
