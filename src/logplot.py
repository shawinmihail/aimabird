#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

path = "../logs/data.log"
df = pd.read_csv(path)

f, axarr = plt.subplots(2, sharex=True)
f.suptitle('data')

axarr[0].plot(df["qPx.w"], 'k')
axarr[0].plot(df["qPx.x"], 'r')
axarr[0].plot(df["qPx.y"], 'g')
axarr[0].plot(df["qPx.z"], 'b')

# axarr[0].plot(df.qGz.w, '--k')
# axarr[0].plot(df.qGz.x, '--r')
# axarr[0].plot(df.qGz.y, '--g')
# axarr[0].plot(df.qGz.z, '--b')

axarr[0].grid()

plt.show()


f, axarr = plt.subplots(2, sharex=True)
f.suptitle('data')

axarr[0].plot(df.rxPx, 'r')
axarr[0].plot(df.ryPx, 'g')
axarr[0].plot(df.rzPx, 'b')

axarr[0].plot(df.rxGz, '--r')
axarr[0].plot(df.ryGz, '--g')
axarr[0].plot(df.rzGz, '--b')

axarr[0].grid()

plt.show()