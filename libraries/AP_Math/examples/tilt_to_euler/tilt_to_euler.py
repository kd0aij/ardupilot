# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 18:04:14 2020

@author: markw
"""
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
from matplotlib import cm
import matplotlib.pyplot as plt
import numpy as np
import csv

# read CSV file consisting of x axis followed by len(x) rows of curve data
data = []
with open('tilt_conversion_data.csv', 'rb') as csvfile:
    datareader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
    for row in datareader:
        data.append(np.asarray(row[:-1]))

# first row is x axis for plot
x = data[0];
X, Y = np.meshgrid(x, x);
Z = np.stack(data[1:102]);

fig = plt.figure()
ax = fig.gca(projection='3d')
surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
ax.set_xlabel('roll-in')
ax.set_ylabel('pitch-in')
ax.set_zlabel('roll-out')
# Add a color bar which maps values to colors.
fig.colorbar(surf, shrink=0.5, aspect=5)


plt.show()
