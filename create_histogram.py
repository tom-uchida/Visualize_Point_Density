import cv2, matplotlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
plt.style.use('seaborn-white')

from matplotlib import cycler
colors = cycler('color', ['#EE6666', '#3388BB', '#9988DD', '#EECC55', '#88BB44', '#FFBBBB'])
plt.rc('axes', facecolor='#E6E6E6', edgecolor='none', axisbelow=True, grid=False, prop_cycle=colors)
plt.rc('grid', color='w', linestyle='solid')
plt.rc('patch', edgecolor='#E6E6E6')
plt.rc('lines', linewidth=2)
plt.rcParams["mathtext.fontset"] = "cm"
plt.rcParams["mathtext.rm"] = "Times New Roman"

# Check arguments
import sys
args = sys.argv
if len(args) != 2:
    print("\nUSAGE : $ python create_histogram.py [*.csv]")
    sys.exit()

# Read csv file
point_densities = pd.read_csv( args[1], header=None )
point_densities = point_densities.values

print(point_densities[:10])

# Create figure
fig = plt.figure(figsize=(6, 6)) # figsize=(width, height)
gs  = gridspec.GridSpec(1,1)

# Histogram(Low image)
ax = fig.add_subplot(gs[0,0])
ax.hist(point_densities, bins=1000, color='black', alpha=1.0)
# ax.set_title("")
ax.set_xlabel('Number of points in the search sphere')
ax.set_ylabel('Freq')
# ax.set_xlim([-5, 260])
# ax.set_ylim([0, 20000])
# ax2.axvline(threshold_pixel_value, color='red')

plt.show()