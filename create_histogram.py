import cv2, matplotlib
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
# plt.style.use('seaborn-white')
# plt.style.use('seaborn-whitegrid')
plt.style.use('bmh')

# from matplotlib import cycler
# colors = cycler('color', ['#EE6666', '#3388BB', '#9988DD', '#EECC55', '#88BB44', '#FFBBBB'])
# plt.rc('axes', facecolor='#E6E6E6', edgecolor='none', axisbelow=True, grid=False, prop_cycle=colors)
# plt.rc('grid', color='w', linestyle='solid')
# plt.rc('patch', edgecolor='#E6E6E6')
# plt.rc('lines', linewidth=2)
# plt.rcParams["mathtext.fontset"] = "cm"
# plt.rcParams["mathtext.rm"] = "Times New Roman"


# Check arguments
import sys
args = sys.argv
if len(args) != 2:
    print("\nUSAGE : $ python create_histogram.py [*.csv]")
    sys.exit()

# Read csv file
point_densities = pd.read_csv( args[1], header=None )
point_densities = point_densities.values
# print(point_densities[:10])

# To remove outlier
mean    = np.mean(point_densities)
# median  = np.median(point_densities)
# var     = np.var(point_densities)
std     = np.std(point_densities)
print("mean     :", mean)
# print("median   :", median)
# print("var      :", var)
print("std      :", std)

sigma_1  = mean + 1*std
sigma_2  = mean + 2*std
sigma_3  = mean + 3*std
print("1sigma   :", sigma_1)
print("2sigma   :", sigma_2)
print("3sigma   :", sigma_3)

# # Standardize after removing outliers
# b_outlier = point_densities > sigma_1
# point_densities = np.where(b_outlier, sigma_1, point_densities)
# mean = np.mean(point_densities)
# std  = np.std(point_densities)
# print("\nmean     :", mean)
# print("std      :", std)

# Standardization
# z = (point_densities - mean) / std
# # print("z        :", z)
# mean_z  = np.mean(z)
# var_z   = np.var(z)
# print("\nmean_z   :", mean_z)
# print("var_z    :", var_z)



# Create figure
fig = plt.figure(figsize=(8, 6)) # figsize=(width, height)
gs  = gridspec.GridSpec(1,1)

# Histogram(Low image)
ax = fig.add_subplot(gs[0,0])
ax.hist(point_densities, bins=1000, color='black', alpha=1.0)
# ax.hist(z, bins=100, color='black', alpha=1.0)
# ax.set_title("After standardization")
ax.set_xlabel('Number of points in the search sphere')
ax.set_ylabel('Freq')
# ax.set_xlim([0, 1])
ax.set_ylim([0, 50000])
ax.axvline(sigma_1, color='red', label="$1\sigma$")
ax.axvline(sigma_2, color='blue', label="$2\sigma$")
ax.axvline(sigma_3, color='green', label="$3\sigma$")

plt.legend(fontsize=16)
plt.show()