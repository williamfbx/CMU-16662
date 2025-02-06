#!/home/boxiangf/anaconda3/envs/16662_env/bin/python

import matplotlib.pyplot as plt
import numpy as np

data1 = np.loadtxt('force_vs_time_force_ctrl_sine_stable.csv', delimiter=',')
data2 = np.loadtxt('force_vs_time_impedance_ctrl_sine_stable.csv', delimiter=',')

time1 = data1[:, 0]
force1 = data1[:, 1]

time2 = data2[:, 0]
force2 = data2[:, 1]


def plotter(time1, force1, time2, force2):
    plt.figure(figsize=(10, 6))

    plt.plot(time1, force1, label="Force Controller", linewidth=1)
    
    plt.plot(time2, force2, label="Impedance Controller", linewidth=1, linestyle="--")

    plt.xlabel("Time (s)", fontsize=14)
    plt.ylabel("Force (N)", fontsize=14)
    plt.title("Force vs Time", fontsize=16)
    plt.ylim(0, 150)
    plt.grid(True, linestyle="--", alpha=0.7)
    plt.legend(fontsize=12)
    plt.tight_layout()
    plt.show()

plotter(time1, force1, time2, force2)