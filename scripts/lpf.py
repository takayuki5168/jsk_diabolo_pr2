# average LPF
# average_num=10 is good!

import numpy as np
from matplotlib import pyplot as plt
import sys

if len(sys.argv) == 2:
    log_file = sys.argv[-1]
else:
    log_file = "../log/po.log"
    
pitch = []
yaw = []    
with open(log_file, 'r') as f:
    for l in f.readlines():
        w = l.split(' ')
        pitch.append(float(w[2]))
        yaw.append(float(w[3]))

lpf_average_nums = [1, 3, 5, 10]
lpf_pitch = [[] for i in range(len(lpf_average_nums))]
lpf_yaw = [[] for i in range(len(lpf_average_nums))]
for i in range(len(lpf_average_nums)):
    for j in range(len(pitch) - lpf_average_nums[i]):
        p = 0
        y = 0
        for k in range(lpf_average_nums[i]):
            p += pitch[j + k]
            y += yaw[j + k]
        lpf_pitch[i].append(p / lpf_average_nums[i])
        lpf_yaw[i].append(y / lpf_average_nums[i])

fig = plt.figure()
fig.patch.set_facecolor('white')
plt.subplots_adjust(wspace=0.4, hspace=0.6)

plt.subplot(2,1,1)
plt.title("Pitch")
plt.xlabel('step')
plt.ylabel('degree')
plt.plot(np.array([i for i in range(len(lpf_pitch[0]))]), np.array(lpf_pitch[0]), label="raw Pitch", linewidth=1)
plt.plot(np.array([i for i in range(len(lpf_pitch[3]))]), np.array(lpf_pitch[3]), label="smoothed Pitch", linewidth=1)
plt.legend()

plt.subplot(2,1,2)
plt.title("Yaw")
plt.xlabel('step')
plt.ylabel('degree')
plt.plot(np.array([i for i in range(len(lpf_yaw[0]))]), np.array(lpf_yaw[0]), label="raw Yaw", linewidth=1)
plt.plot(np.array([i for i in range(len(lpf_yaw[3]))]), np.array(lpf_yaw[3]), label="smoothed Yaw", linewidth=1)
plt.legend()

plt.show()
