import numpy as np
import matplotlib.pyplot as plt

folder_name = "results/"


# loading Data
rotation = np.load("results/17_12_2019_21_44_39/rotation.npy")
setpoints = np.load("results/17_12_2019_21_44_39/setpoints.npy")
time = np.load("results/17_12_2019_21_44_39/time.npy")
translation = np.load("results/17_12_2019_21_44_39/translation.npy")
# wind=np.load('example-results/wind.npy')
pid = np.load("results/17_12_2019_21_44_39/pid.npy")

# subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
fig.suptitle("Rotational Motion")
fig, (ax4, ax5, ax6) = plt.subplots(3, 1)
fig.suptitle("Translational Motion")
# fig,(ax7,ax8,ax9)=plt.subplots(3,1)
# fig.suptitle("PID")

# label of the axes
ax1.set(ylabel="Roll")
ax2.set(ylabel="Pitch")
ax3.set(ylabel="Yaw")


ax1.spines["top"].set_visible(False)
ax1.spines["right"].set_visible(False)
ax2.spines["top"].set_visible(False)
ax2.spines["right"].set_visible(False)
ax3.spines["top"].set_visible(False)
ax3.spines["right"].set_visible(False)
ax4.spines["top"].set_visible(False)
ax4.spines["right"].set_visible(False)
ax5.spines["top"].set_visible(False)
ax5.spines["right"].set_visible(False)
ax6.spines["top"].set_visible(False)
ax6.spines["right"].set_visible(False)
# ax7.spines['top'].set_visible(False)
# ax7.spines['right'].set_visible(False)
# ax8.spines['top'].set_visible(False)
# ax8.spines['right'].set_visible(False)
# ax9.spines['top'].set_visible(False)
# ax9.spines['right'].set_visible(False)

# plotting
plt.figure(1)

ax1.plot(time, rotation[:, 0], "r", label="Roll")
ax1.plot(time, setpoints[:, 0], "--", label="Set Points")
ax1.plot(time, pid[:, 0], "m", label="PID Roll")
ax1.legend(loc="upper right")
ax1.grid(True)
ax2.plot(time, rotation[:, 1], "g", label="Pitch")
ax2.plot(time, setpoints[:, 1], "--", label="Set Points")
ax2.plot(time, pid[:, 1], "y", label="PID Pitch")
ax2.legend(loc="upper right")
ax2.grid(True)
ax3.plot(time, rotation[:, 2], "b", label="Yaw")
ax3.plot(time, setpoints[:, 2], "--", label="Set Points")
ax3.plot(time, pid[:, 2], "k", label="PID Yaw")
ax3.legend(loc="upper right")
ax3.grid(True)


plt.figure(2)
ax4.plot(time, translation[:, 0])
ax4.grid(True)
ax5.plot(time, translation[:, 1])
ax5.grid(True)
ax6.plot(time, translation[:, 2])
ax6.grid(True)


# plt.figure(3)
# ax7.plot(time,pid[:,0],"r",label="PID Roll")
# ax7.plot(time,setpoints[:,0],"--",label="Set Points")
# ax7.legend(loc="upper right")
# ax8.plot(time,pid[:,1],"g",label="PID Pitch")
# ax8.plot(time,setpoints[:,1],"--",label="Set Points")
# ax8.legend(loc="upper right")
# ax9.plot(time,pid[:,2],"b",label="PID Yaw")
# ax9.plot(time,setpoints[:,2],"--",label="Set Points")
# ax9.legend(loc="upper right")

plt.show()
