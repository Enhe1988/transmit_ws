#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.rcParams['font.family'] = ['DejaVu Sans', 'SimHei', 'WenQuanYi Micro Hei']
matplotlib.rcParams['axes.unicode_minus'] = False

bag = rosbag.Bag('/home/enhe/transmit_ws/imu_new_board.bag')
timestamps, ax_l, ay_l, az_l, gx_l, gy_l, gz_l = [], [], [], [], [], [], []
for topic, msg, t in bag.read_messages(topics=['/drone/imu']):
    timestamps.append(msg.header.stamp.to_sec())
    ax_l.append(msg.linear_acceleration.x)
    ay_l.append(msg.linear_acceleration.y)
    az_l.append(msg.linear_acceleration.z)
    gx_l.append(msg.angular_velocity.x)
    gy_l.append(msg.angular_velocity.y)
    gz_l.append(msg.angular_velocity.z)
bag.close()

ts = np.array(timestamps)
t_rel = ts - ts[0]
ax, ay, az = np.array(ax_l), np.array(ay_l), np.array(az_l)
gx, gy, gz = np.array(gx_l), np.array(gy_l), np.array(gz_l)
acc_mag = np.sqrt(ax**2 + ay**2 + az**2)

spike_mask = acc_mag > 15

fig, axes = plt.subplots(2, 1, figsize=(14, 8))

# Accel
ax1 = axes[0]
ax1.plot(t_rel, ax, '.', markersize=1, alpha=0.4, color='#2196F3', label='X')
ax1.plot(t_rel, ay, '.', markersize=1, alpha=0.4, color='#4CAF50', label='Y')
ax1.plot(t_rel, az, '.', markersize=1, alpha=0.4, color='#FF9800', label='Z')
if spike_mask.sum() > 0:
    ax1.plot(t_rel[spike_mask], ax[spike_mask], 'rv', markersize=6, label=f'Spike ({spike_mask.sum()})')
    ax1.plot(t_rel[spike_mask], ay[spike_mask], 'rv', markersize=6)
    ax1.plot(t_rel[spike_mask], az[spike_mask], 'rv', markersize=6)
ax1.set_title(f'Accelerometer  (static 30s, {len(ts)} samples, {len(ts)/t_rel[-1]:.0f}Hz)    Spikes: {spike_mask.sum()}', fontsize=13)
ax1.set_ylabel('m/s²', fontsize=12)
ax1.legend(fontsize=11, loc='upper right')
ax1.grid(True, alpha=0.3)

# Gyro
ax2 = axes[1]
ax2.plot(t_rel, gx, '.', markersize=1, alpha=0.4, color='#2196F3', label='X')
ax2.plot(t_rel, gy, '.', markersize=1, alpha=0.4, color='#4CAF50', label='Y')
ax2.plot(t_rel, gz, '.', markersize=1, alpha=0.4, color='#FF9800', label='Z')
ax2.set_title(f'Gyroscope  (max |g|={np.sqrt(gx**2+gy**2+gz**2).max():.4f} rad/s)    Spikes: 0', fontsize=13)
ax2.set_ylabel('rad/s', fontsize=12)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.legend(fontsize=11, loc='upper right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('imu_raw_clean.png', dpi=150)
plt.close()
print("Saved: imu_raw_clean.png")
