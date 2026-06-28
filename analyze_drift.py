#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag('imu_ana.bag')
timestamps, gx_l, gy_l, gz_l = [], [], [], []
for topic, msg, t in bag.read_messages(topics=['/drone/imu']):
    timestamps.append(msg.header.stamp.to_sec())
    gx_l.append(msg.angular_velocity.x)
    gy_l.append(msg.angular_velocity.y)
    gz_l.append(msg.angular_velocity.z)
bag.close()

ts = np.array(timestamps)
t_rel = ts - ts[0]
gx = np.array(gx_l)
gy = np.array(gy_l)
gz = np.array(gz_l)

# 每10秒计算一次均值，看漂移趋势
window = 10  # seconds
bins = np.arange(0, t_rel[-1], window)
gx_mean, gy_mean, gz_mean, t_bins = [], [], [], []

for b in bins:
    mask = (t_rel >= b) & (t_rel < b + window)
    if mask.sum() > 10:
        gx_mean.append(np.mean(gx[mask]))
        gy_mean.append(np.mean(gy[mask]))
        gz_mean.append(np.mean(gz[mask]))
        t_bins.append(b + window/2)

gx_mean = np.array(gx_mean)
gy_mean = np.array(gy_mean)
gz_mean = np.array(gz_mean)
t_bins = np.array(t_bins)

# 找稳定点：后续变化率 < 0.0001 rad/s per minute
dx = np.diff(gx_mean) / window * 60  # rad/s per minute
stable_idx = None
for i in range(len(dx)-5):
    if np.all(np.abs(dx[i:i+5]) < 0.0001):
        stable_idx = i
        break

fig, ax = plt.subplots(figsize=(14, 5))
ax.plot(t_bins, np.array(gx_mean)*1000, '-o', markersize=3, label='Gyr X (mrad/s)')
ax.plot(t_bins, np.array(gy_mean)*1000, '-o', markersize=3, label='Gyr Y (mrad/s)')
ax.plot(t_bins, np.array(gz_mean)*1000, '-o', markersize=3, label='Gyr Z (mrad/s)')

if stable_idx is not None:
    stable_t = t_bins[stable_idx]
    ax.axvline(x=stable_t, color='red', linestyle='--', linewidth=2,
               label=f'Stable at ~{stable_t/60:.1f} min')
    print(f"Gyro X stabilizes at ~{stable_t/60:.1f} minutes")
else:
    print("Gyro X did not fully stabilize in this recording")

ax.set_xlabel('Time (s)')
ax.set_ylabel('Angular velocity (mrad/s)')
ax.set_title('Gyroscope bias drift (10s window average)')
ax.legend()
ax.grid(True, alpha=0.3)

# 二级x轴显示分钟
ax2 = ax.twiny()
ax2.set_xlim(ax.get_xlim())
ax2.set_xticks(np.arange(0, t_rel[-1], 60))
ax2.set_xticklabels([f'{int(x/60)}min' for x in np.arange(0, t_rel[-1], 60)])

plt.tight_layout()
plt.savefig('gyro_drift.png', dpi=150)
plt.close()
print("Saved: gyro_drift.png")

# 打印每分钟变化量
print("\nGyr X drift rate (rad/s per minute):")
for i in range(0, min(len(dx), 60), 6):
    print(f"  {t_bins[i]/60:.1f}min: {dx[i]*1000:.4f} mrad/s/min")
