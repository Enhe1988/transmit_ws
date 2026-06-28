#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt

bag = rosbag.Bag('imu_raw_test.bag')

timestamps = []
ax_list, ay_list, az_list = [], [], []
gx_list, gy_list, gz_list = [], [], []

for topic, msg, t in bag.read_messages(topics=['/drone/imu']):
    timestamps.append(msg.header.stamp.to_sec())
    ax_list.append(msg.linear_acceleration.x)
    ay_list.append(msg.linear_acceleration.y)
    az_list.append(msg.linear_acceleration.z)
    gx_list.append(msg.angular_velocity.x)
    gy_list.append(msg.angular_velocity.y)
    gz_list.append(msg.angular_velocity.z)

bag.close()

ts = np.array(timestamps)
t_rel = ts - ts[0]
ax, ay, az = np.array(ax_list), np.array(ay_list), np.array(az_list)
gx, gy, gz = np.array(gx_list), np.array(gy_list), np.array(gz_list)
acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
gyro_mag = np.sqrt(gx**2 + gy**2 + gz**2)

fig, axes = plt.subplots(4, 2, figsize=(16, 14))
fig.suptitle(f'IMU Raw Data Analysis (static, {len(ts)} samples, {len(ts)/(t_rel[-1]):.0f} Hz)', fontsize=14)

# Accel XYZ
axes[0,0].plot(t_rel, ax, '.', markersize=0.5, alpha=0.5, label='ax')
axes[0,0].plot(t_rel, ay, '.', markersize=0.5, alpha=0.5, label='ay')
axes[0,0].plot(t_rel, az, '.', markersize=0.5, alpha=0.5, label='az')
axes[0,0].set_title('Accelerometer XYZ')
axes[0,0].set_ylabel('m/s^2')
axes[0,0].legend()
axes[0,0].grid(True)

# Accel magnitude
axes[0,1].plot(t_rel, acc_mag, '.', markersize=0.5, color='red', alpha=0.5)
axes[0,1].axhline(y=9.81, color='green', linestyle='--', label='gravity (9.81)')
axes[0,1].set_title(f'|Accel| (mean={acc_mag.mean():.4f}, std={acc_mag.std():.4f})')
axes[0,1].set_ylabel('m/s^2')
axes[0,1].legend()
axes[0,1].grid(True)

# Gyro XYZ
axes[1,0].plot(t_rel, gx, '.', markersize=0.5, alpha=0.5, label='gx')
axes[1,0].plot(t_rel, gy, '.', markersize=0.5, alpha=0.5, label='gy')
axes[1,0].plot(t_rel, gz, '.', markersize=0.5, alpha=0.5, label='gz')
axes[1,0].set_title('Gyroscope XYZ')
axes[1,0].set_ylabel('rad/s')
axes[1,0].legend()
axes[1,0].grid(True)

# Gyro magnitude
axes[1,1].plot(t_rel, gyro_mag, '.', markersize=0.5, color='purple', alpha=0.5)
axes[1,1].set_title(f'|Gyro| (mean={gyro_mag.mean():.6f}, max={gyro_mag.max():.4f})')
axes[1,1].set_ylabel('rad/s')
axes[1,1].grid(True)

# Accel histogram
axes[2,0].hist(acc_mag, bins=200, edgecolor='none')
axes[2,0].axvline(x=9.81, color='green', linestyle='--', label='gravity')
axes[2,0].set_title('|Accel| Distribution')
axes[2,0].set_xlabel('m/s^2')
axes[2,0].legend()
axes[2,0].grid(True)

# Spike detection
spike_mask = acc_mag > 15
if spike_mask.sum() > 0:
    axes[2,1].plot(t_rel[spike_mask], acc_mag[spike_mask], 'rx', markersize=5)
    axes[2,1].set_title(f'Accel Spikes (|a|>15): {spike_mask.sum()} hits')
else:
    axes[2,1].text(0.5, 0.5, f'No spikes (|a|>15)\nmax={acc_mag.max():.2f} m/s^2',
                   ha='center', va='center', transform=axes[2,1].transAxes, fontsize=14)
    axes[2,1].set_title('Accel Spikes (|a|>15)')
axes[2,1].set_ylabel('m/s^2')
axes[2,1].grid(True)

# dt analysis
dt = np.diff(ts) * 1000
axes[3,0].plot(t_rel[1:], dt, '.', markersize=0.5, alpha=0.5)
axes[3,0].set_title(f'dt (mean={dt.mean():.3f}ms, std={dt.std():.3f}ms, non-mono={np.sum(dt<=0)})')
axes[3,0].set_xlabel('time (s)')
axes[3,0].set_ylabel('ms')
axes[3,0].grid(True)

# dt histogram
axes[3,1].hist(dt, bins=200, edgecolor='none')
axes[3,1].set_title('dt Distribution')
axes[3,1].set_xlabel('ms')
axes[3,1].grid(True)

plt.tight_layout()
plt.savefig('imu_raw_analysis.png', dpi=150)
plt.close()

# Print stats
print(f"Total: {len(ts)} samples, {t_rel[-1]:.1f}s, {len(ts)/t_rel[-1]:.0f} Hz")
print(f"Accel: |a| mean={acc_mag.mean():.4f} std={acc_mag.std():.4f} min={acc_mag.min():.4f} max={acc_mag.max():.4f}")
print(f"Gyro:  |g| mean={gyro_mag.mean():.6f} max={gyro_mag.max():.4f}")
print(f"Spikes (|a|>15): {(acc_mag>15).sum()}, (|a|>20): {(acc_mag>20).sum()}, (|a|>30): {(acc_mag>30).sum()}")
print(f"dt: mean={dt.mean():.3f}ms std={dt.std():.3f}ms non-mono={np.sum(dt<=0)}")
print(f"Saved: imu_raw_analysis.png")
