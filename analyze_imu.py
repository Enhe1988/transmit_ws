#!/usr/bin/env python3
import rosbag
import numpy as np

bag = rosbag.Bag('imu_test.bag')

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
ax, ay, az = np.array(ax_list), np.array(ay_list), np.array(az_list)
gx, gy, gz = np.array(gx_list), np.array(gy_list), np.array(gz_list)
acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
gyro_mag = np.sqrt(gx**2 + gy**2 + gz**2)

dt = np.diff(ts) * 1000  # ms

print(f"=== IMU Data Quality Report ===")
print(f"Total messages: {len(ts)}")
print(f"Duration: {ts[-1] - ts[0]:.2f} s")
print(f"Avg rate: {len(ts) / (ts[-1] - ts[0]):.1f} Hz")
print()

print(f"--- Timestamp dt (ms) ---")
print(f"  Mean: {dt.mean():.3f}  Std: {dt.std():.3f}")
print(f"  Min: {dt.min():.3f}  Max: {dt.max():.3f}")
# dt distribution
for threshold in [0.5, 1.0, 2.0, 3.0, 5.0, 10.0]:
    count = np.sum(dt > threshold)
    if count > 0:
        print(f"  dt > {threshold}ms: {count} ({100*count/len(dt):.1f}%)")

# Check monotonicity
non_mono = np.sum(dt <= 0)
print(f"  Non-monotonic (dt<=0): {non_mono}")
print()

print(f"--- Accelerometer (m/s^2) ---")
print(f"  X: mean={ax.mean():.4f}  std={ax.std():.4f}")
print(f"  Y: mean={ay.mean():.4f}  std={ay.std():.4f}")
print(f"  Z: mean={az.mean():.4f}  std={az.std():.4f}")
print(f"  |a| mean={acc_mag.mean():.4f}  std={acc_mag.std():.4f}")
print(f"  |a| min={acc_mag.min():.4f}  max={acc_mag.max():.4f}")
spikes_acc = np.sum(acc_mag > 15)
print(f"  Spikes (|a|>15): {spikes_acc}")
spikes_acc2 = np.sum(acc_mag > 12)
print(f"  Spikes (|a|>12): {spikes_acc2}")
print()

print(f"--- Gyroscope (rad/s) ---")
print(f"  X: mean={gx.mean():.6f}  std={gx.std():.6f}")
print(f"  Y: mean={gy.mean():.6f}  std={gy.std():.6f}")
print(f"  Z: mean={gz.mean():.6f}  std={gz.std():.6f}")
print(f"  |g| mean={gyro_mag.mean():.6f}  max={gyro_mag.max():.6f}")
spikes_gyro = np.sum(gyro_mag > 1.0)
print(f"  Spikes (|g|>1 rad/s): {spikes_gyro}")
print()

# Gravity check
print(f"--- Gravity Check ---")
print(f"  Expected ~9.81, measured |a|={acc_mag.mean():.4f}")
print(f"  Error: {abs(acc_mag.mean() - 9.81):.4f} m/s^2 ({100*abs(acc_mag.mean()-9.81)/9.81:.2f}%)")
