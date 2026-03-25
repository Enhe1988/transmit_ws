#!/usr/bin/env python3
"""
相机-IMU 时间偏移量分析脚本（互相关法）
用法: python3 check_time_offset.py <bag文件路径>
      python3 check_time_offset.py          # 自动查找 bags/ 目录下最新的 bag
"""

import sys
import glob
import os
import rosbag
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from cv_bridge import CvBridge


def find_latest_bag():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    bags_dir = os.path.join(script_dir, '..', 'bags')
    bags = glob.glob(os.path.join(bags_dir, '*.bag'))
    if not bags:
        print(f"[错误] 在 {bags_dir} 下没有找到任何 .bag 文件")
        sys.exit(1)
    latest = max(bags, key=os.path.getmtime)
    print(f"[信息] 自动选择最新 bag: {latest}")
    return latest


def resample_to_grid(times, values, dt):
    """将不均匀采样插值到均匀时间网格"""
    t_start = times[0]
    t_end   = times[-1]
    t_grid  = np.arange(t_start, t_end, dt)
    v_grid  = np.interp(t_grid, times, values)
    return t_grid, v_grid


def main():
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        bag_path = find_latest_bag()

    if not os.path.exists(bag_path):
        print(f"[错误] 文件不存在: {bag_path}")
        sys.exit(1)

    print(f"[信息] 正在读取: {bag_path}")
    bridge = CvBridge()
    bag = rosbag.Bag(bag_path)

    # ── 相机：计算相邻帧差 ─────────────────────────────────────────────
    cam_times, cam_diffs = [], []
    prev_gray = None
    cam_count = 0

    for _, msg, _ in bag.read_messages('/camera/image_raw'):
        try:
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
        if prev_gray is not None:
            diff = float(np.mean(np.abs(gray - prev_gray)))
            cam_times.append(msg.header.stamp.to_sec())
            cam_diffs.append(diff)
        prev_gray = gray
        cam_count += 1

    print(f"[信息] 读取相机帧: {cam_count} 帧")

    # ── IMU：计算角速度模长 ────────────────────────────────────────────
    imu_times, imu_gyro = [], []
    imu_count = 0

    for _, msg, _ in bag.read_messages('/drone/imu'):
        imu_times.append(msg.header.stamp.to_sec())
        mag = (msg.angular_velocity.x ** 2 +
               msg.angular_velocity.y ** 2 +
               msg.angular_velocity.z ** 2) ** 0.5
        imu_gyro.append(mag)
        imu_count += 1

    bag.close()
    print(f"[信息] 读取 IMU 帧: {imu_count} 条")

    if cam_count == 0:
        print("[错误] bag 中没有相机数据，请确认话题名称为 /camera/image_raw")
        sys.exit(1)
    if imu_count == 0:
        print("[错误] bag 中没有 IMU 数据，请确认话题名称为 /drone/imu")
        sys.exit(1)

    cam_times = np.array(cam_times)
    cam_diffs = np.array(cam_diffs)
    imu_times = np.array(imu_times)
    imu_gyro  = np.array(imu_gyro)

    # ── 跳过前 20 秒，只取稳定段 ─────────────────────────────────────
    t0 = max(cam_times[0], imu_times[0]) + 20.0
    cam_mask  = cam_times >= t0
    imu_mask  = imu_times >= t0
    cam_times = cam_times[cam_mask]
    cam_diffs = cam_diffs[cam_mask]
    imu_times = imu_times[imu_mask]
    imu_gyro  = imu_gyro[imu_mask]

    if len(cam_times) < 10 or len(imu_times) < 10:
        print("[错误] 跳过 20 秒后数据不足，请录制更长的 bag（建议 40 秒以上）")
        sys.exit(1)

    # ── 插值到统一时间网格（以相机帧率为基准，约 33ms）──────────────────
    dt = 0.033
    t_cam_grid, cam_grid = resample_to_grid(cam_times, cam_diffs, dt)
    t_imu_grid, imu_grid = resample_to_grid(imu_times, imu_gyro,  dt)

    # 归一化
    cam_grid = cam_grid / cam_grid.max()
    imu_grid = imu_grid / imu_grid.max()

    # ── 互相关：找让二者最匹配的时间偏移 ─────────────────────────────
    # 搜索范围限制在 ±1 秒内，避免无意义的大偏移
    max_lag_samples = int(1.0 / dt)
    n = len(cam_grid)
    m = len(imu_grid)
    min_len = min(n, m)
    corr = signal.correlate(cam_grid[:min_len], imu_grid[:min_len], mode='full')
    lags  = np.arange(-(min_len - 1), min_len)

    # 只在 ±max_lag_samples 范围内找峰值
    center = len(lags) // 2
    search_corr = corr.copy()
    search_corr[:center - max_lag_samples] = 0
    search_corr[center + max_lag_samples:] = 0

    best_lag   = lags[np.argmax(search_corr)]
    offset_sec = best_lag * dt
    offset_ms  = offset_sec * 1000.0

    print()
    print("=" * 45)
    print(f"  互相关最优时间偏移: {offset_ms:+.1f} ms")
    if offset_ms > 0:
        print(f"  => 相机时间戳偏晚，建议补偿 {abs(offset_ms):.0f} ms")
    elif offset_ms < 0:
        print(f"  => 相机时间戳偏早，建议补偿 {abs(offset_ms):.0f} ms（方向相反）")
    else:
        print(f"  => 时间戳已对齐，无需补偿")
    print("=" * 45)
    print()

    # ── 画图 ──────────────────────────────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=False)

    # 上图：原始信号对比（用各自真实时间轴）
    ax1.plot(imu_times, imu_gyro / imu_gyro.max(),
             color='royalblue', linewidth=0.8, label='IMU 角速度（归一化）')
    ax1.plot(cam_times, cam_diffs / cam_diffs.max(),
             color='tomato', linewidth=0.8, label='相机帧差（归一化）')
    ax1.set_ylabel('归一化幅值')
    ax1.set_title('原始信号（未补偿）')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.4)

    # 下图：互相关曲线，标出最优偏移
    lag_times = lags * dt * 1000  # 转为 ms
    ax2.plot(lag_times, corr / corr.max(), color='purple', linewidth=0.8)
    ax2.axvline(offset_ms, color='red', linestyle='--', linewidth=1.5,
                label=f'最优偏移 {offset_ms:+.1f} ms')
    ax2.axvline(0, color='gray', linestyle=':', linewidth=1, label='零偏移')
    ax2.set_xlim(-1000, 1000)
    ax2.set_xlabel('时间偏移 (ms)')
    ax2.set_ylabel('互相关系数')
    ax2.set_title('互相关曲线（峰值处即为真实时间偏移）')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.4)

    plt.suptitle(f'相机-IMU 时间偏移分析\n{os.path.basename(bag_path)}', fontsize=13)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
