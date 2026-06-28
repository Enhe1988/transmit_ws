#!/usr/bin/env python3
"""
时间戳质量检查脚本
检查：1) 相机时间戳是否单调递增  2) IMU 时间戳是否单调递增
      3) 相机与 IMU 时间域是否对齐  4) 帧间隔统计
用法: python3 check_timestamps.py [bag文件路径]
"""

import sys
import os
import glob
import rosbag
import numpy as np


def find_latest_bag():
    bags_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'bags')
    bags = glob.glob(os.path.join(bags_dir, '*.bag'))
    if not bags:
        print(f"[错误] bags/ 目录下没有找到 .bag 文件")
        sys.exit(1)
    latest = max(bags, key=os.path.getmtime)
    print(f"[信息] 自动选择最新 bag: {latest}")
    return latest


def check_monotonic(times, name):
    regressions = []
    for i in range(1, len(times)):
        if times[i] <= times[i-1]:
            regressions.append((i, times[i-1], times[i], times[i] - times[i-1]))
    if regressions:
        print(f"  [✗] {name} 发现 {len(regressions)} 处时间戳回退：")
        for idx, prev, curr, diff in regressions[:5]:  # 只打印前5个
            print(f"      第{idx}帧: {prev:.6f} -> {curr:.6f}  差值={diff*1000:.3f}ms")
        if len(regressions) > 5:
            print(f"      ... 还有 {len(regressions)-5} 处")
    else:
        print(f"  [✓] {name} 时间戳严格单调递增，无回退")
    return len(regressions)


def main():
    bag_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_bag()

    if not os.path.exists(bag_path):
        print(f"[错误] 文件不存在: {bag_path}")
        sys.exit(1)

    print(f"\n正在读取: {os.path.basename(bag_path)}\n")
    bag = rosbag.Bag(bag_path)

    cam_times = []
    imu_times = []

    for topic, msg, _ in bag.read_messages(['/camera/image_raw', '/drone/imu']):
        t = msg.header.stamp.to_sec()
        if topic == '/camera/image_raw':
            cam_times.append(t)
        else:
            imu_times.append(t)

    bag.close()

    print(f"读取完成：相机 {len(cam_times)} 帧，IMU {len(imu_times)} 条\n")

    if len(cam_times) == 0 or len(imu_times) == 0:
        print("[错误] 数据缺失，请检查话题名称")
        sys.exit(1)

    cam_times = np.array(cam_times)
    imu_times = np.array(imu_times)

    # ── 1. 单调性检查 ──────────────────────────────────────────────────
    print("=" * 50)
    print("【单调性检查】")
    check_monotonic(cam_times, "相机")
    check_monotonic(imu_times, "IMU")

    # ── 2. 帧间隔统计 ──────────────────────────────────────────────────
    print("\n" + "=" * 50)
    print("【帧间隔统计】")

    cam_diffs = np.diff(cam_times) * 1000  # 转 ms
    imu_diffs = np.diff(imu_times) * 1000

    print(f"  相机帧间隔: 均值={cam_diffs.mean():.1f}ms  "
          f"标准差={cam_diffs.std():.1f}ms  "
          f"最小={cam_diffs.min():.1f}ms  最大={cam_diffs.max():.1f}ms")
    print(f"  IMU  帧间隔: 均值={imu_diffs.mean():.1f}ms  "
          f"标准差={imu_diffs.std():.1f}ms  "
          f"最小={imu_diffs.min():.1f}ms  最大={imu_diffs.max():.1f}ms")

    cam_large = np.sum(cam_diffs > cam_diffs.mean() * 1.8)
    imu_large = np.sum(imu_diffs > imu_diffs.mean() * 1.8)
    if cam_large:
        print(f"  [!] 相机有 {cam_large} 处间隔超过均值 1.8 倍（疑似丢帧）")
    if imu_large:
        print(f"  [!] IMU 有 {imu_large} 处间隔超过均值 1.8 倍（疑似丢包）")

    # ── 3. 时间域对齐检查 ──────────────────────────────────────────────
    print("\n" + "=" * 50)
    print("【时间域对齐检查】")

    cam_start = cam_times[0]
    cam_end   = cam_times[-1]
    imu_start = imu_times[0]
    imu_end   = imu_times[-1]

    overlap_start = max(cam_start, imu_start)
    overlap_end   = min(cam_end, imu_end)
    overlap_sec   = overlap_end - overlap_start

    print(f"  相机时间范围: {cam_start:.3f} ~ {cam_end:.3f}  ({cam_end-cam_start:.1f}s)")
    print(f"  IMU  时间范围: {imu_start:.3f} ~ {imu_end:.3f}  ({imu_end-imu_start:.1f}s)")
    print(f"  重叠时长: {overlap_sec:.1f}s")

    start_diff = abs(cam_start - imu_start) * 1000
    print(f"  起始时刻差值: {start_diff:.1f}ms", end="  ")
    if start_diff < 100:
        print("[✓] 对齐良好")
    elif start_diff < 500:
        print("[!] 有偏差，建议检查锚点是否共享成功")
    else:
        print("[✗] 偏差过大，时间域未对齐，锚点共享可能失败")

    # ── 4. 结论 ────────────────────────────────────────────────────────
    print("\n" + "=" * 50)
    print("【结论】")
    issues = 0
    if start_diff > 500:
        print("  ✗ 时间域未对齐，检查 IMU 节点是否先启动并成功设置参数服务器锚点")
        issues += 1
    if cam_large > len(cam_times) * 0.05:
        print("  ! 相机丢帧较多，可能影响标定质量")
        issues += 1
    if issues == 0:
        print("  ✓ 时间戳质量正常，可用于 kalibr 标定")
    print()


if __name__ == '__main__':
    main()
