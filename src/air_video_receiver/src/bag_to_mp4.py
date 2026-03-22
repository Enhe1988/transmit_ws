#!/usr/bin/env python3
"""
从 ROS bag 文件中提取 /camera/image_raw 话题并导出为 MP4 视频。
用法：
    python3 bag_to_mp4.py <bag文件路径> [输出mp4路径]
示例：
    python3 bag_to_mp4.py ../bags/3.18_1.bag output.mp4
"""

import sys
import cv2
import rosbag
from cv_bridge import CvBridge

def bag_to_mp4(bag_path, output_path, topic="/camera/image_raw"):
    bridge = CvBridge()

    # 先读一帧确认分辨率和帧率
    print(f"正在读取: {bag_path}")
    bag = rosbag.Bag(bag_path, 'r')

    info = bag.get_type_and_topic_info()
    topic_info = info.topics.get(topic)
    if topic_info is None:
        print(f"错误：bag 中没有找到话题 {topic}")
        bag.close()
        sys.exit(1)

    total_frames = topic_info.message_count
    fps = topic_info.frequency if topic_info.frequency else 24.0
    print(f"共 {total_frames} 帧，频率 {fps:.1f} fps")

    # 读第一帧确认分辨率
    first_frame = None
    for _, msg, _ in bag.read_messages(topics=[topic]):
        first_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        break

    if first_frame is None:
        print("错误：无法读取第一帧")
        bag.close()
        sys.exit(1)

    h, w = first_frame.shape[:2]
    print(f"分辨率: {w}x{h}")
    print(f"输出文件: {output_path}")

    # 创建 VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))

    # 遍历所有帧
    count = 0
    for _, msg, _ in bag.read_messages(topics=[topic]):
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        writer.write(frame)
        count += 1
        if count % 100 == 0:
            print(f"进度: {count}/{total_frames} ({100*count//total_frames}%)")

    writer.release()
    bag.close()
    print(f"完成！共写入 {count} 帧 -> {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 bag_to_mp4.py <bag路径> [输出路径]")
        sys.exit(1)

    bag_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else "output.mp4"
    bag_to_mp4(bag_path, output_path)
