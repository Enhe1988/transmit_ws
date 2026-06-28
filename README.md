# air_video_receiver

微型扑翼飞行器地面站数据接收与时间同步 ROS 包。

负责接收天空端通过 UDP 下传的 IMU 原始数据与 H.265 视频流，进行时间戳精确对齐后，以标准 ROS 消息发布，供后续 VIO 或端到端定位算法消费。

---

## 系统架构

```
天空端 (Linux 芯片 + STM32H7)
├── BMI088 IMU ──── UDP:7777 ──→ imu_node  →  /drone/imu
└── 摄像头 H.265 ── UDP:9002 ──→ h265_node →  /camera/image_raw
```

**天空端**负责高频采集 IMU 与视频数据，并在硬件层面为每个数据包打上微秒级时间戳。  
**地面站**（本包）负责接收、解析、时间对齐，输出标准 ROS 消息。

---

## 依赖

| 依赖 | 版本要求 |
|------|---------|
| ROS | Noetic |
| OpenCV | ≥ 3.x |
| FFmpeg | libavcodec / libswscale / libavutil |
| GStreamer | 1.0（CMake 查找，h265_node 实际使用 FFmpeg） |

安装 FFmpeg 开发库：
```bash
sudo apt install libavcodec-dev libavformat-dev libswscale-dev libavutil-dev
```

---

## 编译

```bash
cd ~/transmit_ws
catkin_make
source devel/setup.bash
```

---

## 节点说明

### imu_node

接收天空端 BMI088 IMU FIFO 数据，解析后发布为 `sensor_msgs/Imu`。

**订阅**：无  
**发布**：`/drone/imu`（sensor_msgs/Imu）

**UDP 数据格式**（448 字节/包）：

```
ImuFifoData
├── ImuBlock accel  (224 bytes)  加速度计
│   ├── timestamp_sample: uint64  本包第一个采样点硬件时间戳（微秒）
│   ├── dt: float                 包内相邻采样点时间间隔（秒）
│   ├── scale: float              int16 原始值 → 物理量缩放系数（m/s²）
│   ├── samples: uint8            本包有效采样点数量（通常 3~4，最多 32）
│   └── x/y/z[32]: int16         三轴原始数据
└── ImuBlock gyro   (224 bytes)  陀螺仪（结构相同，scale 单位 rad/s）
```

第 `i` 个采样点的天空端时间 = `timestamp_sample + i × dt`

**参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_rate` | 0 | 发布频率（Hz），0 = 全速（约 486Hz） |
| `acc_spike_threshold` | 30.0 | 加速度尖峰过滤阈值（m/s²） |
| `gyro_spike_threshold` | 20.0 | 陀螺仪尖峰过滤阈值（rad/s） |

**时间锚点机制**：

收到第一个有效采样点时，同时记录 `base_ros_time`（当前 ROS 时间）和 `base_sky_time_us`（天空端硬件时间），建立一次性映射。后续所有帧的 ROS 时间戳通过纯整数偏移推算：

```
ros_stamp = base_ros_time + (sample_sky_time - base_sky_time)
```

锚点通过 ROS 参数服务器共享给 h265_node（`/time_anchor/ros_sec`、`/time_anchor/ros_nsec`、`/time_anchor/sky_us`），实现 IMU 与相机时间同源对齐。

---

### h265_node

接收天空端 H.265 视频流，解码后发布为 `sensor_msgs/Image`。

**订阅**：无  
**发布**：`/camera/image_raw`（sensor_msgs/Image，bgr8）

**解码器**：FFmpeg（libavcodec HEVC）

**UDP 包格式**：

天空端在每帧 H.265 数据前添加 12 字节头：

```
[0xAA 0xAA 0xAA 0xAA] [uint64_t 小端，曝光时刻硬件时间戳（微秒）] [H.265 裸流...]
  4 字节帧标记              8 字节时间戳
```

节点提取该时间戳后，借用 imu_node 建立的时间锚点，计算相机帧的 ROS 时间戳：

```
ros_stamp = base_ros_time + (frame_sky_time - base_sky_time) - 66ms
```

`-66ms` 为 H.265 编码流水线延迟补偿，经 kalibr 标定验证。

---

## Launch 文件

### 完整系统

```bash
# 启动 IMU + 相机 + rqt_image_view
roslaunch air_video_receiver full_system.launch

# 同时录制 bag
roslaunch air_video_receiver full_system.launch record_bag:=true
```

### 单独启动 IMU

```bash
# 正常模式（加载校准文件）
roslaunch air_video_receiver imu_start.launch

# 静态校准模式（采集 1000 个样本，结果写入 config/imu_calib.yaml）
roslaunch air_video_receiver imu_start.launch calibrate:=true

# 正常模式 + 录包
roslaunch air_video_receiver imu_start.launch record_bag:=true
```

### 单独启动相机

```bash
roslaunch air_video_receiver camera_start.launch

# 手动指定时间偏移（ms）
roslaunch air_video_receiver camera_start.launch time_offset_ms:=10.0
```

---

## IMU 校准

静止放置飞行器，运行校准模式采集陀螺仪零偏：

```bash
roslaunch air_video_receiver imu_start.launch calibrate:=true
```

校准结果自动写入 `config/imu_calib.yaml`。之后正常启动时会自动加载。

---

## 工具脚本

| 脚本 | 说明 |
|------|------|
| `src/check_timestamps.py` | 检查 bag 中相机/IMU 时间戳单调性、帧间隔、时间域对齐情况 |
| `src/check_time_offset.py` | 分析相机与 IMU 时间偏移 |
| `analyze_drift.py` | 陀螺仪零偏漂移趋势分析，输出 gyro_drift.png |
| `analyze_imu.py` | IMU 原始数据可视化分析 |

检查时间戳质量：

```bash
# 自动选择最新 bag
python3 src/check_timestamps.py

# 指定 bag 文件
python3 src/check_timestamps.py bags/flight_data_xxx.bag
```

---

## 话题总览

| 话题 | 类型 | 发布节点 | 说明 |
|------|------|---------|------|
| `/drone/imu` | sensor_msgs/Imu | imu_node | IMU 原始数据，约 486Hz |
| `/camera/image_raw` | sensor_msgs/Image | h265_node | BGR 图像，天空端实际帧率 |

---

## 时间同步说明

系统时间同步分三层：

1. **天空端硬件时钟**：IMU 每个采样点和每帧图像在天空端曝光时打上同一个硬件微秒时钟时间戳，两者时间源相同。

2. **时间锚点映射**：imu_node 在第一个采样点到达时建立锚点，将天空端硬件时间域映射到 ROS 时间域。后续所有时间戳通过偏移量推算，不再依赖 `ros::Time::now()`，消除网络抖动影响。

3. **锚点共享**：h265_node 通过 ROS 参数服务器读取 imu_node 建立的锚点，复用同一套时间基准，确保相机与 IMU 时间戳可以精确对齐。

> **注意**：需确保 imu_node 先于 h265_node 启动并完成锚点建立，h265_node 会轮询参数服务器直到获取锚点。`full_system.launch` 中已保证启动顺序。
