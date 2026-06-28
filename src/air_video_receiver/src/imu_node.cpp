/**
 * IMU 接收节点
 *
 * 功能：通过 UDP 接收天空端发来的 IMU FIFO 数据包，解析后发布为 ROS sensor_msgs/Imu 消息
 *
 * 数据流：
 *   天空端 BMI088 传感器 → STM32H7 读取 FIFO → UDP 发送(端口7777) → 本节点接收解析 → ROS /drone/imu
 *
 * 每个 UDP 包 (448字节) 的结构：
 *   ImuFifoData = ImuBlock(加速度计) + ImuBlock(陀螺仪)
 *   每个 ImuBlock 包含：
 *     - timestamp: 包级时间戳
 *     - timestamp_sample: 第一个采样点的硬件时间戳 (微秒)
 *     - dt: 采样间隔 (秒)，包内相邻采样点之间的时间差
 *     - scale: 原始 int16 → 物理量的缩放系数
 *     - samples: 本包内的采样点数量 (通常 3~4 个)
 *     - x[32], y[32], z[32]: 原始采样数据 (int16)
 *
 * 时间戳计算：
 *   第 i 个采样点的时间 = timestamp_sample + i * dt
 *   然后通过时间锚点转换为 ROS 时间：
 *     ros_time = base_ros_time + (sample_sky_time - base_sky_time)
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cstring>
#include <cmath>

// ============ 数据结构定义 ============
// 必须与天空端 STM32 的数据结构完全一致，包括字节对齐
#pragma pack(push, 1)
struct ImuBlock
{
    uint64_t timestamp;        // 包级时间戳 (目前未使用)
    uint64_t timestamp_sample; // 本包第一个采样点的硬件时间戳 (微秒)
    uint32_t device_id;        // 设备 ID
    float dt;                  // 包内相邻采样点的时间间隔 (秒)，例如 0.001 = 1ms
    float scale;               // int16 原始值 → 物理单位的缩放系数
                               //   加速度计: scale 使得 x[i]*scale 的单位为 m/s²
                               //   陀螺仪:   scale 使得 x[i]*scale 的单位为 rad/s
    uint8_t samples;           // 本包内有效采样点数量 (通常 3~4，最多 32)
    uint8_t _pad[3];           // 填充字节，保证对齐
    int16_t x[32];             // X 轴原始数据数组
    int16_t y[32];             // Y 轴原始数据数组
    int16_t z[32];             // Z 轴原始数据数组
};

struct ImuFifoData
{
    ImuBlock accel;  // 加速度计数据块 (前 224 字节)
    ImuBlock gyro;   // 陀螺仪数据块   (后 224 字节)
};                   // 总计 448 字节
#pragma pack(pop)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 发布 IMU 话题，队列大小 2000 防止高频数据丢失
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/drone/imu", 2000);

    // ============ 可配置参数 ============
    // 发布频率：0 = 全速发布所有采样点(取决于硬件采样率，实测约 486Hz)
    //          >0 = 降采样到指定频率，例如 200 表示 200Hz
    // 用法：rosrun air_video_receiver imu_node _publish_rate:=200
    int publish_rate_hz = 0;
    private_nh.param("publish_rate", publish_rate_hz, 0);

    // 尖峰过滤阈值：超过此值的采样点会被丢弃
    // FIFO 数据偶尔损坏会产生 ~39 m/s² 的假数据，30 的阈值可以过滤掉
    // 正常运动很少超过 20 m/s²（约 2g），30 留有安全余量
    float acc_spike_threshold = 30.0f;  // m/s^2
    float gyro_spike_threshold = 20.0f; // rad/s
    private_nh.param("acc_spike_threshold", acc_spike_threshold, 30.0f);
    private_nh.param("gyro_spike_threshold", gyro_spike_threshold, 20.0f);

    // 预计算平方值，避免每次比较都开方
    float acc_spike_sq = acc_spike_threshold * acc_spike_threshold;
    float gyro_spike_sq = gyro_spike_threshold * gyro_spike_threshold;

    // ============ UDP Socket 初始化 ============
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    fcntl(sockfd, F_SETFL, O_NONBLOCK);  // 非阻塞模式，recv 无数据时立即返回
    sockaddr_in servaddr{};
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;  // 监听所有网卡
    servaddr.sin_port = htons(7777);        // 天空端发送到端口 7777

    if (bind(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("Port 7777 is already in use!");
        return -1;
    }

    uint8_t buffer[1024];  // UDP 接收缓冲区

    // ============ 时间锚点 ============
    // 问题：天空端的时间戳 (sky_time) 和地面端的 ROS 时间不在同一个时间域
    // 解决：用第一个收到的采样点建立映射关系
    //   base_ros_time  = 收到第一个采样点时的 ROS 时间 (ros::Time::now())
    //   base_sky_time_us = 第一个采样点的天空端硬件时间戳
    // 之后所有采样点的 ROS 时间 = base_ros_time + (sample_sky_time - base_sky_time)
    // 这个锚点也通过参数服务器共享给 h265_node，用于视频帧时间同步
    bool time_initialized = false;
    ros::Time base_ros_time;
    uint64_t base_sky_time_us = 0;

    // ============ 降采样网格门控 ============
    // 工作原理：在时间轴上划分固定间隔的"格子"(例如 5ms = 200Hz)
    // 每个格子里只发布第一个落入的采样点，后续的跳过
    // next_target_us 记录下一个格子的起始时间
    uint64_t next_target_us = 0;
    uint64_t publish_interval_us = 0;
    if (publish_rate_hz > 0)
        publish_interval_us = 1000000 / publish_rate_hz;  // 例如 200Hz → 5000us

    // ============ 时间戳单调性保护 ============
    // 问题：UDP 不保证包的到达顺序，包 A 和包 B 可能到达顺序颠倒
    //       另外包内推算的时间 (timestamp_sample + i*dt) 可能与下一个包重叠
    // 解决：记录上次发布的时间戳，只发布严格递增的采样点
    // 代价：约丢弃 19% 的数据 (600Hz → 486Hz)
    uint64_t last_published_us = 0;

    // ============ 心跳监控 ============
    ros::Time last_print_time = ros::Time::now();
    int msg_count = 0;    // 本秒发布的消息数
    int spike_count = 0;  // 本秒过滤的尖峰数

    ROS_INFO("============================================");
    ROS_INFO("IMU Node Started (port 7777, rate=%s)",
             publish_rate_hz > 0 ? (std::to_string(publish_rate_hz) + "Hz").c_str() : "FULL");
    ROS_INFO("============================================");

    // ============ 主循环 ============
    while (ros::ok())
    {
        ssize_t len;
        // 内层循环：一次性读取所有待处理的 UDP 包（非阻塞，读到没数据为止）
        while ((len = recv(sockfd, buffer, sizeof(buffer), 0)) > 0)
        {
            // 包大小校验：ImuFifoData 结构体固定 448 字节
            if (len != 448)
            {
                ROS_WARN_THROTTLE(2.0, "Unexpected packet size: %ld (expected 448)", len);
                continue;
            }

            // 将原始字节流解释为 ImuFifoData 结构体
            ImuFifoData *data = (ImuFifoData *)buffer;

            // 获取本包内的采样点数量 (通常 3~4 个)
            uint8_t count = data->accel.samples;
            if (count == 0 || count > 32) continue;  // 异常包，跳过

            // 获取包内采样间隔，用于推算每个采样点的时间戳  这个地方确定一下运行时候是用了包里的dt还是自己写的0.001，1e6f是什么意思，时间间隔完全采用加速度计的吗，那陀螺仪间隔呢
            // 例如 dt = 0.001s，则第 0 个点在 timestamp_sample，
            //       第 1 个点在 timestamp_sample + 1000us，
            //       第 2 个点在 timestamp_sample + 2000us，依此类推
            float dt_sec = (data->accel.dt > 1e-6f) ? data->accel.dt : 0.001f;
            uint64_t dt_us = (uint64_t)(dt_sec * 1e6f);

            // 检查加速度计和陀螺仪的采样间隔是否一致
            if (std::abs(data->accel.dt - data->gyro.dt) > 1e-6f)
                ROS_WARN_THROTTLE(1.0, "accel.dt=%.6f gyro.dt=%.6f MISMATCH!", data->accel.dt, data->gyro.dt);

            // 遍历包内每一个采样点
            for (int i = 0; i < count; ++i)
            {
                // 将 int16 原始值 × 缩放系数 → 物理单位
                float ax = data->accel.x[i] * data->accel.scale;  // m/s²
                float ay = data->accel.y[i] * data->accel.scale;
                float az = data->accel.z[i] * data->accel.scale;
                float gx = data->gyro.x[i] * data->gyro.scale;    // rad/s
                float gy = data->gyro.y[i] * data->gyro.scale;
                float gz = data->gyro.z[i] * data->gyro.scale;

                // FIFO 末尾可能有零填充（samples 声称有 4 个但实际只有 3 个有效数据）
                // 三轴同时为零在物理上不可能（至少有重力），所以可以安全跳过
                if (ax == 0.0f && ay == 0.0f && az == 0.0f) continue;

                // 加速度尖峰过滤：对每个轴单独检查
                // 注意：不能只检查合力，FIFO 损坏可能只污染单轴，合力可能不超阈值
                if (std::abs(ax) > acc_spike_threshold ||
                    std::abs(ay) > acc_spike_threshold ||
                    std::abs(az) > acc_spike_threshold)
                {
                    spike_count++;
                    ROS_WARN_THROTTLE(1.0, "Accel spike filtered: [%.1f, %.1f, %.1f] m/s^2", ax, ay, az);
                    continue;
                }

                // 陀螺仪尖峰过滤：对每个轴单独检查
                if (std::abs(gx) > gyro_spike_threshold ||
                    std::abs(gy) > gyro_spike_threshold ||
                    std::abs(gz) > gyro_spike_threshold)
                {
                    spike_count++;
                    ROS_WARN_THROTTLE(1.0, "Gyro spike filtered: [%.1f, %.1f, %.1f] rad/s", gx, gy, gz);
                    continue;
                }

                // 时间锚点初始化（只在收到第一个有效采样点时执行一次）
                if (!time_initialized)
                {
                    base_ros_time = ros::Time::now();
                    base_sky_time_us = data->accel.timestamp_sample;
                    next_target_us = base_sky_time_us;
                    time_initialized = true;
                    // 通过参数服务器共享给 h265_node，使视频帧也能用同一个时间基准
                    nh.setParam("/time_anchor/ros_sec", (int)base_ros_time.sec);
                    nh.setParam("/time_anchor/ros_nsec", (int)base_ros_time.nsec);
                    nh.setParam("/time_anchor/sky_us", (double)base_sky_time_us);
                    ROS_INFO("TIME ANCHOR SET: ros=%.3f sky=%lu us",
                             base_ros_time.toSec(), base_sky_time_us);
                }

                // 计算当前采样点的天空端绝对时间
                // 包内第 i 个点 = 包的基准时间 + i × 采样间隔
                uint64_t sample_time_us = data->accel.timestamp_sample + (i * dt_us);
                if (sample_time_us < base_sky_time_us) continue;  // 比锚点还早，跳过

                // 时间戳单调性保护：丢弃时间没有严格递增的采样点
                // 原因：UDP 包可能乱序到达，或相邻包的时间范围有重叠
                if (sample_time_us <= last_published_us) continue;

                // 降采样门控（仅在指定了 publish_rate 时生效）
                if (publish_interval_us > 0)
                {
                    if (sample_time_us < next_target_us) continue;  // 还没到下一个格子
                    next_target_us += publish_interval_us;           // 推进到下一个格子
                }

                last_published_us = sample_time_us;

                // 将天空端时间转换为 ROS 时间
                uint64_t elapsed_us = sample_time_us - base_sky_time_us;
                ros::Duration elapsed(elapsed_us / 1000000, (elapsed_us % 1000000) * 1000);

                // 构造并发布 ROS IMU 消息
                sensor_msgs::Imu msg;
                msg.header.stamp = base_ros_time + elapsed;
                msg.header.frame_id = "imu_link";
                msg.linear_acceleration.x = ax;   // 原始加速度，不减零偏
                msg.linear_acceleration.y = ay;    // VINS 内部会自己估计零偏
                msg.linear_acceleration.z = az;
                msg.angular_velocity.x = gx;       // 原始角速度，不减零偏
                msg.angular_velocity.y = gy;
                msg.angular_velocity.z = gz;
                msg.orientation_covariance[0] = -1; // 标准约定：-1 表示此消息不含姿态数据

                imu_pub.publish(msg);
                msg_count++;
            }
        }

        // 每秒打印一次心跳，方便监控数据接收状态
        if ((ros::Time::now() - last_print_time).toSec() >= 1.0)
        {
            if (msg_count == 0)
                ROS_WARN("[IMU] No data in last 1s! Check hardware/network.");
            else
                ROS_INFO("[IMU] Published %d msgs (spikes filtered: %d)", msg_count, spike_count);
            msg_count = 0;
            spike_count = 0;
            last_print_time = ros::Time::now();
        }

        usleep(500);      // 500us 休眠，避免空转占满 CPU
        ros::spinOnce();   // 处理 ROS 回调（本节点无订阅，但保持 ROS 通信正常）
    }

    close(sockfd);
    return 0;
}
