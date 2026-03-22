#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#pragma pack(push, 1)
struct ImuBlock
{
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint32_t device_id;
    float dt;
    float scale;
    uint8_t samples;
    uint8_t _pad[3];
    int16_t x[32];
    int16_t y[32];
    int16_t z[32];
};

struct ImuFifoData
{
    ImuBlock accel;
    ImuBlock gyro;
};
#pragma pack(pop)

// --- Mahony 算法变量 ---
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
const float Kp = 2.0f;
const float Ki = 0.005f;

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float recipNorm, halfvx, halfvy, halfvz, halfex, halfey, halfez, qa, qb, qc;

    if ((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))
        return;

    recipNorm = 1.0f / std::sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if (Ki > 0.0f)
    {
        exInt += halfex * Ki * dt;
        eyInt += halfey * Ki * dt;
        ezInt += halfez * Ki * dt;
        gx += exInt;
        gy += eyInt;
        gz += ezInt;
    }
    gx += halfex * Kp;
    gy += halfey * Kp;
    gz += halfez * Kp;
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/drone/imu", 1000);

    bool force_calibrate = false;
    private_nh.param("calibrate", force_calibrate, false);

    float bias_gyro_x = 0, bias_gyro_y = 0, bias_gyro_z = 0;
    private_nh.param("gyro_bias_x", bias_gyro_x, 0.0f);
    private_nh.param("gyro_bias_y", bias_gyro_y, 0.0f);
    private_nh.param("gyro_bias_z", bias_gyro_z, 0.0f);

    bool is_calibrated = !force_calibrate;
    int calibration_count = 0;
    const int CALIBRATION_SAMPLES = 1000;
    double sum_gyro_x = 0, sum_gyro_y = 0, sum_gyro_z = 0;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    fcntl(sockfd, F_SETFL, O_NONBLOCK);
    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(7777);

    if (bind(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("CRITICAL ERROR: Port 7777 is already in use!");
        return -1;
    }

    uint8_t buffer[1024];

    static bool time_initialized = false;
    static ros::Time base_ros_time;
    static uint64_t base_sky_time_us = 0;

    ROS_INFO("============================================");
    ROS_INFO("IMU Node Started! Monitoring Port 7777...");
    ROS_INFO("============================================");

    ros::Time last_print_time = ros::Time::now();
    int packets_processed_this_sec = 0;

    while (ros::ok())
    {
        ssize_t len;

        // 核心修复：处理缓冲区内的【每一个包】，保证积分连续性
        while ((len = recv(sockfd, buffer, sizeof(buffer), 0)) > 0)
        {
            if (len != 448)
            {
                ROS_WARN_THROTTLE(2.0, "Got weird packet size: %ld (Expected 448)", len);
                continue;
            }

            ImuFifoData* data = (ImuFifoData*)buffer;

            // 获取这个包里究竟打包了几个 IMU 采样点 (通常是 3 或 4)
            uint8_t count = data->accel.samples; 
            if (count == 0 || count > 32) continue; // 安全防护

            // 获取硬件真实的时间步长 (比如 0.001s)
            float dt_sec = (data->accel.dt > 0.000001f) ? data->accel.dt : 0.001f; 
            uint64_t dt_us = (uint64_t)(dt_sec * 1000000.0f); // 转成微秒

            // 核心修复：遍历包内的【每一个】采样点
            for (int i = 0; i < count; ++i) {
                // 1. 依次提取数组中的每一个物理值
                float ax = data->accel.x[i] * data->accel.scale;
                float ay = data->accel.y[i] * data->accel.scale;
                float az = data->accel.z[i] * data->accel.scale;
                float gx = data->gyro.x[i] * data->gyro.scale;
                float gy = data->gyro.y[i] * data->gyro.scale;
                float gz = data->gyro.z[i] * data->gyro.scale;

                // --- 阶段 A：静态校准 ---
                if (!is_calibrated) {
                    sum_gyro_x += gx; sum_gyro_y += gy; sum_gyro_z += gz;
                    calibration_count++;
                    if (calibration_count >= CALIBRATION_SAMPLES) {
                        bias_gyro_x = sum_gyro_x / CALIBRATION_SAMPLES;
                        bias_gyro_y = sum_gyro_y / CALIBRATION_SAMPLES;
                        bias_gyro_z = sum_gyro_z / CALIBRATION_SAMPLES;
                        is_calibrated = true;
                        ROS_INFO("Calibration Complete! Bias -> X:%.4f Y:%.4f Z:%.4f", bias_gyro_x, bias_gyro_y, bias_gyro_z);
                        // --- 补回写文件的代码开始 ---
                        std::string pkg_path = ros::package::getPath("air_video_receiver");
                        std::string yaml_path = pkg_path + "/config/imu_calib.yaml";
                        std::ofstream outfile(yaml_path);
                        if (outfile.is_open())
                        {
                            outfile << "gyro_bias_x: " << bias_gyro_x << "\n";
                            outfile << "gyro_bias_y: " << bias_gyro_y << "\n";
                            outfile << "gyro_bias_z: " << bias_gyro_z << "\n";
                            outfile.close();
                            ROS_INFO("Successfully saved bias to %s", yaml_path.c_str());
                        }
                        else
                        {
                            ROS_ERROR("Failed to open %s for writing! Check folder permissions.", yaml_path.c_str());
                        }
                        // --- 补回写文件的代码结束 ---
                    }
                    continue; 
                }

                // --- 阶段 B：Mahony 姿态解算 (高频步进) ---
                gx -= bias_gyro_x; gy -= bias_gyro_y; gz -= bias_gyro_z;
                // 现在的 dt 是完美匹配单次采样的！不会再有慢动作了！
                MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt_sec);

                // --- 阶段 C：精细时间对齐 ---
                if (!time_initialized) {
                    base_ros_time = ros::Time::now(); 
                    base_sky_time_us = data->accel.timestamp_sample; 
                    time_initialized = true;
                    ROS_INFO("\n>>> TIME ANCHOR SET! Publishing 1000Hz IMU Data... <<<\n");
                }

                // 极端严谨：为这个打包数组里的第 i 个数据，计算其独立的绝对时间
                // 假设 timestamp_sample 是这个包里第一个点 [0] 的时间
                uint64_t current_sample_time_us = data->accel.timestamp_sample + (i * dt_us);

                if (current_sample_time_us < base_sky_time_us) continue;

                uint64_t elapsed_us = current_sample_time_us - base_sky_time_us;
                uint32_t elapsed_sec = elapsed_us / 1000000;
                uint32_t elapsed_nsec = (elapsed_us % 1000000) * 1000; 
                ros::Duration elapsed_duration(elapsed_sec, elapsed_nsec);

                // --- 阶段 D：发布 ---
                sensor_msgs::Imu msg;
                msg.header.stamp = base_ros_time + elapsed_duration;
                msg.header.frame_id = "imu_link"; 
                
                msg.linear_acceleration.x = ax; msg.linear_acceleration.y = ay; msg.linear_acceleration.z = az;
                msg.angular_velocity.x = gx; msg.angular_velocity.y = gy; msg.angular_velocity.z = gz;
                
                msg.orientation.w = q0; msg.orientation.x = q1; msg.orientation.y = q2; msg.orientation.z = q3;

                imu_pub.publish(msg);
                // 这里加个计数器，用来测试我们一秒钟到底发了多少个数据
                packets_processed_this_sec++;

                // 临时验证：打印欧拉角，用于定量验证姿态解算准确性
                float roll  = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2));
                float pitch = asinf(2.0f*(q0*q2-q3*q1));
                float yaw   = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3));
                ROS_INFO_THROTTLE(0.5, "RPY: %.1f  %.1f  %.1f (deg)",
                    roll*57.3f, pitch*57.3f, yaw*57.3f);
            }
        }

        // --- 心跳监控雷达 ---
        if ((ros::Time::now() - last_print_time).toSec() >= 1.0)
        {
            if (packets_processed_this_sec == 0)
            {
                ROS_WARN("[RADAR] No data received in the last 1 second! Check hardware/network.");
            }
            else
            {
                ROS_INFO("[RADAR] Working fine. Processed %d packets.", packets_processed_this_sec);
            }
            packets_processed_this_sec = 0;
            last_print_time = ros::Time::now();
        }

        usleep(1000);
        ros::spinOnce();
    }
    return 0;
}