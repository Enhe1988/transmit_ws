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
    servaddr.sin_port = htons(9002);

    if (bind(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("CRITICAL ERROR: Port 9002 is already in use!");
        return -1;
    }

    uint8_t buffer[1024];

    static bool time_initialized = false;
    static ros::Time base_ros_time;
    static uint64_t base_sky_time_us = 0;

    ROS_INFO("============================================");
    ROS_INFO("IMU Node Started! Monitoring Port 9002...");
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

            ImuFifoData *data = (ImuFifoData *)buffer;

            float ax = data->accel.x[0] * data->accel.scale;
            float ay = data->accel.y[0] * data->accel.scale;
            float az = data->accel.z[0] * data->accel.scale;
            float gx = data->gyro.x[0] * data->gyro.scale;
            float gy = data->gyro.y[0] * data->gyro.scale;
            float gz = data->gyro.z[0] * data->gyro.scale;

            // --- 阶段 A：校准 ---
            if (!is_calibrated)
            {
                sum_gyro_x += gx;
                sum_gyro_y += gy;
                sum_gyro_z += gz;
                calibration_count++;
                if (calibration_count >= CALIBRATION_SAMPLES)
                {
                    bias_gyro_x = sum_gyro_x / CALIBRATION_SAMPLES;
                    bias_gyro_y = sum_gyro_y / CALIBRATION_SAMPLES;
                    bias_gyro_z = sum_gyro_z / CALIBRATION_SAMPLES;
                    is_calibrated = true;
                    ROS_INFO("Calibration Complete! Bias -> X:%.4f Y:%.4f Z:%.4f", bias_gyro_x, bias_gyro_y, bias_gyro_z);
                }
                continue;
            }

            // --- 阶段 B：Mahony 姿态解算 ---
            gx -= bias_gyro_x;
            gy -= bias_gyro_y;
            gz -= bias_gyro_z;
            float dt_sec = (data->accel.dt > 0.000001f) ? data->accel.dt : 0.001f;
            MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt_sec);

            // --- 阶段 C：时间对齐 ---
            if (!time_initialized)
            {
                base_ros_time = ros::Time::now();
                base_sky_time_us = data->accel.timestamp_sample;
                time_initialized = true;
                ROS_INFO("\n>>> TIME ANCHOR SET! Publishing IMU Data... <<<\n");
            }

            // 丢弃明显的乱序包（比如时间跳回了开机前）
            if (data->accel.timestamp_sample < base_sky_time_us)
                continue;

            uint64_t elapsed_us = data->accel.timestamp_sample - base_sky_time_us;
            uint32_t elapsed_sec = elapsed_us / 1000000;
            uint32_t elapsed_nsec = (elapsed_us % 1000000) * 1000;
            ros::Duration elapsed_duration(elapsed_sec, elapsed_nsec);

            // --- 阶段 D：发布 ---
            sensor_msgs::Imu msg;
            msg.header.stamp = base_ros_time + elapsed_duration;
            msg.header.frame_id = "imu_link";

            msg.linear_acceleration.x = ax;
            msg.linear_acceleration.y = ay;
            msg.linear_acceleration.z = az;
            msg.angular_velocity.x = gx;
            msg.angular_velocity.y = gy;
            msg.angular_velocity.z = gz;

            msg.orientation.w = q0;
            msg.orientation.x = q1;
            msg.orientation.y = q2;
            msg.orientation.z = q3;

            imu_pub.publish(msg);
            packets_processed_this_sec++;
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