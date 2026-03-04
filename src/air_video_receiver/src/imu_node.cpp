#include <ros/ros.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <iomanip>

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
    ImuBlock accel; // Offset 0
    ImuBlock gyro;  // Offset 224
};
#pragma pack(pop)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_raw_inspector");
    ros::NodeHandle nh;

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
        return -1;

    // 设置非阻塞，用于排空旧数据
    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(7777);

    if (bind(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("Bind failed");
        return -1;
    }

    uint8_t buffer[1024];
    ROS_INFO("Raw Inspector Started. Printing INT16 values.");

    while (ros::ok())
    {
        ssize_t len;
        ssize_t last_len = -1;

        // 【关键】排空旧包，只留最新的一帧
        while ((len = recv(sockfd, buffer, sizeof(buffer), 0)) > 0)
        {
            last_len = len;
        }

        if (last_len == 448)
        {
            ImuFifoData *data = (ImuFifoData *)buffer;

            // 清屏打印，方便观察
            std::cout << "\033[2J\033[1;1H";
            std::cout << "--- IMU Raw Data Inspector (Index 0) ---" << std::endl;
            std::cout << "TimeSample: " << data->accel.timestamp_sample << std::endl;
            std::cout << "Samples   : Acc[" << (int)data->accel.samples << "] Gyro[" << (int)data->gyro.samples << "]" << std::endl;

            std::cout << std::fixed << std::setprecision(0);
            std::cout << "\n[ACCEL RAW]  X: " << std::setw(8) << data->accel.x[0]
                      << "  Y: " << std::setw(8) << data->accel.y[0]
                      << "  Z: " << std::setw(8) << data->accel.z[0] << std::endl;

            std::cout << "[GYRO  RAW]  X: " << std::setw(8) << data->gyro.x[0]
                      << "  Y: " << std::setw(8) << data->gyro.y[0]
                      << "  Z: " << std::setw(8) << data->gyro.z[0] << std::endl;

            std::cout << "\n--- Diagnostic ---" << std::endl;
            // 逻辑判断：静止时，谁的 Z 轴大，谁就是 Accel
            if (std::abs(data->gyro.z[0]) > std::abs(data->accel.z[0]) + 1000)
            {
                std::cout << "!! WARNING: Gyro Z is much larger than Accel Z. Positions might be SWAPPED." << std::endl;
            }
        }

        ros::Duration(0.05).sleep(); // 20Hz 刷新率足够肉眼观察
        ros::spinOnce();
    }
    close(sockfd);
    return 0;
}