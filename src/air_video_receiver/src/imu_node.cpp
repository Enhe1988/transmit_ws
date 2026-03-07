#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>

#pragma pack(push, 1)
struct ImuBlock
{
    uint64_t timestamp;
    uint64_t timestamp_sample;
    uint32_t device_id;
    float dt;
    float scale; // 动态缩放因子
    uint8_t samples;
    uint8_t _pad[3]; // 对齐补齐字节，确保头部刚好 32 字节
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
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/drone/imu", 100);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
        return -1;

    // 设置非阻塞，排空旧包，确保最低延迟
    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(7777);

    if (bind(sockfd, (const sockaddr *)&servaddr, sizeof(servaddr)) < 0)
    {
        ROS_ERROR("Bind failed! Port 7777 is occupied.");
        return -1;
    }

    uint8_t buffer[1024];
    ROS_INFO("IMU Node Started! Using DYNAMIC scale factors.");

    while (ros::ok())
    {
        ssize_t len;
        ssize_t last_len = -1;

        // 拿空缓冲区，只留最新一包
        while ((len = recv(sockfd, buffer, sizeof(buffer), 0)) > 0)
        {
            last_len = len;
        }

        if (last_len == 448)
        {
            ImuFifoData *data = (ImuFifoData *)buffer;

            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "imu_link";

            // 核心修复：原始数据直接乘上各自数据块自带的 Scale
            msg.linear_acceleration.x = data->accel.x[0] * data->accel.scale;
            msg.linear_acceleration.y = data->accel.y[0] * data->accel.scale;
            msg.linear_acceleration.z = data->accel.z[0] * data->accel.scale;

            msg.angular_velocity.x = data->gyro.x[0] * data->gyro.scale;
            msg.angular_velocity.y = data->gyro.y[0] * data->gyro.scale;
            msg.angular_velocity.z = data->gyro.z[0] * data->gyro.scale;

            // 如果有需要，可以在这里打印一下当前的 scale，观察 px4 切换模式
            // ROS_INFO_THROTTLE(1.0, "Accel Scale: %f, Gyro Scale: %f", data->accel.scale, data->gyro.scale);

            imu_pub.publish(msg);
        }

        // 匹配 1000Hz 处理速度
        usleep(1000);
        ros::spinOnce();
    }
    return 0;
}