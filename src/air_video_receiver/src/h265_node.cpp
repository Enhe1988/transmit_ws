#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

// 全局变量：ROS发布者
image_transport::Publisher pub;

/**
 * @brief 这是 GStreamer 的回调函数，当有新图像解码完成时触发
 * 这相当于“搬运工线程”，负责把数据从 GStreamer 搬到 ROS
 */
GstFlowReturn new_sample(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
    {
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    // 1. 锁定内存，准备读取
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // 2. 获取图像尺寸 (我们已知是 1280x720，但最好从 Caps 获取以防万一)
    // 这里为了精简代码，直接按 720p BGR 格式处理 (3通道)
    int width = 1280;
    int height = 720;

    // 3. 构建 OpenCV Mat (零拷贝，直接引用 GStreamer 的内存)
    // 注意：这里必须保证 GStreamer 输出的是 BGR 格式
    cv::Mat frame(height, width, CV_8UC3, (char *)map.data);

    // 4. 封装 ROS 消息
    if (!frame.empty())
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now(); // 记录当前时间戳
        header.frame_id = "camera_link"; // 坐标系名称

        // 使用 cv_bridge 将 Mat 转为 ROS 消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        // 5. 发布！
        pub.publish(msg);
    }

    // 6. 清理内存
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

int main(int argc, char **argv)
{
    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "h265_receiver_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // 创建发布者，话题名为 /camera/image_raw
    pub = it.advertise("/camera/image_raw", 1);

    // 2. 初始化 GStreamer
    gst_init(&argc, &argv);

    // 3. 构建 GStreamer 管道 (Pipeline)
    // 关键点：
    // udpsrc: 监听 9000 端口
    // h265parse: 整理 H265 数据包
    // avdec_h265: 解码
    // videoconvert: 确保转为 BGR 格式给 OpenCV 用
    // appsink: 也就是我们的程序接口
    // -----------------------------------------------------------
    // 注意：如果你的天空端发的是 RTP 包，请用下面这行：
    // std::string pipeline_str = "udpsrc port=9000 ! application/x-rtp, payload=96 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw, format=BGR ! appsink name=mysink";
    // -----------------------------------------------------------
    // 如果是裸流 (Raw Stream)，用这行：
    std::string pipeline_str = "udpsrc port=9002 ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw, format=BGR ! appsink name=mysink";

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error)
    {
        ROS_ERROR("GStreamer Pipeline 构建失败: %s", error->message);
        return -1;
    }

    // 4. 获取 appsink 元素并连接回调
    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");

    // 开启“发送信号”功能，这样 new_sample 才能被触发
    g_object_set(appsink, "emit-signals", TRUE, "sync", FALSE, nullptr);

    // 连接回调函数
    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample), nullptr);

    // 5. 启动管道 (开始干活)
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    ROS_INFO("H.265 视频接收节点已启动，监听端口 9000...");

    // 6. ROS 主循环 (保持程序不退出)
    ros::spin();

    // 7. 退出清理
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}