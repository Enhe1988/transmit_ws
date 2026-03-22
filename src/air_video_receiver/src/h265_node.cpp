#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

image_transport::Publisher pub;

GstFlowReturn new_sample(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
        return GST_FLOW_ERROR;

    // --- 核心优化：动态获取底层解码器输出的真实尺寸，防止绿屏和内存崩溃 ---
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gint width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstMapInfo map;

    // 锁定内存读取
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    // 构建 OpenCV Mat (使用动态获取的宽高)
    cv::Mat frame(height, width, CV_8UC3, (char *)map.data);

    if (!frame.empty())
    {
        std_msgs::Header header;
        // ⚠️ TODO: 目前是软同步。等待技术支持回复后，这里要替换为基于 IMU 锚点的硬件同步时间！
        header.stamp = ros::Time::now();
        header.frame_id = "camera_link";

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pub.publish(msg);
    }

    // 清理内存
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "h265_receiver_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    pub = it.advertise("/camera/image_raw", 1);

    gst_init(&argc, &argv);

    // 监听 9002 端口
    std::string pipeline_str = "udpsrc port=9002 ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw, format=BGR ! appsink name=mysink";

    GError *error = nullptr;
    GstElement *pipeline = gst_parse_launch(pipeline_str.c_str(), &error);

    if (error)
    {
        ROS_ERROR("GStreamer Pipeline 构建失败: %s", error->message);
        return -1;
    }

    GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    if (!appsink)
    {
        ROS_ERROR("Failed to get appsink element from pipeline!");
        gst_object_unref(pipeline);
        return -1;
    }
    g_object_set(appsink, "emit-signals", TRUE, "sync", FALSE, nullptr);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(new_sample), nullptr);
    gst_object_unref(appsink);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    ROS_INFO("H.265 Video Node Started! Listening on UDP port 9002...");

    ros::spin();

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    return 0;
}