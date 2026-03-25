#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

image_transport::Publisher pub;
double g_time_offset_sec = 0.0; // 相机时间补偿量（秒），正值表示时间戳往前拨

// --- PTS 时间锚点（与 IMU 锚点机制对称）---
static bool      time_initialized  = false;
static ros::Time base_ros_time;
static GstClockTime base_pts_ns = 0;

GstFlowReturn new_sample(GstAppSink *appsink, gpointer user_data)
{
    GstSample *sample = gst_app_sink_pull_sample(appsink);
    if (!sample)
        return GST_FLOW_ERROR;

    // 动态获取解码器输出的真实尺寸，防止绿屏和内存崩溃
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gint width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);

    GstBuffer *buffer = gst_sample_get_buffer(sample);

    // 获取 GStreamer buffer 的 PTS（纳秒）
    GstClockTime pts = GST_BUFFER_PTS(buffer);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    cv::Mat frame(height, width, CV_8UC3, (char *)map.data);

    if (!frame.empty() && GST_CLOCK_TIME_IS_VALID(pts))
    {
        // 建立锚点：第一帧同时记录 ROS 时间和 PTS
        if (!time_initialized)
        {
            base_ros_time     = ros::Time::now();
            base_pts_ns       = pts;
            time_initialized  = true;
            ROS_INFO(">>> CAMERA TIME ANCHOR SET! PTS-based timestamps active. <<<");
        }

        // 用 PTS 偏移量推算 ROS 时间戳，消除回调调度抖动
        GstClockTime elapsed_ns = pts - base_pts_ns;
        uint32_t elapsed_sec  = elapsed_ns / 1000000000ULL;
        uint32_t elapsed_nsec = elapsed_ns % 1000000000ULL;
        ros::Time stamp = base_ros_time + ros::Duration(elapsed_sec, elapsed_nsec)
                          - ros::Duration(g_time_offset_sec);

        std_msgs::Header header;
        header.stamp    = stamp;
        header.frame_id = "camera_link";

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        pub.publish(msg);
    }

    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return GST_FLOW_OK;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "h265_receiver_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);

    double time_offset_ms = 0.0;
    private_nh.param("time_offset_ms", time_offset_ms, 0.0);
    g_time_offset_sec = time_offset_ms / 1000.0;
    ROS_INFO("Camera time offset: %.1f ms", time_offset_ms);

    pub = it.advertise("/camera/image_raw", 1);

    gst_init(&argc, &argv);

    std::string pipeline_str =
        "udpsrc port=9002 ! h265parse ! avdec_h265 ! videoconvert ! "
        "video/x-raw, format=BGR ! appsink name=mysink";

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
