#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

// 引入 FFmpeg 的 C 语言头文件
extern "C" {
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
}

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstring>
#include <thread>

image_transport::Publisher pub;

// FFmpeg 全局变量
const AVCodec *codec = nullptr;
AVCodecContext *codec_ctx = nullptr;
AVCodecParserContext *parser = nullptr;
AVPacket *pkt = nullptr;
AVFrame *frame = nullptr;
SwsContext *sws_ctx = nullptr;

// --- 时间锚点（从 IMU 节点通过参数服务器共享）---
static bool      anchor_initialized = false;
static ros::Time anchor_ros_time;
static uint64_t  anchor_sky_us = 0;

// --- 当前帧的天空端硬件时间戳（从 "aaaa" 包头提取）---
static uint64_t  current_frame_sky_us = 0;
static bool      current_frame_has_ts = false;

// ==========================================================
// 核心解码与发布函数
// ==========================================================
void decode_and_publish()
{
    int ret = avcodec_send_packet(codec_ctx, pkt);
    if (ret < 0) return;

    while (ret >= 0) {
        ret = avcodec_receive_frame(codec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return;
        else if (ret < 0) return;

        cv::Mat bgr_mat(frame->height, frame->width, CV_8UC3);

        if (!sws_ctx) {
            sws_ctx = sws_getContext(frame->width, frame->height, codec_ctx->pix_fmt,
                                     frame->width, frame->height, AV_PIX_FMT_BGR24,
                                     SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
        }

        uint8_t *dest[4] = { bgr_mat.data, nullptr, nullptr, nullptr };
        int dest_linesize[4] = { static_cast<int>(bgr_mat.step[0]), 0, 0, 0 };
        sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height, dest, dest_linesize);

        // --- 时间戳赋值：使用 IMU 锚点 + 硬件时间戳推算 ---
        ros::Time stamp;
        if (anchor_initialized && current_frame_has_ts &&
            current_frame_sky_us >= anchor_sky_us)
        {
            uint64_t elapsed_us  = current_frame_sky_us - anchor_sky_us;
            uint32_t elapsed_sec = elapsed_us / 1000000;
            uint32_t elapsed_ns  = (elapsed_us % 1000000) * 1000;
            stamp = anchor_ros_time + ros::Duration(elapsed_sec, elapsed_ns);
            stamp -= ros::Duration(0.066); // 补偿 H.265 编码流水线延迟（kalibr 两次标定一致）
        }
        else
        {
            stamp = ros::Time::now();
        }

        // --- 调试：打印时间戳来源和值 ---
        ROS_INFO_THROTTLE(1.0, "[CAM] sky_us=%lu  anchor_sky=%lu  ros_stamp=%.6f  hw_sync=%s",
            current_frame_sky_us, anchor_sky_us, stamp.toSec(),
            (anchor_initialized && current_frame_has_ts) ? "YES" : "NO");

        std_msgs::Header header;
        header.stamp    = stamp;
        header.frame_id = "camera_link";

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", bgr_mat).toImageMsg();
        pub.publish(msg);
    }
}

// ==========================================================
// UDP 接收与解析线程
// ==========================================================
void udp_receiver_thread()
{
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        ROS_ERROR("Failed to create UDP socket!");
        return;
    }

    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port        = htons(9002);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        ROS_ERROR("Failed to bind UDP port 9002!");
        return;
    }

    uint8_t buffer[65536];
    ROS_INFO("UDP thread listening. Hardware timestamp mode ACTIVE......");

    while (ros::ok())
    {
        int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, nullptr, nullptr);
        if (n <= 0) continue;

        uint8_t *payload_ptr = buffer;
        int      payload_len = n;

        // 检测帧头标记 "aaaa"（4字节 0xAA）+ 8字节小端时间戳
        if (n >= 12 &&
            buffer[0] == 0xAA && buffer[1] == 0xAA &&
            buffer[2] == 0xAA && buffer[3] == 0xAA)
        {
            // 提取 8 字节小端 uint64_t 时间戳（微秒）
            uint64_t ts = 0;
            memcpy(&ts, buffer + 4, 8); // x86 本身小端，直接 memcpy 即可
            current_frame_sky_us = ts;
            current_frame_has_ts = true;

            // 尝试从参数服务器获取 IMU 锚点（仅在未初始化时轮询）
            if (!anchor_initialized) {
                int    ros_sec, ros_nsec;
                double sky_us;
                if (ros::param::get("/time_anchor/ros_sec",  ros_sec)  &&
                    ros::param::get("/time_anchor/ros_nsec", ros_nsec) &&
                    ros::param::get("/time_anchor/sky_us",   sky_us))
                {
                    anchor_ros_time     = ros::Time(ros_sec, ros_nsec);
                    anchor_sky_us       = (uint64_t)sky_us;
                    anchor_initialized  = true;
                    ROS_INFO(">>> CAMERA: IMU time anchor received! Hardware sync ACTIVE. <<<");
                }
            }

            payload_ptr = buffer + 12;
            payload_len = n - 12;
        }

        // 将裸流喂给 FFmpeg 解析器
        while (payload_len > 0) {
            int consumed = av_parser_parse2(parser, codec_ctx,
                                            &pkt->data, &pkt->size,
                                            payload_ptr, payload_len,
                                            AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
            if (consumed < 0) break;
            payload_ptr += consumed;
            payload_len -= consumed;

            if (pkt->size > 0) {
                decode_and_publish();
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "h265_receiver_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/camera/image_raw", 1);

    codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!codec) {
        ROS_ERROR("H.265 (HEVC) Decoder not found in FFmpeg!");
        return -1;
    }

    parser = av_parser_init(codec->id);
    if (!parser) {
        ROS_ERROR("Parser not found!");
        return -1;
    }

    codec_ctx = avcodec_alloc_context3(codec);
    codec_ctx->flags      |= AV_CODEC_FLAG_LOW_DELAY;
    codec_ctx->thread_type = FF_THREAD_SLICE;

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        ROS_ERROR("Could not open codec!");
        return -1;
    }

    pkt   = av_packet_alloc();
    frame = av_frame_alloc();

    ROS_INFO("FFmpeg Decoder initialized. Waiting for IMU time anchor...");

    std::thread udp_thread(udp_receiver_thread);

    ros::spin();

    av_parser_close(parser);
    avcodec_free_context(&codec_ctx);
    av_frame_free(&frame);
    av_packet_free(&pkt);
    if (sws_ctx) sws_freeContext(sws_ctx);
    udp_thread.join();

    return 0;
}
