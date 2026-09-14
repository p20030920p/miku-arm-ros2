/**
 * @file camera_test.cpp
 * @brief 相机数据探索工具 —— 显示所有图像 + 定点采集 RGB 和深度
 * 
 * 学习目标：
 *   1. 直观看到 D435 能提供哪些图像（彩色、深度、红外等）
 *   2. 理解深度图的编码格式（16UC1，单位毫米，0 = 无效）
 *   3. 通过预设坐标观察同一位置在不同图像中的数值变化
 * 
 * 订阅话题：
 *   /camera/color/image_raw              (sensor_msgs::Image, bgr8)
 *   /camera/aligned_depth_to_color/image_raw (sensor_msgs::Image, 16UC1/mono16)
 *   /camera/infra1/image_rect_raw        (sensor_msgs::Image, mono8) — 左红外
 *   /camera/infra2/image_rect_raw        (sensor_msgs::Image, mono8) — 右红外
 * 
 * 用法：
 *   窗口 1：彩色图像（带十字标记）
 *   窗口 2：深度图像（伪彩色热力图，方便观察）
 *   窗口 3：左红外图像
 *   终端：实时打印标记点的 RGB 和深度值
 *   按键 'q' 或 ESC 退出
 */

// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp
#include <rclcpp/rclcpp.hpp>
// ROS 2 差异：cv_bridge/cv_bridge.h → cv_bridge/cv_bridge.hpp
#include <cv_bridge/cv_bridge.hpp>
// ROS 2 差异：sensor_msgs/Image.h → sensor_msgs/msg/image.hpp
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

// ROS 1 的写法是 #include "ArucoDetector.cpp"（aruco 包源码，头文件式实现）。
// ROS 2 里改为包含 deep_camera 自己的接入头：它负责补 OpenCV 4 兼容宏后再
// 引入 aruco 的 ArucoDetector.cpp（详见该头文件顶部的说明与 CMakeLists.txt）。
// ROS 2 差异：aruco 的 C++ 源码级依赖改为 ARUCO_SRC_DIR + 兼容头的方式接入
#include "deep_camera/aruco_detector_compat.hpp"

// ROS 2 差异：std::bind 替换 ROS 1 的 boost::bind
#include <functional>
#include <memory>

// ============================================================
// ArUco 检测相关 —— 相机标定矩阵（无标定则用默认值）
// ============================================================
// 如果没有标定文件，用 D435 默认内参近似矩阵（640×480 分辨率）
cv::Mat g_camera_matrix = (cv::Mat_<double>(3, 3) <<
    615.0,   0.0, 320.0,
      0.0, 615.0, 240.0,
      0.0,   0.0,   1.0);

cv::Mat g_distortion = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

// ============================================================
// 全局变量 —— 存储最新的各帧图像
// ============================================================
cv::Mat g_color;       // 彩色图 (BGR, CV_8UC3)
cv::Mat g_depth;       // 深度图 (CV_16UC1, mm)
cv::Mat g_infra1;      // 左红外 (CV_8UC1)
cv::Mat g_infra2;      // 右红外 (CV_8UC1)

bool g_color_ok  = false;
bool g_depth_ok  = false;
bool g_infra1_ok = false;
bool g_infra2_ok = false;

// ============================================================
// 可配置参数
// ============================================================
// ★ 在这里修改你想要观察的像素坐标 (u, v)
//    彩色图默认分辨率 640x480，所以取中心点 (320, 240) 作为默认值
int g_mark_u = 320;   // 水平像素 (column)
int g_mark_v = 240;   // 垂直像素 (row)

// 终端打印频率控制：每 N 帧打印一次，避免刷屏
const int PRINT_EVERY_N_FRAMES = 15;
int g_frame_count = 0;


// ============================================================
// 日志
// ============================================================
// ROS 2 差异：ROS 1 的 ROS_INFO/ROS_WARN 隐式使用全局 ros::console；
//             ROS 2 的 RCLCPP_* 必须显式传入 logger。
//             本节点沿用 ROS 1 的“全局变量 + 自由函数”结构（回调里没有 this），
//             因此使用与节点同名的具名 logger，日志内容与 ROS 1 保持一致。
static rclcpp::Logger getLogger()
{
    return rclcpp::get_logger("camera_test");
}


// ============================================================
// 辅助函数：绘制十字标记
// ============================================================
void drawCross(cv::Mat& img, int u, int v, cv::Scalar color, int size = 20, int thickness = 2)
{
    cv::line(img, cv::Point(u - size, v), cv::Point(u + size, v), color, thickness);
    cv::line(img, cv::Point(u, v - size), cv::Point(u, v + size), color, thickness);
    cv::circle(img, cv::Point(u, v), 5, color, 2);
}

// ============================================================
// 辅助函数：将深度图转为伪彩色（方便肉眼观察）
// ============================================================
cv::Mat depthToColorMap(const cv::Mat& depth)
{
    cv::Mat depth_normalized;
    // 深度范围映射到 0~255（假设有效范围 0~4000mm = 0~4m）
    // 可根据实际场景调整 max_depth
    double max_depth = 4000.0;  // 4 米
    depth.convertTo(depth_normalized, CV_8UC1, 255.0 / max_depth);
    // 深度为 0 的地方（无效点）设为黑色
    cv::Mat mask = (depth == 0);

    cv::Mat color_map;
    cv::applyColorMap(depth_normalized, color_map, cv::COLORMAP_JET);
    // 把无效点（深度=0）用黑色覆盖
    color_map.setTo(cv::Scalar(0, 0, 0), mask);

    return color_map;
}


// ============================================================
// 各图像的回调函数（独立订阅，简单直观）
// ============================================================
// ROS 2 差异：回调参数类型 sensor_msgs::ImageConstPtr
//             → sensor_msgs::msg::Image::ConstSharedPtr（std::shared_ptr<const T>）

void colorCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        g_color = cv_ptr->image.clone();
        g_color_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(getLogger(), "彩色图转换失败: %s", e.what());
    }
}

void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try
    {
        // 深度图编码通常是 "16UC1" 或 "mono16"，直接用原始编码
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        g_depth = cv_ptr->image.clone();
        g_depth_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(getLogger(), "深度图转换失败: %s", e.what());
    }
}

void infra1Callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        g_infra1 = cv_ptr->image.clone();
        g_infra1_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(getLogger(), "红外1转换失败: %s", e.what());
    }
}

void infra2Callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        g_infra2 = cv_ptr->image.clone();
        g_infra2_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(getLogger(), "红外2转换失败: %s", e.what());
    }
}


// ============================================================
// 主函数
// ============================================================

int main(int argc, char** argv)
{
    // ROS 2 差异：ros::init + ros::NodeHandle → rclcpp::init + rclcpp::Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camera_test");

    // 从节点参数读取标记坐标（可在 launch/params 文件中配置，也可用默认值）
    // ROS 2 差异：ROS 1 的全局参数服务器 nh.param<int>(...) 在 ROS 2 中改为
    //             节点参数 declare_parameter + get_parameter（默认值兜底）。
    //             启动时可用：ros2 run deep_camera camera_test \
    //                 --ros-args -p mark_u:=320 -p mark_v:=240
    node->declare_parameter<int>("mark_u", 320);
    node->declare_parameter<int>("mark_v", 240);
    g_mark_u = node->get_parameter("mark_u").as_int();
    g_mark_v = node->get_parameter("mark_v").as_int();

    // ----------------------------------------------------------
    // 订阅四个话题
    // ----------------------------------------------------------
    // ROS 2 差异：ros::Subscriber → rclcpp::Subscription<T>::SharedPtr，
    //             nh.subscribe(话题, 队列, 回调) →
    //             node->create_subscription<T>(话题, 队列, std::bind(回调, _1))
    //             话题名与消息类型与 ROS 1 完全一致。
    auto sub_color  = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/color/image_raw", 1, std::bind(&colorCallback, std::placeholders::_1));
    auto sub_depth  = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/aligned_depth_to_color/image_raw", 1, std::bind(&depthCallback, std::placeholders::_1));
    // 红外图像话题默认不开启，需要在 launch 中设置 enable_infra1:=true enable_infra2:=true
    // 如果没开启也不影响程序运行，只是这两个窗口会是空的
    auto sub_infra1 = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra1/image_rect_raw", 1, std::bind(&infra1Callback, std::placeholders::_1));
    auto sub_infra2 = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/infra2/image_rect_raw", 1, std::bind(&infra2Callback, std::placeholders::_1));

    RCLCPP_INFO(getLogger(), "============================================");
    RCLCPP_INFO(getLogger(), "  相机数据探索工具已启动");
    RCLCPP_INFO(getLogger(), "  标记坐标: (u=%d, v=%d)", g_mark_u, g_mark_v);
    RCLCPP_INFO(getLogger(), "  观察该点的 RGB 值和深度值（每 %d 帧打印一次）", PRINT_EVERY_N_FRAMES);
    RCLCPP_INFO(getLogger(), "  按 'q' 或 ESC 键退出");
    RCLCPP_INFO(getLogger(), "============================================");

    // ----------------------------------------------------------
    // 主循环：以固定频率刷新显示
    // ----------------------------------------------------------
    // ROS 2 差异：原 ROS 1 在循环里用 ros::spinOnce() 手动泵回调。
    //             ROS 2 对应做法：SingleThreadedExecutor + spin_some()
    //             （spin_some 与 spinOnce 语义一致：处理完当前立即可用的工作后立即返回，
    //              不会阻塞，因此不会影响下面的 cv::imshow / cv::waitKey 刷新节奏）。
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // ROS 2 差异：ros::Rate → rclcpp::WallRate
    rclcpp::WallRate rate(30);  // 30Hz

    // ROS 2 差异：ros::ok() → rclcpp::ok()
    while (rclcpp::ok())
    {
        // 处理 ROS 回调
        executor.spin_some();

        // ---- ArUco 标记检测 ----
        static int aruco_frame_count = 0;
        static int threshold_block_size = 7;        // 动态自适应
        static const int BLOCK_SIZE_MIN = 5;
        static const int BLOCK_SIZE_MAX = 13;
        vector<ArucoMarker> markers;

        if (g_color_ok && !g_color.empty())
        {
            markers = ArucoDetector::getMarkers(g_color,
                0.9,                    // limitCosine：放宽到 0.9，容忍更大畸变
                threshold_block_size,   // ★ 动态阈值块
                50,                     // minArea：降到 50，允许检测更小的标记
                0.035                   // maxError：放宽到 0.035
            );

            // ★ 动态阈值自适应：检测不到时切换块大小
            if (markers.empty())
            {
                threshold_block_size += 2;
                if (threshold_block_size > BLOCK_SIZE_MAX)
                    threshold_block_size = BLOCK_SIZE_MIN;

            }
        }
        aruco_frame_count++;

        // ---- 显示彩色图像（带十字标记）----
        if (g_color_ok && !g_color.empty())
        {
            cv::Mat color_display = g_color.clone();
            drawCross(color_display, g_mark_u, g_mark_v, cv::Scalar(0, 255, 0));  // 绿色十字
            // 显示该点的实时深度值
            std::string coord_text;
            if (g_depth_ok && !g_depth.empty()
                && g_mark_u >= 0 && g_mark_u < g_depth.cols
                && g_mark_v >= 0 && g_mark_v < g_depth.rows)
            {
                uint16_t depth_mm = g_depth.at<uint16_t>(g_mark_v, g_mark_u);
                if (depth_mm == 0)
                    coord_text = "Depth: N/A";
                else
                    coord_text = "Depth: " + std::to_string(depth_mm) + "mm";
            }
            else
            {
                coord_text = "Depth: --";
            }
            cv::putText(color_display, coord_text,
                        cv::Point(g_mark_u + 10, g_mark_v - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);


            // ★ 绘制检测到的 ArUco 标记
            for (size_t i = 0; i < markers.size(); i++)
            {
                const ArucoMarker& m = markers[i];

                // ① 绘制四边形边框（品红色）
                for (int j = 0; j < 4; j++)
                {
                    cv::line(color_display,
                        m.projected[j],
                        m.projected[(j + 1) % 4],
                        cv::Scalar(255, 0, 255), 2);
                }

                // ② 计算标记中心点
                cv::Point2f center(0, 0);
                for (int j = 0; j < 4; j++)
                {
                    center.x += m.projected[j].x;
                    center.y += m.projected[j].y;
                }
                center.x /= 4.0f;
                center.y /= 4.0f;

                // ③ 绘制标记 ID（带黑色描边，可读性更好）
                std::string id_text = "ID:" + std::to_string(m.id);
                cv::putText(color_display, id_text, center,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 0), 3);           // 黑色描边
                cv::putText(color_display, id_text, center,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 1);       // 黄色文字

                // ④ 在四角画小圆点，标记角点顺序
                cv::Scalar corner_colors[4] = {
                    cv::Scalar(0, 0, 255),    // 红：左上
                    cv::Scalar(0, 255, 0),    // 绿：右上
                    cv::Scalar(255, 0, 0),    // 蓝：右下
                    cv::Scalar(255, 255, 0)   // 青：左下
                };
                for (int j = 0; j < 4; j++)
                {
                    cv::circle(color_display, m.projected[j], 4, corner_colors[j], -1);
                }
            }

            // ⑤ 在左上角显示检测统计
            std::string stats = "ArUco found: " + std::to_string(markers.size());
            cv::putText(color_display, stats,
                cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 255), 2);

            
            cv::imshow("1. Color (BGR)", color_display);
        }

        // ---- 显示深度图（伪彩色）----
        if (g_depth_ok && !g_depth.empty())
        {
            cv::Mat depth_color = depthToColorMap(g_depth);
            drawCross(depth_color, g_mark_u, g_mark_v, cv::Scalar(255, 255, 255));  // 白色十字
            cv::imshow("2. Depth (colormap, mm)", depth_color);
        }

        // ---- 显示左红外----
        if (g_infra1_ok && !g_infra1.empty())
        {
            cv::Mat infra1_color;
            cv::cvtColor(g_infra1, infra1_color, cv::COLOR_GRAY2BGR);
            drawCross(infra1_color, g_mark_u, g_mark_v, cv::Scalar(0, 255, 255));
            cv::imshow("3. Left Infrared", infra1_color);
        }

        // ---- 显示右红外----
        if (g_infra2_ok && !g_infra2.empty())
        {
            cv::Mat infra2_color;
            cv::cvtColor(g_infra2, infra2_color, cv::COLOR_GRAY2BGR);
            drawCross(infra2_color, g_mark_u, g_mark_v, cv::Scalar(0, 255, 255));
            cv::imshow("4. Right Infrared", infra2_color);
        }

        // ---- 终端打印：每 N 帧打印一次标记点的 RGB 和深度 ----
        g_frame_count++;
        if (g_frame_count % PRINT_EVERY_N_FRAMES == 0)
        {
            RCLCPP_INFO(getLogger(), "---- 第 %d 帧 ----", g_frame_count);

            // 检查坐标是否在有效范围内
            bool in_range = true;
            if (g_color_ok)
            {
                if (g_mark_u < 0 || g_mark_u >= g_color.cols ||
                    g_mark_v < 0 || g_mark_v >= g_color.rows)
                {
                    RCLCPP_WARN(getLogger(), "  标记坐标超出彩色图范围 (%d x %d)！",
                                g_color.cols, g_color.rows);
                    in_range = false;
                }
            }

            if (in_range)
            {
                // 打印 RGB 值（OpenCV 中是 BGR 顺序）
                if (g_color_ok && !g_color.empty())
                {
                    cv::Vec3b bgr = g_color.at<cv::Vec3b>(g_mark_v, g_mark_u);
                    RCLCPP_INFO(getLogger(), "  RGB = (%d, %d, %d)    BGR = (%d, %d, %d)",
                             bgr[2], bgr[1], bgr[0],   // R, G, B
                             bgr[0], bgr[1], bgr[2]);  // B, G, R
                }

                // 打印深度值
                if (g_depth_ok && !g_depth.empty())
                {
                    // 深度图是 16UC1，值为毫米
                    uint16_t depth_mm = g_depth.at<uint16_t>(g_mark_v, g_mark_u);
                    if (depth_mm == 0)
                    {
                        RCLCPP_WARN(getLogger(), "  深度 = 0 mm (无效点)");
                    }
                    else
                    {
                        // ROS 2 差异：显式转换以匹配 %u（uint16_t 在可变参数中提升为 int，数值不变）
                        RCLCPP_INFO(getLogger(), "  深度 = %u mm = %.3f m",
                                    static_cast<unsigned int>(depth_mm), depth_mm / 1000.0);
                    }
                }
                else
                {
                    RCLCPP_WARN(getLogger(), "  深度图未就绪");
                }
            }

            // 打印 ArUco 检测结果
            if (!markers.empty())
            {
                RCLCPP_INFO(getLogger(), "  检测到 %zu 个 ArUco 标记:", markers.size());
                for (size_t i = 0; i < markers.size(); i++)
                {
                    RCLCPP_INFO(getLogger(), "    标记[%zu]: ID=%d", i, markers[i].id);
                }
            }
        }

        // ---- 键盘控制 ----
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')  // 'q' 或 ESC (27)
        {
            RCLCPP_INFO(getLogger(), "用户按下退出键，关闭节点...");
            break;
        }
        // 用方向键微调标记点位置（方便调试）
        else if (key == 81)       // 左箭头
            g_mark_u = std::max(0, g_mark_u - 5);
        else if (key == 83)       // 右箭头
            g_mark_u += 5;
        else if (key == 82)       // 上箭头
            g_mark_v = std::max(0, g_mark_v - 5);
        else if (key == 84)       // 下箭头
            g_mark_v += 5;

        rate.sleep();
    }

    cv::destroyAllWindows();
    RCLCPP_INFO(getLogger(), "相机测试工具已关闭。");
    // ROS 2 差异：退出前显式 rclcpp::shutdown()（ROS 1 的 ros::ok() 循环退出后由 ros::spin 收尾）
    rclcpp::shutdown();
    return 0;
}
