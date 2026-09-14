/**
 * @file visual_grasp_node.cpp
 * @brief 视觉引导抓取 —— 第一步：图像同步 + 鼠标点击提取像素坐标与深度
 * 
 * 功能说明：
 *   1. 使用 message_filters::ApproximateTime 同步订阅彩色图与对齐后的深度图
 *   2. 通过 cv_bridge 将 ROS 图像消息转为 OpenCV 的 cv::Mat 格式
 *   3. 使用 cv::imshow 显示彩色图像
 *   4. 鼠标点击事件回调：打印点击点的像素坐标 (u, v) 及对应的深度值 (mm)
 * 
 * 依赖话题（由 roslaunch realsense2_camera rs_camera.launch align_depth:=true 提供）：
 *   - /camera/color/image_raw                (sensor_msgs::Image, 彩色图)
 *   - /camera/aligned_depth_to_color/image_raw (sensor_msgs::Image, 16UC1, 单位: mm)
 */

#include <ros/ros.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS ↔ OpenCV 桥接
#include <cv_bridge/cv_bridge.h>

// ROS 消息类型
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// message_filters 用于时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// ============================================================
// 全局变量
// ============================================================

// 用于存储最新同步到的深度图（在鼠标回调中访问）
cv::Mat g_current_depth;   // 类型 CV_16UC1，单位毫米

// 深度图是否有效的标志位
bool g_depth_valid = false;

// 显示窗口的名称
const std::string WINDOW_NAME = "Color Image (Click to get depth)";


// ============================================================
// 鼠标回调函数
// ============================================================

/**
 * @brief OpenCV 鼠标事件回调
 * @param event  鼠标事件类型（左键按下、移动等）
 * @param x      图像列坐标（即 u，水平方向像素索引）
 * @param y      图像行坐标（即 v，垂直方向像素索引）
 * @param flags  事件标志（与键盘修饰键相关）
 * @param param  用户自定义参数（此处未使用）
 * 
 * 当用户左键点击彩色图像时：
 *   - 打印像素坐标 (u, v)
 *   - 如果深度图有效，读取并打印该像素对应的深度值（单位：毫米）
 */
void onMouse(int event, int x, int y, int flags, void* param)
{
    // 只响应左键按下事件
    if (event != cv::EVENT_LBUTTONDOWN)
        return;

    // 打印像素坐标（u = x, v = y）
    ROS_INFO("========================================");
    ROS_INFO("  [鼠标点击] 像素坐标 u = %d, v = %d", x, y);

    // 检查坐标是否在图像范围内
    if (!g_depth_valid || g_current_depth.empty())
    {
        ROS_WARN("  深度图尚未就绪，无法读取深度值。请稍后重试。");
        ROS_INFO("========================================");
        return;
    }

    if (x < 0 || x >= g_current_depth.cols || y < 0 || y >= g_current_depth.rows)
    {
        ROS_WARN("  点击位置超出图像范围！图像尺寸: %d x %d", 
                 g_current_depth.cols, g_current_depth.rows);
        ROS_INFO("========================================");
        return;
    }

    // 读取深度值（16位无符号整数，单位：毫米）
    // RealSense D435 对齐后的深度图为 16UC1 格式
    uint16_t depth_mm = g_current_depth.at<uint16_t>(y, x);

    if (depth_mm == 0)
    {
        ROS_WARN("  该点的深度值为 0（可能是无效点或超出相机量程）。");
    }
    else
    {
        // 同时打印毫米和米两种单位，方便后续计算
        ROS_INFO("  深度值: %u mm  ( = %.3f m )", depth_mm, depth_mm / 1000.0);
    }

    ROS_INFO("========================================");
}


// ============================================================
// 同步回调函数
// ============================================================

/**
 * @brief 彩色图与深度图的同步回调
 * @param color_msg  彩色图像消息 (sensor_msgs::ImageConstPtr)
 * @param depth_msg  对齐后的深度图像消息 (sensor_msgs::ImageConstPtr)
 * 
 * 当 message_filters 判定两条消息时间戳匹配时，此函数被调用。
 * 它完成以下工作：
 *   1. 用 cv_bridge 将 ROS 图像转换为 OpenCV 格式
 *   2. 更新全局深度图（供鼠标回调使用）
 *   3. 在窗口中显示彩色图像
 *   4. 窗口已绑定鼠标回调，点击即可查看深度
 */
void syncCallback(const sensor_msgs::ImageConstPtr& color_msg,
                  const sensor_msgs::ImageConstPtr& depth_msg)
{
    // ----------------------------------------------------------
    // 步骤 1: 将 ROS 图像消息转换为 OpenCV 的 cv::Mat
    // ----------------------------------------------------------

    cv_bridge::CvImagePtr cv_color_ptr;
    cv_bridge::CvImagePtr cv_depth_ptr;

    try
    {
        // 彩色图：bgr8 编码
        cv_color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);

        // 深度图：保持原始编码（16UC1），不做变换
        // 注意：message_filters 订阅时已声明了编码格式，这里也可以用
        //       cv_bridge::toCvShare 实现零拷贝（只读），但为安全起见使用 toCvCopy
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
        // 通常 depth_msg->encoding 为 "16UC1" 或 "mono16"
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge 转换失败: %s", e.what());
        return;
    }

    // ----------------------------------------------------------
    // 步骤 2: 更新全局深度图（供鼠标回调读取）
    // ----------------------------------------------------------
    g_current_depth = cv_depth_ptr->image.clone();  // 深拷贝，避免悬空指针
    g_depth_valid = true;

    // ----------------------------------------------------------
    // 步骤 3: 显示彩色图像
    // ----------------------------------------------------------
    cv::imshow(WINDOW_NAME, cv_color_ptr->image);

    // cv::waitKey(1) 是必须的，它让 OpenCV 处理 GUI 事件（包括鼠标回调）
    // 参数 1 表示等待 1ms，足以保证画面刷新且不阻塞 ROS 主循环
    int key = cv::waitKey(1);

    // 按 ESC 键（ASCII 27）可退出
    if (key == 27)
    {
        ROS_INFO("用户按下 ESC，正在关闭节点...");
        ros::shutdown();
    }
}


// ============================================================
// 主函数
// ============================================================

int main(int argc, char** argv)
{
    // ----------------------------------------------------------
    // 步骤 1: 初始化 ROS 节点
    // ----------------------------------------------------------
    ros::init(argc, argv, "visual_grasp_node");
    ros::NodeHandle nh;
    ROS_INFO("视觉引导抓取节点已启动。");

    // ----------------------------------------------------------
    // 步骤 2: 创建 OpenCV 显示窗口
    // ----------------------------------------------------------
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    // ----------------------------------------------------------
    // 步骤 3: 绑定鼠标回调
    // ----------------------------------------------------------
    // 只要窗口存在，鼠标点击就会触发 onMouse 函数
    cv::setMouseCallback(WINDOW_NAME, onMouse, nullptr);
    ROS_INFO("请在 '%s' 窗口中点击图像，查看像素坐标与深度值。", WINDOW_NAME.c_str());
    ROS_INFO("按 ESC 键退出。");

    // ----------------------------------------------------------
    // 步骤 4: 使用 message_filters 进行时间同步订阅
    // ----------------------------------------------------------

    // 4a. 定义同步策略类型
    //     ApproximateTime 会根据时间戳的接近程度自动匹配彩色图和深度图
    //     模板参数：同步的消息类型（顺序与订阅先后对应）
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image,   // 彩色图
        sensor_msgs::Image    // 深度图
    > MySyncPolicy;

    // 4b. 创建两个 message_filters::Subscriber
    //     订阅话题并指定队列大小（队列越大，容忍的时间偏移越大，但内存占用更多）
    message_filters::Subscriber<sensor_msgs::Image> color_sub(nh, 
        "/camera/color/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, 
        "/camera/aligned_depth_to_color/image_raw", 10);

    // 4c. 创建同步器
    //     参数 1: 同步策略（队列大小），此队列决定最多缓存多少条未匹配的消息
    //     参数 2, 3...: 所有参与同步的 subscriber
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), color_sub, depth_sub);

    // 4d. 注册回调函数
    sync.registerCallback(boost::bind(&syncCallback, _1, _2));

    // ----------------------------------------------------------
    // 步骤 5: 进入 ROS 事件循环
    // ----------------------------------------------------------
    ROS_INFO("开始等待图像话题...");
    ros::spin();

    // ----------------------------------------------------------
    // 清理 OpenCV 窗口
    // ----------------------------------------------------------
    cv::destroyAllWindows();
    ROS_INFO("视觉引导抓取节点已关闭。");

    return 0;
}