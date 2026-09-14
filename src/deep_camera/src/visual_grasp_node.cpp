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
 *
 * ROS 2 差异：ROS 1 的 roslaunch realsense2_camera rs_camera.launch align_depth:=true
 *             在 ROS 2 中对应 `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true`，
 *             话题名与消息类型完全不变，因此本文件的订阅逻辑无需改动。
 */

// ROS 2 差异：ros/ros.h → rclcpp/rclcpp.hpp
#include <rclcpp/rclcpp.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS ↔ OpenCV 桥接
// ROS 2 差异：cv_bridge/cv_bridge.h → cv_bridge/cv_bridge.hpp
#include <cv_bridge/cv_bridge.hpp>

// ROS 消息类型
// ROS 2 差异：消息头文件加 msg/ 子目录，类型名加 msg 命名空间
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// message_filters 用于时间同步
// ROS 2 差异：ROS 2 的 message_filters 同样提供 Subscriber /
//             sync_policies::ApproximateTime / Synchronizer，
//             但头文件后缀变为 .hpp
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>

// ROS 2 差异：std::bind 替换 ROS 1 的 boost::bind
#include <functional>
#include <memory>

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
// 日志
// ============================================================
// ROS 2 差异：ROS 1 的 ROS_INFO/ROS_WARN 是全局宏（隐式使用全局 ros::console），
//             ROS 2 的 RCLCPP_* 必须显式传入 logger。
//             本节点沿用 ROS 1 的“全局变量 + 自由函数”结构，鼠标回调里没有
//             this->get_logger()，因此使用与节点同名的具名 logger。
//             （移植规范允许：其它包可自行用 rclcpp::get_logger("包名")）
static rclcpp::Logger getLogger()
{
    return rclcpp::get_logger("visual_grasp_node");
}


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
    RCLCPP_INFO(getLogger(), "========================================");
    RCLCPP_INFO(getLogger(), "  [鼠标点击] 像素坐标 u = %d, v = %d", x, y);

    // 检查坐标是否在图像范围内
    if (!g_depth_valid || g_current_depth.empty())
    {
        RCLCPP_WARN(getLogger(), "  深度图尚未就绪，无法读取深度值。请稍后重试。");
        RCLCPP_INFO(getLogger(), "========================================");
        return;
    }

    if (x < 0 || x >= g_current_depth.cols || y < 0 || y >= g_current_depth.rows)
    {
        RCLCPP_WARN(getLogger(), "  点击位置超出图像范围！图像尺寸: %d x %d",
                    g_current_depth.cols, g_current_depth.rows);
        RCLCPP_INFO(getLogger(), "========================================");
        return;
    }

    // 读取深度值（16位无符号整数，单位：毫米）
    // RealSense D435 对齐后的深度图为 16UC1 格式
    uint16_t depth_mm = g_current_depth.at<uint16_t>(y, x);

    if (depth_mm == 0)
    {
        RCLCPP_WARN(getLogger(), "  该点的深度值为 0（可能是无效点或超出相机量程）。");
    }
    else
    {
        // 同时打印毫米和米两种单位，方便后续计算
        // ROS 2 差异：RCLCPP_* 的格式串会被做 printf 格式检查，
        //             uint16_t 在可变参数里会被提升为 int，
        //             显式转换为 unsigned int 以匹配 %u（数值不变）
        RCLCPP_INFO(getLogger(), "  深度值: %u mm  ( = %.3f m )",
                    static_cast<unsigned int>(depth_mm), depth_mm / 1000.0);
    }

    RCLCPP_INFO(getLogger(), "========================================");
}


// ============================================================
// 同步回调函数
// ============================================================

/**
 * @brief 彩色图与深度图的同步回调
 * @param color_msg  彩色图像消息 (sensor_msgs::msg::Image::ConstSharedPtr)
 * @param depth_msg  对齐后的深度图像消息 (sensor_msgs::msg::Image::ConstSharedPtr)
 * 
 * 当 message_filters 判定两条消息时间戳匹配时，此函数被调用。
 * 它完成以下工作：
 *   1. 用 cv_bridge 将 ROS 图像转换为 OpenCV 格式
 *   2. 更新全局深度图（供鼠标回调使用）
 *   3. 在窗口中显示彩色图像
 *   4. 窗口已绑定鼠标回调，点击即可查看深度
 *
 * ROS 2 差异：ROS 1 的回调参数是 boost::shared_ptr<const T>（ConstPtr），
 *             ROS 2 是 std::shared_ptr<const T>（ConstSharedPtr）。
 */
void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
{
    // ----------------------------------------------------------
    // 步骤 1: 将 ROS 图像消息转换为 OpenCV 的 cv::Mat
    // ----------------------------------------------------------

    cv_bridge::CvImagePtr cv_color_ptr;
    cv_bridge::CvImagePtr cv_depth_ptr;

    try
    {
        // 彩色图：bgr8 编码
        // ROS 2 差异：sensor_msgs::image_encodings::BGR8 依然有效，
        //             来自 <sensor_msgs/image_encodings.hpp>（由 cv_bridge.hpp 引入）
        cv_color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);

        // 深度图：保持原始编码（16UC1），不做变换
        // 注意：message_filters 订阅时已声明了编码格式，这里也可以用
        //       cv_bridge::toCvShare 实现零拷贝（只读），但为安全起见使用 toCvCopy
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding);
        // 通常 depth_msg->encoding 为 "16UC1" 或 "mono16"
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(getLogger(), "cv_bridge 转换失败: %s", e.what());
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
        RCLCPP_INFO(getLogger(), "用户按下 ESC，正在关闭节点...");
        // ROS 2 差异：ros::shutdown() → rclcpp::shutdown()
        rclcpp::shutdown();
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
    // ROS 2 差异：ros::init + ros::NodeHandle
    //             → rclcpp::init + std::make_shared<rclcpp::Node>("视觉引导抓取节点")
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("visual_grasp_node");
    RCLCPP_INFO(getLogger(), "视觉引导抓取节点已启动。");

    // ----------------------------------------------------------
    // 步骤 2: 创建 OpenCV 显示窗口
    // ----------------------------------------------------------
    cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);

    // ----------------------------------------------------------
    // 步骤 3: 绑定鼠标回调
    // ----------------------------------------------------------
    // 只要窗口存在，鼠标点击就会触发 onMouse 函数
    cv::setMouseCallback(WINDOW_NAME, onMouse, nullptr);
    RCLCPP_INFO(getLogger(), "请在 '%s' 窗口中点击图像，查看像素坐标与深度值。",
                WINDOW_NAME.c_str());
    RCLCPP_INFO(getLogger(), "按 ESC 键退出。");

    // ----------------------------------------------------------
    // 步骤 4: 使用 message_filters 进行时间同步订阅
    // ----------------------------------------------------------

    // 4a. 定义同步策略类型
    //     ApproximateTime 会根据时间戳的接近程度自动匹配彩色图和深度图
    //     模板参数：同步的消息类型（顺序与订阅先后对应）
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,   // 彩色图
        sensor_msgs::msg::Image    // 深度图
    > MySyncPolicy;

    // 4b. 创建两个 message_filters::Subscriber
    //     ROS 2 差异：构造函数第一个参数是 rclcpp::Node*（不是 NodeHandle），
    //                 第三个参数是 QoS（rmw_qos_profile_t）而不是队列大小。
    //                 这里不传第三个参数 → 使用 rmw_qos_profile_default，
    //                 其 history=KEEP_LAST、depth=10、reliability=RELIABLE，
    //                 与 ROS 1 时的队列长度 10 完全对应。
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub(node.get(),
        "/camera/color/image_raw");
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub(node.get(),
        "/camera/aligned_depth_to_color/image_raw");

    // 4c. 创建同步器
    //     参数 1: 同步策略（队列大小），此队列决定最多缓存多少条未匹配的消息
    //     参数 2, 3...: 所有参与同步的 subscriber
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), color_sub, depth_sub);

    // 4d. 注册回调函数
    // ROS 2 差异：boost::bind + _1/_2 → std::bind + std::placeholders::_1/_2
    sync.registerCallback(std::bind(&syncCallback, std::placeholders::_1, std::placeholders::_2));

    // ----------------------------------------------------------
    // 步骤 5: 进入 ROS 事件循环
    // ----------------------------------------------------------
    RCLCPP_INFO(getLogger(), "开始等待图像话题...");
    // ROS 2 差异：ros::spin() → rclcpp::spin(node)。
    //             rclcpp::spin 内部用单线程执行器在当前（主）线程 spin，
    //             因此同步回调里的 cv::imshow/cv::waitKey 依旧在主线程执行，
    //             与 ROS 1 的行为一致（GUI 事件处理不受影响）。
    rclcpp::spin(node);

    // ----------------------------------------------------------
    // 清理 OpenCV 窗口
    // ----------------------------------------------------------
    cv::destroyAllWindows();
    RCLCPP_INFO(getLogger(), "视觉引导抓取节点已关闭。");

    // ROS 2 差异：rclcpp::shutdown() 可重复调用（ESC 触发时已调用过一次），无副作用
    rclcpp::shutdown();

    return 0;
}
