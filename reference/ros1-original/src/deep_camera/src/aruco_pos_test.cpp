/**
 * @file aruco_pos_test.cpp
 * @brief ArUco 标记位姿识别工具（移植自 camera_test.cpp）
 *
 * 功能：
 *   1. 实时检测画面中的 ArUco 标记
 *   2. 按 aruco_set/aruco标定码分布.md 的编码规则分类：
 *        - 服务器柜子: ID = 1~4, 边长 1 cm
 *        - 硬盘(r,c):  ID = r*100 + c*10 + d, r∈[1,3], c∈[1,12]
 *            d=1 宽面左(3cm), d=2 宽面右(3cm), d=3 窄面(1cm)
 *   3. 范围内标记: 在 ID 下方实时显示 4x4 变换矩阵（标记系 -> 相机系）
 *   4. 范围外标记: 仅显示 ID
 *   5. 右上角: 距离(带单位) + 欧拉角化简参数
 *
 * 订阅话题：
 *   /camera/color/image_raw              (bgr8)
 *   /camera/aligned_depth_to_color/image_raw (16UC1, mm)
 *   /camera/infra1/image_rect_raw        (mono8)
 *   /camera/infra2/image_rect_raw        (mono8)
 *
 * 按键：'q' / ESC 退出；
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include "ArucoDetector.cpp"

#include <iomanip>
#include <sstream>
#include <cmath>

// ============================================================
// 相机标定矩阵（无标定则用 D435 默认内参近似，640×480）
// ============================================================
cv::Mat g_camera_matrix = (cv::Mat_<double>(3, 3) <<
    615.0,   0.0, 320.0,
      0.0, 615.0, 240.0,
      0.0,   0.0,   1.0);

cv::Mat g_distortion = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

// ============================================================
// 全局变量 —— 存储最新的各帧图像
// ============================================================
cv::Mat g_color;       // 彩色图 (BGR, CV_8UC3)
cv::Mat g_infra1;      // 左红外 (CV_8UC1)
cv::Mat g_infra2;      // 右红外 (CV_8UC1)

bool g_color_ok  = false;
bool g_infra1_ok = false;
bool g_infra2_ok = false;

// ============================================================
// 可配置参数
// ============================================================

const int PRINT_EVERY_N_FRAMES = 15;
int g_frame_count = 0;

// ============================================================
// ArUco 编码设计规则（见 aruco_set/aruco标定码分布.md）
// ============================================================
struct MarkerDesign
{
    bool valid;          // 是否在设计的范围内
    std::string label;   // 类别标签
    double size_m;       // 物理边长（米）
};

MarkerDesign getMarkerDesign(int id)
{
    MarkerDesign d;
    d.valid = false;
    d.label = "UNKNOWN";
    d.size_m = 0.0;

    // 服务器柜子：ID 1~4，边长 1 cm
    if (id >= 1 && id <= 4)
    {
        d.valid = true;
        d.label = "Cabinet";
        d.size_m = 0.01;
        return d;
    }

    // 硬盘(r, c)：ID = r*100 + c*10 + f
    //   r ∈ [1,3], c ∈ [1,12], f=1 宽面左 / f=2 宽面右 / f=3 窄面
    int r = id / 100;                  // 行
    int c = (id - r * 100) / 10;       // 列（支持 10~12 两位数）
    int f = id % 10;                   // ★ 改名为 f：面编码

    if (r >= 1 && r <= 3 && c >= 1 && c <= 12 && f >= 1 && f <= 3)
    {
        d.valid = true;
        if (f == 3)      { d.label = "HDD-N";   d.size_m = 0.01; }
        else if (f == 1) { d.label = "HDD-L"; d.size_m = 0.03; }
        else             { d.label = "HDD-R"; d.size_m = 0.03; }
    }
    return d;
}

// ============================================================
// 位姿计算结果结构体
// ============================================================
struct PoseResult
{
    bool valid;
    cv::Mat T;          // 4x4 变换矩阵（标记系 -> 相机系）
    cv::Mat rvec, tvec; // 旋转向量 / 平移向量
    double dist_m;      // 相机到标记中心的距离（米）
    cv::Vec3d rpy_deg;  // 欧拉角（度）
};

// ============================================================
// 计算标记位姿：solvePnP -> 4x4 矩阵 + 距离 + 欧拉角
// ============================================================
PoseResult computeMarkerPose(const ArucoMarker& m)
{
    PoseResult pr;
    pr.valid = false;

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(m.info.world, m.projected,
                           g_camera_matrix, g_distortion,
                           rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    if (!ok) return pr;

    pr.rvec = rvec;
    pr.tvec = tvec;

    // 旋转向量 -> 旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 组装 4x4 齐次矩阵 T = [R | t; 0 0 0 1]
    pr.T = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(pr.T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(pr.T(cv::Rect(3, 0, 1, 3)));

    // 距离 = 平移向量模长（米）
    pr.dist_m = cv::norm(tvec);

    // 从旋转矩阵提取欧拉角（roll, pitch, yaw），转角度制
    double sy = std::sqrt(R.at<double>(0,0) * R.at<double>(0,0)
                        + R.at<double>(1,0) * R.at<double>(1,0));
    double roll, pitch, yaw;
    if (sy > 1e-6)
    {
        roll  = std::atan2(R.at<double>(2,1), R.at<double>(2,2));
        pitch = std::atan2(-R.at<double>(2,0), sy);
        yaw   = std::atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        roll  = std::atan2(-R.at<double>(1,2), R.at<double>(1,1));
        pitch = std::atan2(-R.at<double>(2,0), sy);
        yaw   = 0.0;
    }
    pr.rpy_deg = cv::Vec3d(roll, pitch, yaw) * 180.0 / CV_PI;

    pr.valid = true;
    return pr;
}

// ============================================================
// 在标记中心绘制 X/Y/Z 坐标轴（红 X、绿 Y、蓝 Z）
// ============================================================
void drawMarkerAxes(cv::Mat& img, const PoseResult& pr, double axis_len)
{
    std::vector<cv::Point3d> referencial;
    referencial.push_back(cv::Point3d(0, 0, 0));
    referencial.push_back(cv::Point3d(axis_len, 0, 0));
    referencial.push_back(cv::Point3d(0, axis_len, 0));
    referencial.push_back(cv::Point3d(0, 0, axis_len));

    std::vector<cv::Point2d> projected;
    cv::projectPoints(referencial, pr.rvec, pr.tvec,
                      g_camera_matrix, g_distortion, projected);

    cv::line(img, projected[0], projected[1], cv::Scalar(0, 0, 255), 2);   // X
    cv::line(img, projected[0], projected[2], cv::Scalar(0, 255, 0), 2);   // Y
    cv::line(img, projected[0], projected[3], cv::Scalar(255, 0, 0), 2);   // Z

    cv::putText(img, "X", projected[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    cv::putText(img, "Y", projected[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(img, "Z", projected[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
}

// ============================================================
// 在指定位置绘制 4x4 变换矩阵
// ============================================================
void drawTransformMatrix(cv::Mat& img, const cv::Mat& T, cv::Point origin)
{
    const int line_h = 14;
    for (int i = 0; i < 4; i++)
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << "[ ";
        for (int j = 0; j < 4; j++)
        {
            ss << std::setw(6) << T.at<double>(i, j) << " ";
        }
        ss << "]";
        cv::putText(img, ss.str(),
                    cv::Point(origin.x, origin.y + (i + 1) * line_h),
                    cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 255, 0), 1);
    }
}

// ============================================================
// 右上角绘制状态面板（自动右对齐）
// ============================================================
void drawTopRightPanel(cv::Mat& img, const std::vector<std::string>& lines)
{
    const int margin = 10;
    int y = 30;
    for (const std::string& text : lines)
    {
        int baseline = 0;
        cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX,
                                      0.5, 1, &baseline);
        cv::putText(img, text,
                    cv::Point(img.cols - sz.width - margin, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
        y += sz.height + 8;
    }
}

// ============================================================
// 各图像的回调函数
// ============================================================
void colorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        g_color = cv_ptr->image.clone();
        g_color_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("彩色图转换失败: %s", e.what());
    }
}


void infra1Callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        g_infra1 = cv_ptr->image.clone();
        g_infra1_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("红外1转换失败: %s", e.what());
    }
}

void infra2Callback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        g_infra2 = cv_ptr->image.clone();
        g_infra2_ok = true;
    }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("红外2转换失败: %s", e.what());
    }
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_pos_test");
    ros::NodeHandle nh;

    // ----------------------------------------------------------
    // 订阅四个话题
    // ----------------------------------------------------------
    ros::Subscriber sub_color  = nh.subscribe("/camera/color/image_raw", 1, colorCallback);
    ros::Subscriber sub_infra1 = nh.subscribe("/camera/infra1/image_rect_raw", 1, infra1Callback);
    ros::Subscriber sub_infra2 = nh.subscribe("/camera/infra2/image_rect_raw", 1, infra2Callback);

    ROS_INFO("============================================");
    ROS_INFO("  ArUco 位姿识别工具已启动");
    ROS_INFO("  范围内标记显示变换矩阵，范围外仅显示 ID");
    ROS_INFO("  按 'q' 或 ESC 键退出");
    ROS_INFO("============================================");

    ros::Rate rate(30);  // 30Hz

    while (ros::ok())
    {
        ros::spinOnce();

        // ---- ArUco 标记检测 ----
        static int threshold_block_size = 7;        // 动态自适应
        static const int BLOCK_SIZE_MIN = 5;
        static const int BLOCK_SIZE_MAX = 13;
        vector<ArucoMarker> markers;

        if (g_color_ok && !g_color.empty())
        {
            markers = ArucoDetector::getMarkers(g_color,
                0.9,                    // limitCosine：放宽，容忍更大畸变
                threshold_block_size,   // 动态阈值块
                50,                     // minArea
                0.035                   // maxError
            );

            // 动态阈值自适应：检测不到时切换块大小
            if (markers.empty())
            {
                threshold_block_size += 2;
                if (threshold_block_size > BLOCK_SIZE_MAX)
                    threshold_block_size = BLOCK_SIZE_MIN;
            }
        }

        // ★ 按编码规则为范围内标记设置真实物理尺寸（solvePnP 必需）
        for (size_t i = 0; i < markers.size(); i++)
        {
            MarkerDesign des = getMarkerDesign(markers[i].id);
            if (des.valid)
            {
                markers[i].info.size = des.size_m;
                markers[i].info.calculateWorldPoints();
            }
        }

        // ---- 显示彩色图像 ----
        if (g_color_ok && !g_color.empty())
        {
            cv::Mat color_display = g_color.clone();

            // ★ 绘制检测到的 ArUco 标记
            PoseResult first_pose;
            bool have_pose = false;
            size_t in_design_count = 0;

            for (size_t i = 0; i < markers.size(); i++)
            {
                const ArucoMarker& m = markers[i];
                MarkerDesign des = getMarkerDesign(m.id);
                if (des.valid) in_design_count++;

                // ① 四边形边框（品红色）
                for (int j = 0; j < 4; j++)
                {
                    cv::line(color_display,
                        m.projected[j],
                        m.projected[(j + 1) % 4],
                        cv::Scalar(255, 0, 255), 2);
                }

                // ② 标记中心点
                cv::Point2f center(0, 0);
                for (int j = 0; j < 4; j++)
                {
                    center.x += m.projected[j].x;
                    center.y += m.projected[j].y;
                }
                center.x /= 4.0f;
                center.y /= 4.0f;

                // ③ ID 文字（范围内标记附加类别标签）
                std::string id_text = "ID:" + std::to_string(m.id);
                if (des.valid) id_text += " [" + des.label + "]";

                cv::putText(color_display, id_text, center,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 0, 0), 3);           // 黑色描边
                cv::putText(color_display, id_text, center,
                    cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 1);       // 黄色文字

                // ④ 四角圆点标记角点顺序
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

                // ⑤ ★ 范围内标记：计算位姿，在 ID 下方显示变换矩阵
                if (des.valid)
                {
                    PoseResult pr = computeMarkerPose(m);
                    if (pr.valid)
                    {
                        drawMarkerAxes(color_display, pr, des.size_m);
                        drawTransformMatrix(color_display, pr.T,
                            cv::Point(int(center.x) - 70, int(center.y) + 28));

                        if (!have_pose)
                        {
                            first_pose = pr;
                            have_pose = true;
                        }
                    }
                }
                // 范围外标记：什么都不做，仅保留上面的 ID 显示
            }

            // ⑥ 左上角：检测统计
            std::string stats = "ArUco found: " + std::to_string(markers.size());
            cv::putText(color_display, stats,
                cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX,
                0.7, cv::Scalar(0, 255, 255), 2);

            // ⑦ 右上角：距离（带单位）+ 化简参数（欧拉角）
            std::vector<std::string> panel;
            panel.push_back("In design: " + std::to_string(in_design_count)
                            + " / " + std::to_string(markers.size()));
            if (have_pose)
            {
                std::ostringstream s1, s2;
                s1 << std::fixed << std::setprecision(3)
                   << "Dist: " << first_pose.dist_m << " m  ("
                   << first_pose.dist_m * 100.0 << " cm)";
                s2 << std::fixed << std::setprecision(1)
                   << "RPY: " << first_pose.rpy_deg[0] << ", "
                   << first_pose.rpy_deg[1] << ", "
                   << first_pose.rpy_deg[2] << " deg";
                panel.push_back(s1.str());
                panel.push_back(s2.str());
            }
            else
            {
                panel.push_back("Dist: N/A");
                panel.push_back("RPY: N/A");
            }
            drawTopRightPanel(color_display, panel);

            cv::imshow("1. Color (BGR) + ArUco Pose", color_display);
        }

        // ---- 终端打印 ----
        g_frame_count++;
        if (g_frame_count % PRINT_EVERY_N_FRAMES == 0)
        {
            if (!markers.empty())
            {
                ROS_INFO("---- 第 %d 帧, 检测到 %zu 个标记 ----",
                         g_frame_count, markers.size());
                for (size_t i = 0; i < markers.size(); i++)
                {
                    MarkerDesign des = getMarkerDesign(markers[i].id);
                    if (!des.valid)
                    {
                        ROS_INFO("  [%zu] ID=%d (未在设计范围内)",
                                 i, markers[i].id);
                    }
                    else
                    {
                        PoseResult pr = computeMarkerPose(markers[i]);
                        if (pr.valid)
                        {
                            ROS_INFO("  [%zu] ID=%d [%s] 距离=%.3f m "
                                     "RPY=(%.1f, %.1f, %.1f) deg",
                                     i, markers[i].id, des.label.c_str(),
                                     pr.dist_m,
                                     pr.rpy_deg[0], pr.rpy_deg[1], pr.rpy_deg[2]);
                        }
                    }
                }
            }
        }

        // ---- 键盘控制 ----
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q' || key == 27)
        {
            ROS_INFO("用户按下退出键，关闭节点...");
            break;
        }
        rate.sleep();
    }

    cv::destroyAllWindows();
    ROS_INFO("ArUco 位姿识别工具已关闭。");
    return 0;
}