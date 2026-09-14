/**
 * @file aruco_detector_compat.hpp
 * @brief deep_camera 侧的 ArUco C++ 检测库接入头（ROS 1 → ROS 2 移植用）
 *
 * ============================================================================
 * ROS 1 → ROS 2 差异说明（aruco 的 C++ 源码级依赖）
 * ============================================================================
 * ROS 1 的 deep_camera/CMakeLists.txt 里写的是：
 *     include_directories(... ${CMAKE_SOURCE_DIR}/aruco/src)
 *     find_package(catkin REQUIRED COMPONENTS ... aruco)
 * 而源码中直接 `#include "ArucoDetector.cpp"`（注意是 .cpp，不是 .h）。
 *
 * 之所以能这么写，是因为 aruco 包的这份“库”其实是**头文件式实现**：
 *     ArucoDetector.cpp / ArucoMarker.cpp / ArucoMarkerInfo.cpp /
 *     SquareFinder.cpp / CornerRefinement.cpp / math/*.cpp
 * 每个文件都以 `#pragma once` 开头、类成员函数全部写在类体内（隐式 inline），
 * 因此谁 include 谁就得到完整实现，**不需要链接任何库**，也不会产生
 * 需要外部链接的符号。
 *
 * ROS 2 里 aruco 包由另一路任务并行移植。本包移植时它**尚未导出可链接的库**
 * （既没有 CMakeLists.txt / package.xml，install/ 下也没有 aruco），
 * 如果这里写 find_package(aruco REQUIRED)，deep_camera 就无法独立构建。
 *
 * 所以这里采用任务约定的回退方案：
 *   1. CMake 用缓存变量 ARUCO_SRC_DIR 定位 aruco 的源码目录
 *      （默认 ${CMAKE_CURRENT_SOURCE_DIR}/../aruco/src，不硬编码绝对路径）；
 *   2. 该目录加入需要 ArUco 的目标的 include path；
 *   3. 源码改为 `#include "deep_camera/aruco_detector_compat.hpp"`（本文件），
 *      由本文件在包含 aruco 的 ArucoDetector.cpp 之前补好兼容层；
 *   4. **不链接** aruco 库：实现已经由 include 进入本目标的编译单元，
 *      既不强依赖 aruco 是否构建成功，也彻底杜绝重复符号（duplicate symbol）风险。
 *
 * aruco 源码本身只读，未做任何修改；所有兼容处理都收敛在本文件里。
 */

#pragma once

// ---------------------------------------------------------------------------
// OpenCV：先包含总头文件
// ---------------------------------------------------------------------------
// 原 ROS 1 的 camera_test.cpp / aruco_pos_test.cpp 也是先 include
// <opencv2/opencv.hpp>，再 include "ArucoDetector.cpp"，靠这个顺序间接把
// calib3d 模块（cv::solvePnP / cv::projectPoints / cv::SOLVEPNP_ITERATIVE）
// 带进来 —— aruco 的源码里并没有显式 include <opencv2/calib3d.hpp>。
// 这里把这个隐式依赖显式化，保持完全相同的编译结果。
#include <opencv2/opencv.hpp>

// ---------------------------------------------------------------------------
// OpenCV 4 兼容层
// ---------------------------------------------------------------------------
// aruco 的 ArucoDetector.cpp 里用了 C 风格的旧常量：
//     CV_RGB2GRAY, CV_THRESH_BINARY, CV_THRESH_OTSU
// 在 OpenCV 4.6（Ubuntu 24.04 的 libopencv-dev）中，<opencv2/imgproc/imgproc.hpp>
// 不再间接引入 C 兼容常量头，直接编译会报 "was not declared in this scope"。
//
// 正确做法是显式包含 OpenCV 仍提供的 <opencv2/imgproc/types_c.h>：
// 其中 CV_RGB2GRAY=7、CV_BGR2GRAY=6、CV_THRESH_BINARY=0、CV_THRESH_OTSU=8，
// 与 C++ 枚举 cv::COLOR_RGB2GRAY / cv::COLOR_BGR2GRAY / cv::THRESH_BINARY /
// cv::THRESH_OTSU 数值完全相同，因此 ArUco 的灰度化与 Otsu 阈值化行为
// 与 ROS 1 下**逐位一致**，没有任何算法改动。
//
// 注意：这些常量在 OpenCV 里是**枚举量而不是宏**，所以不能用
// `#ifndef CV_RGB2GRAY` 来判断是否存在——一旦用 #define 抢名字，反而会把
// types_c.h 里的枚举声明破坏掉（`cv::COLOR_RGB2GRAY = 7` 不是合法枚举名）。
// 因此这里先用 __has_include 探测该头文件；只有在未来 OpenCV 真的删掉它时，
// 才退回到数值等价的宏兜底。
#if defined(__has_include)
#  if __has_include(<opencv2/imgproc/types_c.h>)
#    include <opencv2/imgproc/types_c.h>
#    define DEEP_CAMERA_HAS_CV_LEGACY_CONSTS 1
#  endif
#endif

#ifndef DEEP_CAMERA_HAS_CV_LEGACY_CONSTS
// 兜底：OpenCV 版本已删除 types_c.h 时，用数值等价的 C++ 枚举名替换
#ifndef CV_RGB2GRAY
#define CV_RGB2GRAY cv::COLOR_RGB2GRAY
#endif
#ifndef CV_THRESH_BINARY
#define CV_THRESH_BINARY cv::THRESH_BINARY
#endif
#ifndef CV_THRESH_OTSU
#define CV_THRESH_OTSU cv::THRESH_OTSU
#endif
#endif  // DEEP_CAMERA_HAS_CV_LEGACY_CONSTS

// ---------------------------------------------------------------------------
// 引入 aruco 包的检测实现（等价于 ROS 1 的 #include "ArucoDetector.cpp"）
// ---------------------------------------------------------------------------
// 该文件通过 -I${ARUCO_SRC_DIR} 找到；它自身会依次 include
// SquareFinder.cpp / CornerRefinement.cpp / ArucoMarker.cpp / ArucoMarkerInfo.cpp
// （这些相对 include 由编译器按 ArucoDetector.cpp 所在目录解析）。
//
// 它同时带来 `using namespace cv; using namespace std;`，
// 原 ROS 1 源码依赖这一点书写 `vector<ArucoMarker>` 等无限定名，此处保持不变。
#include "ArucoDetector.cpp"
