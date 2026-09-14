# ROS 1 → ROS 2 Jazzy 移植规范（本工作区通用）

本文件是 **Damiao_ARM** 项目从 catkin/ROS 1 Noetic 移植到 ament/ROS 2 Jazzy 的
统一约定。所有参与移植的（子）任务都必须严格遵守，保证各包风格一致、可编译。

## 目录

- **原始 ROS 1 源码（只读参考，禁止修改）**：`/home/qzl/workspace/Damiao_ARM/miku_dummy-master/`
- **ROS 2 目标工作空间（所有改动写这里）**：`/home/qzl/workspace/Damiao_ARM/ros2_ws/src/`

环境：Ubuntu 24.04 + ROS 2 Jazzy（`source /opt/ros/jazzy/setup.bash`），构建工具 `colcon`，
C++ 标准 **C++17**。

---

## 1. 头文件映射

| ROS 1 | ROS 2 |
|---|---|
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `#include <ros/package.h>` | `#include <ament_index_cpp/get_package_share_directory.hpp>` |
| `#include <arm_control/ArmMsg.h>` | `#include <arm_control/msg/arm_msg.hpp>` |
| `#include <aruco/Marker.h>` | `#include <aruco/msg/marker.hpp>` |
| `#include <sensor_msgs/JointState.h>` | `#include <sensor_msgs/msg/joint_state.hpp>` |
| `#include <sensor_msgs/Joy.h>` | `#include <sensor_msgs/msg/joy.hpp>` |
| `#include <sensor_msgs/Image.h>` | `#include <sensor_msgs/msg/image.hpp>` |
| `#include <sensor_msgs/CameraInfo.h>` | `#include <sensor_msgs/msg/camera_info.hpp>` |
| `#include <trajectory_msgs/JointTrajectory.h>` | `#include <trajectory_msgs/msg/joint_trajectory.hpp>` |
| `#include <geometry_msgs/Pose.h>` | `#include <geometry_msgs/msg/pose.hpp>` |
| `#include <std_msgs/Bool.h>` | `#include <std_msgs/msg/bool.hpp>` |
| `#include <cv_bridge/cv_bridge.h>` | `#include <cv_bridge/cv_bridge.hpp>` |
| `#include <image_transport/image_transport.h>` | `#include <image_transport/image_transport.hpp>`（Jazzy 中已弃用，建议直接用 `rclcpp` 发布 `sensor_msgs::msg::Image`） |
| `#include <serial/serial.h>` | `#include <serial/serial.h>`（本工作区 `hardware/include/serial/serial.h` 提供 POSIX 兼容实现，用法不变） |

## 2. 消息类型名

ROS 2 里消息类型带命名空间与 `msg` 子命名空间：

- `arm_control::ArmMsg` → `arm_control::msg::ArmMsg`
- `aruco::Marker` → `aruco::msg::Marker`
- `sensor_msgs::JointState` → `sensor_msgs::msg::JointState`
- `trajectory_msgs::JointTrajectory` → `trajectory_msgs::msg::JointTrajectory`

**字段名一律改为 snake_case**（ROS 2 的 IDL 生成器强制小写）：

- `msg.Task_status` → `msg.task_status`
- 其余字段（`mode`/`kp`/`kd`/`pos_1`/`vel_1`/`tor_1` … `pos_7`/`vel_7`/`tor_7`）本来就是 snake_case，不变。

## 3. 节点与回调

```cpp
// ROS 1
ros::init(argc, argv, "node_name");
ros::NodeHandle nh;
ros::Publisher  pub = nh.advertise<T>("topic", 20);
ros::Subscriber sub = nh.subscribe<T>("topic", 20, &Cls::cb, this);
void cb(const T::ConstPtr& msg);

// ROS 2（节点继承 rclcpp::Node）
class X : public rclcpp::Node {
  X() : Node("node_name") {
    pub = this->create_publisher<T>("topic", 20);
    sub = this->create_subscription<T>(
        "topic", 20,
        std::bind(&X::cb, this, std::placeholders::_1));
  }
  void cb(const T::SharedPtr msg);
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<X>();
  rclcpp::spin(node);            // 纯回调节点
  rclcpp::shutdown();
  return 0;
}
```

- 成员类型：`rclcpp::Publisher<T>::SharedPtr`、`rclcpp::Subscription<T>::SharedPtr`。
- 发布：`pub->publish(msg);`（注意是 `->`）。
- **需要自己开线程跑主循环的节点**（原工程用 `ros::AsyncSpinner`）：
  ```cpp
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  node->run();                    // 原来的阻塞主循环
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  ```
- **循环里手动泵回调的节点**（原工程用 `ros::spinOnce()`）：
  ```cpp
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ...
  executor.spin_some();           // 等价于 ros::spinOnce()
  ```

## 4. 时间与频率

| ROS 1 | ROS 2 |
|---|---|
| `ros::Time::now()` | `node->now()`（成员里用 `this->now()`） |
| `ros::Time t = ...;` | `rclcpp::Time t = ...;` |
| `(a - b).toSec()` | `(a - b).seconds()` |
| `ros::Duration(1.0).sleep()` | `rclcpp::sleep_for(std::chrono::seconds(1))` 或 `rclcpp::sleep_for(std::chrono::duration<double>(1.0))` |
| `ros::Rate r(50); r.sleep();` | `rclcpp::WallRate r(50); r.sleep();` |
| `ros::Rate r(5); r.expectedCycleTime().toSec()` | 直接用常数 `1.0/5.0` |
| `ros::Timer` + `nh.createTimer(ros::Duration(0.001), &C::cb, this, false)` | `this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&C::cb, this));`（**注意 ROS 2 的 timer 回调不带参数**，且默认不需要显式 start） |
| `usleep(10000);` | `rclcpp::sleep_for(std::chrono::milliseconds(10));` 或 `usleep` 保留亦可（`<unistd.h>`） |

## 5. 日志

`RCLCPP_*` 宏第二个参数必须是 logger（不能用 printf 风格的可变参数当 logger）：

| ROS 1 | ROS 2 |
|---|---|
| `ROS_INFO("x=%d", a)` | `RCLCPP_INFO(this->get_logger(), "x=%d", a)` |
| `ROS_WARN(...)` | `RCLCPP_WARN(this->get_logger(), ...)` |
| `ROS_ERROR(...)` | `RCLCPP_ERROR(this->get_logger(), ...)` |
| `ROS_FATAL(...)` | `RCLCPP_FATAL(this->get_logger(), ...)` |
| `ROS_INFO_STREAM("s" << x)` | `RCLCPP_INFO_STREAM(this->get_logger(), "s" << x)` |
| `ROS_ERROR_STREAM(...)` | `RCLCPP_ERROR_STREAM(this->get_logger(), ...)` |
| `ROS_WARN_THROTTLE(1.0, "...")` | `RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "...")`（**单位是毫秒**） |

非 ROS 的面向用户打印（`std::cout` / `printf`）保持原样，不要改。

**库代码**（不继承 Node 的 .cpp）里用 `RCLCPP_*` 时需要 logger：
`arm_control` 包已提供 `include/arm_logger.h`，用法
`RCLCPP_WARN(arm_core_logging::logger(), "...")`。
其它包如遇同样情况，可自行用 `rclcpp::get_logger("包名")`。

## 6. 参数

ROS 1 的全局参数服务器没有对应物，改为 **节点参数**：

```cpp
// ROS 1: nh.getParam("claw/open_pos_deg", p.open_pos_deg)
// ROS 2:
this->declare_parameter<double>("claw.open_pos_deg", p.open_pos_deg);   // 默认值兜底
p.open_pos_deg = this->get_parameter("claw.open_pos_deg").as_double();

// 数组
this->declare_parameter<std::vector<double>>(
    "gravity_gains", std::vector<double>{0.5, 0.55, 0.61, 0.6, 0.7, 0.3});
std::vector<double> gains = this->get_parameter("gravity_gains").as_double_array();
```

启动时用 yaml 覆盖：`ros2 run pkg node --ros-args --params-file xxx.yaml`
（注意 ROS 2 的参数文件顶层是节点名，`/**` 通配所有节点。）

## 7. 包路径

```cpp
// ROS 1: ros::package::getPath("arm_control") + "/urdf/miku_dummy.urdf"
// ROS 2:
#include <ament_index_cpp/get_package_share_directory.hpp>
std::string p = ament_index_cpp::get_package_share_directory("arm_control")
                + "/urdf/miku_dummy.urdf";
```
对应的 `CMakeLists.txt` 必须把该目录 `install(DIRECTORY ... DESTINATION share/${PROJECT_NAME})`。

## 8. C++ 细节

- `M_PI` 在 glibc 下可用，但更稳妥是 `#include <numbers>` 后用 `std::numbers::pi`。
- `ROS_WARN_THROTTLE` 的周期单位是**毫秒**。
- 不要用 `boost::shared_ptr`；ROS 2 用 `std::shared_ptr`。
- 消息字段赋值保持原数值/逻辑，**不要改动任何控制算法、阈值、串口协议**。

## 9. 构建

```bash
source /opt/ros/jazzy/setup.bash
cd /home/qzl/workspace/Damiao_ARM/ros2_ws
colcon build --packages-select <包名> --symlink-install
```

构建通过是硬性验收标准。**禁止**为了"让它编过"而删除原有功能或注释掉代码——
必须真正完成移植。若某个 ROS 1 特性在 ROS 2 无对应物，在代码注释里写明
`// ROS 2 差异：...` 并给出等价实现。
