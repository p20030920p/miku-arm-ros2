/* ====================================================================
 * trajectory_track.cpp —— ROS 2 Jazzy 移植版
 * 轨迹复现：从 teach_path 目录下的 txt 读取示教轨迹，按时间戳以位置模式下发。
 *
 * ROS 2 差异：
 *   - ros::package::getPath("hardware") -> ament_index 的包 share 目录
 *   - 节点类继承 rclcpp::Node；ros::spinOnce() -> executor.spin_some()
 *   - ros::Rate/ros::Duration → rclcpp::WallRate / rclcpp::sleep_for
 * ==================================================================== */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <thread> // 引入线程库
#include <chrono>
#include <memory>

// 用于存储轨迹中单个点的结构体
struct TrajectoryPoint {
    double timestamp;
    double positions[6];
    double velocities[6];
};

class TestNode : public rclcpp::Node {
private:
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;

    // 单线程 executor：主循环里手动 spin_some()，等价于 ROS 1 的 ros::spinOnce()
    rclcpp::executors::SingleThreadedExecutor executor_;

    arm_control::msg::ArmMsg pub_msg;

    std::vector<TrajectoryPoint> trajectory; // 用于存储从文件加载的轨迹

public:
    TestNode() : rclcpp::Node("trajectory_track_node") {
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
            "Arm_rx", 20,
            std::bind(&TestNode::msgCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "轨迹复现节点启动成功。");
    }

    // 把本节点挂到内部 executor（必须在 run() 之前调用一次）
    void attachExecutor()
    {
        executor_.add_node(this->shared_from_this());
    }

    // 订阅回调函数
    void msgCallback(const arm_control::msg::ArmMsg::SharedPtr msg) {
        // 说明：回调体在原工程里整段是注释（只保留订阅通道，不做处理），
        //       这里显式标注参数未使用，避免 -Wunused-parameter 告警。
        (void)msg;
        // RCLCPP_INFO(this->get_logger(), "订阅成功\n ");
        // 可以用来监视机械臂的实际位置与指令位置的差异
    }

    // 从文件加载轨迹数据
    bool loadTrajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开轨迹文件: %s", filepath.c_str());
            return false;
        }

        trajectory.clear();
        std::string line;
        
        // 读取并忽略前2行（文件头）
        std::getline(file, line); 
        std::getline(file, line); 

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue; // 跳过空行或注释行
            }

            std::stringstream ss(line);
            TrajectoryPoint point;
            ss >> point.timestamp;
            // 读取6个位置
            for (int i = 0; i < 6; ++i) {
                ss >> point.positions[i];
            }
            // 读取6个速度
            for (int i = 0; i < 6; ++i) {
                ss >> point.velocities[i];
            }

            if (ss.fail()) {
                RCLCPP_WARN(this->get_logger(), "解析行失败，已跳过: %s", line.c_str());
                continue;
            }
            trajectory.push_back(point);
        }

        file.close();
        if (trajectory.empty()) {
            RCLCPP_ERROR(this->get_logger(), "未从文件中加载任何有效的轨迹点。");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个轨迹点。", trajectory.size());
        return true;
    }
    
    // 运行主循环，实现轨迹复现
    void run() {
        // 1. 获取用户输入的文件名
        std::cout << "请输入 'teach_path' 文件夹中的轨迹文件名 (例如: 2025-08-28_14-42-35):\n";
        std::string filename;
        std::getline(std::cin, filename);

        // 2. 构建完整文件路径并加载
        std::string package_path = ament_index_cpp::get_package_share_directory("hardware");
        if (package_path.empty()) {
            RCLCPP_FATAL(this->get_logger(), "找不到功能包 'hardware'。请检查工作区环境。");
            return;
        }
        std::string full_path = package_path + "/teach_path/" + filename + ".txt";

        if (!loadTrajectory(full_path)) {
            return; // 加载失败，退出
        }

        // 3. 等待用户确认开始
        std::cout << "\n轨迹加载完毕。按下【回车键】开始复现...> ";
        std::string dummy_input;
        std::getline(std::cin, dummy_input);

        // 4. 开始按时间戳发布轨迹
        RCLCPP_INFO(this->get_logger(), "开始执行轨迹复现...");
        rclcpp::Time start_time = this->now();
        rclcpp::WallRate rate(200); // 以200Hz的频率循环，以确保时间戳的精确触发
        size_t current_point_index = 0;

        while (rclcpp::ok() && current_point_index < trajectory.size()) {
            rclcpp::Duration elapsed_time = this->now() - start_time;

            // 检查当前经过的时间是否已经到达或超过下一个轨迹点的时间戳
            if (elapsed_time.seconds() >= trajectory[current_point_index].timestamp) {
                arm_control::msg::ArmMsg msg;
                msg.task_status = 1;
                msg.mode  = 2; // 位置模式

                const auto& curr_point = trajectory[current_point_index];

                // 赋值六个关节的位置
                msg.pos_1 = curr_point.positions[0];
                msg.pos_2 = curr_point.positions[1];
                msg.pos_3 = curr_point.positions[2];
                msg.pos_4 = curr_point.positions[3];
                msg.pos_5 = curr_point.positions[4];
                msg.pos_6 = curr_point.positions[5];

                // --- 直接使用文件中记录的速度 ---
                msg.vel_1 = curr_point.velocities[0];
                msg.vel_2 = curr_point.velocities[1];
                msg.vel_3 = curr_point.velocities[2];
                msg.vel_4 = curr_point.velocities[3];
                msg.vel_5 = curr_point.velocities[4];
                msg.vel_6 = curr_point.velocities[5];

                // 设置增益参数
                msg.kp = 0;
                msg.kd = 6;

                pub->publish(msg);
                RCLCPP_INFO(this->get_logger(), "发布点 %zu, 时间戳: %.4f", current_point_index, trajectory[current_point_index].timestamp);
                // 打印六个关节的位置和速度
                RCLCPP_INFO(this->get_logger(), "位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                         msg.pos_1, msg.pos_2, msg.pos_3,
                         msg.pos_4, msg.pos_5, msg.pos_6);
                RCLCPP_INFO(this->get_logger(), "速度: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                         msg.vel_1, msg.vel_2, msg.vel_3,
                         msg.vel_4, msg.vel_5, msg.vel_6);
                current_point_index++; // 移动到下一个点
            }
            executor_.spin_some();
            rate.sleep();
        }
        
        RCLCPP_INFO(this->get_logger(), "轨迹执行完毕。");
        // 延时一段时间，确保最后的命令被执行
        rclcpp::sleep_for(std::chrono::seconds(1));
        // 发布机械臂使能消息，机械臂恢复到初始位置
        pub_msg.task_status = 1;
        pub_msg.mode  = 2;
        pub_msg.pos_1 = 0  ; pub_msg.pos_2 = 0  ; pub_msg.pos_3 = 0  ; pub_msg.pos_4 = 0  ; pub_msg.pos_5 = 0  ; pub_msg.pos_6 = 0;
        pub_msg.vel_1 = 0.5; pub_msg.vel_2 = 0.5; pub_msg.vel_3 = 0.5; pub_msg.vel_4 = 0.5; pub_msg.vel_5 = 0.5; pub_msg.vel_6 = 0.5;

        pub->publish(pub_msg);
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    node->attachExecutor();
    node->run();
    rclcpp::shutdown();
    return 0;
}