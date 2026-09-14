#include <ros/ros.h>
#include <ros/package.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <thread> // 引入线程库

// 用于存储轨迹中单个点的结构体
struct TrajectoryPoint {
    double timestamp;
    double positions[6];
    double velocities[6];
};

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    arm_control::ArmMsg pub_msg;

    std::vector<TrajectoryPoint> trajectory; // 用于存储从文件加载的轨迹

public:
    TestNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TestNode::msgCallback, this);

        ROS_INFO("轨迹复现节点启动成功。");
    }

    // 订阅回调函数
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg) {
        // ROS_INFO("订阅成功\n ");
        // 可以用来监视机械臂的实际位置与指令位置的差异
    }

    // 从文件加载轨迹数据
    bool loadTrajectory(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            ROS_ERROR("无法打开轨迹文件: %s", filepath.c_str());
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
                ROS_WARN("解析行失败，已跳过: %s", line.c_str());
                continue;
            }
            trajectory.push_back(point);
        }

        file.close();
        if (trajectory.empty()) {
            ROS_ERROR("未从文件中加载任何有效的轨迹点。");
            return false;
        }
        
        ROS_INFO("成功加载 %zu 个轨迹点。", trajectory.size());
        return true;
    }
    
    // 运行主循环，实现轨迹复现
    void run() {
        // 1. 获取用户输入的文件名
        std::cout << "请输入 'teach_path' 文件夹中的轨迹文件名 (例如: 2025-08-28_14-42-35):\n";
        std::string filename;
        std::getline(std::cin, filename);

        // 2. 构建完整文件路径并加载
        std::string package_path = ros::package::getPath("hardware");
        if (package_path.empty()) {
            ROS_FATAL("找不到功能包 'hardware'。请检查工作区环境。");
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
        ROS_INFO("开始执行轨迹复现...");
        ros::Time start_time = ros::Time::now();
        ros::Rate rate(200); // 以200Hz的频率循环，以确保时间戳的精确触发
        size_t current_point_index = 0;

        while (ros::ok() && current_point_index < trajectory.size()) {
            ros::Duration elapsed_time = ros::Time::now() - start_time;

            // 检查当前经过的时间是否已经到达或超过下一个轨迹点的时间戳
            if (elapsed_time.toSec() >= trajectory[current_point_index].timestamp) {
                arm_control::ArmMsg msg;
                msg.Task_status = 1;
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

                pub.publish(msg);
                ROS_INFO("发布点 %zu, 时间戳: %.4f", current_point_index, trajectory[current_point_index].timestamp);
                // 打印六个关节的位置和速度
                ROS_INFO("位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                         msg.pos_1, msg.pos_2, msg.pos_3,
                         msg.pos_4, msg.pos_5, msg.pos_6);
                ROS_INFO("速度: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                         msg.vel_1, msg.vel_2, msg.vel_3,
                         msg.vel_4, msg.vel_5, msg.vel_6);
                current_point_index++; // 移动到下一个点
            }
            ros::spinOnce();
            rate.sleep();
        }
        
        ROS_INFO("轨迹执行完毕。");
        // 延时一段时间，确保最后的命令被执行
        ros::Duration(1.0).sleep();
        // 发布机械臂使能消息，机械臂恢复到初始位置
        pub_msg.Task_status = 1;
        pub_msg.mode  = 2;
        pub_msg.pos_1 = 0  ; pub_msg.pos_2 = 0  ; pub_msg.pos_3 = 0  ; pub_msg.pos_4 = 0  ; pub_msg.pos_5 = 0  ; pub_msg.pos_6 = 0;
        pub_msg.vel_1 = 0.5; pub_msg.vel_2 = 0.5; pub_msg.vel_3 = 0.5; pub_msg.vel_4 = 0.5; pub_msg.vel_5 = 0.5; pub_msg.vel_6 = 0.5;

        pub.publish(pub_msg);
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "trajectory_track_node");
    TestNode node;
    node.run();
    return 0;
}