/* ====================================================================
 * teach_one_node.cpp —— ROS 2 Jazzy 移植版
 * 示教/悬停节点：纯重力补偿（kp=0 纯力矩模式），可手拖机械臂
 * 按回车打印当前末端位姿（正运动学）
 *
 * 依赖封装：
 *   KinematicsSolver   (kinematics_solver.h)   建链 + 正运动学打印
 *   GravityCompensator (gravity_compensator.h) 重力补偿
 * （不需要 LinearPlanner：本节点不做直线运动）
 * ==================================================================== */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件
#include <sensor_msgs/msg/joint_state.hpp> // 用于发布关节状态到RViz
#include <iostream>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <kdl/frames.hpp>       // KDL::Frame / Rotation / Vector

#include "kinematics_solver.h"
#include "gravity_compensator.h"

class TeachOneNode : public rclcpp::Node {
private:
    /* ---------- ROS 相关 ---------- */
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;                 // 发布 Arm_tx（下发给 hardware）
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub;              // 订阅 Arm_rx（hardware 回读）
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub; // 发布 /joint_states（给 RViz）

    /* ---------- 封装模块 ---------- */
    KinematicsSolver   kinematics_;   // 运动学（建链 + FK 打印）
    GravityCompensator gravity_;      // 重力补偿

    /* ---------- 关节角缓冲 ---------- */
    KDL::JntArray q_recv_;            // 回读的实际关节角

    /* ---------- 最近一次状态（供 run() 按回车打印） ---------- */
    arm_control::msg::ArmMsg current_arm_state;

public:
    TeachOneNode() : Node("teach_one_node") {
        // 1. 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<arm_control::msg::ArmMsg>(
            "Arm_rx", 20,
            std::bind(&TeachOneNode::msgCallback, this, std::placeholders::_1));
        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // 2. 运动学：加载新模型 URDF，构建 base_link → link_6 链
        std::string urdf_path =
            ament_index_cpp::get_package_share_directory("arm_control") + "/urdf/miku_dummy.urdf";
        if (!kinematics_.loadURDF(urdf_path, "base_link", "link_6"))
        {
            RCLCPP_ERROR(this->get_logger(), "运动学初始化失败，退出");
            rclcpp::shutdown();
        }

        // 3. 重力补偿：用 KDL 链创建求解器，并设置补偿系数
        gravity_.init(kinematics_.chain());

        // ROS 2 参数：先声明再读取（默认值与 config/arm_control.yaml 一致）
        this->declare_parameter<std::vector<double>>(
            "gravity_gains", std::vector<double>{0.5, 0.55, 0.61, 0.6, 0.7, 0.3});
        std::vector<double> gains = this->get_parameter("gravity_gains").as_double_array();

        if (gains.size() == 6)
        {
            gravity_.setGains(gains);
            RCLCPP_INFO(this->get_logger(),
                        "已从参数读取重力补偿系数: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                        gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
        }
        else
        {
            // 读不到就兜底（比如直接运行节点没走 launch）
            gravity_.setGains({0.5, 0.55, 0.61, 0.6, 0.7, 0.3});
            RCLCPP_WARN(this->get_logger(), "未读到 gravity_gains，使用默认系数");
        }

        // 4. 关节角缓冲分配空间
        q_recv_ = KDL::JntArray(kinematics_.jointCount());

        RCLCPP_INFO(this->get_logger(), "示教节点启动成功");
    }

    /* ==================================================================
     * msgCallback：收到回读 → 更新重力补偿 → 下发（纯力矩模式）
     * ================================================================== */
    void msgCallback(const arm_control::msg::ArmMsg::SharedPtr msg_in) {
        // 1. 保存最近一次状态（供 run() 打印用）
        current_arm_state = *msg_in;

        // 2. 回读当前 6 个关节角
        q_recv_(0) = msg_in->pos_1;
        q_recv_(1) = msg_in->pos_2;
        q_recv_(2) = msg_in->pos_3;
        q_recv_(3) = msg_in->pos_4;
        q_recv_(4) = msg_in->pos_5;
        q_recv_(5) = msg_in->pos_6;

        // 3. 随当前姿态更新重力补偿力矩
        gravity_.update(q_recv_);

        // 4. 组装下发消息（示教模式：kp=0、kd=0、位置给0 → 纯力矩悬停）
        arm_control::msg::ArmMsg msg_out;
        // 发布关节控制
        msg_out.task_status = 1;
        msg_out.mode        = 1;     // 1-mit 2-速度位置
        msg_out.kp          = 0;     // 零刚度，允许手拖
        msg_out.kd          = 0;

        // 赋值关节角度
        msg_out.pos_1 = 0;  msg_out.pos_2 = 0;  msg_out.pos_3 = 0;
        msg_out.pos_4 = 0;  msg_out.pos_5 = 0;  msg_out.pos_6 = 0;

        // 速度、力矩可根据实际需求赋值
        msg_out.vel_1 = 0;  msg_out.vel_2 = 0;  msg_out.vel_3 = 0;
        msg_out.vel_4 = 0;  msg_out.vel_5 = 0;  msg_out.vel_6 = 0;

        // 重力补偿力矩 = 重力矩 × 系数
        std::vector<double> tor = gravity_.compensatedTorques();
        msg_out.tor_1 = tor[0];
        msg_out.tor_2 = tor[1];
        msg_out.tor_3 = tor[2];
        msg_out.tor_4 = tor[3];
        msg_out.tor_5 = tor[4];
        msg_out.tor_6 = tor[5];

        // 5. 发布控制指令
        pub->publish(msg_out);

        // 6. 发布 JointState 供 RViz 显示（真实关节角）
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = {"joint_1", "joint_2", "joint_3",
                   "joint_4", "joint_5", "joint_6"};
        js.position = {msg_in->pos_1, msg_in->pos_2, msg_in->pos_3,
                       msg_in->pos_4, msg_in->pos_5, msg_in->pos_6};
        joint_state_pub->publish(js);
    }

    /* ==================================================================
     * run：按回车 → 打印关节角 + 正运动学末端位姿
     * ================================================================== */
    void run() {
        while(rclcpp::ok()) {
            std::cout << "\n按下【回车键】显示当前位姿> ";
            if (std::cin.get() == EOF){   // stdin 结束（后台启动）→ 退出，别空转
                break;
            }

            // 打印回读的关节角
            printf("  pos: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
                  current_arm_state.pos_1, current_arm_state.pos_2,
                  current_arm_state.pos_3, current_arm_state.pos_4,
                  current_arm_state.pos_5, current_arm_state.pos_6);

            // 打印正运动学末端位姿（用封装的正解）
            KDL::Frame fk = kinematics_.solveFK(q_recv_);
            double r, p, y;
            fk.M.GetRPY(r, p, y);
            printf("实际末端位置: x=%.3f, y=%.3f, z=%.3f\n",
                   fk.p.x(), fk.p.y(), fk.p.z());
            printf("实际末端姿态: R=%.3f, P=%.3f, Y=%.3f\n", r, p, y);
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TeachOneNode>();

    // 启动多线程 ROS 回调（对应原来的 ros::AsyncSpinner(2)）
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    node->run();

    rclcpp::shutdown();
    if (spin_thread.joinable()) {
        spin_thread.join();
    }
    return 0;
}
