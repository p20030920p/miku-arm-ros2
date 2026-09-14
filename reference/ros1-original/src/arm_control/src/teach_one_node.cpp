/* ====================================================================
 * teach_one_node.cpp
 * 示教/悬停节点：纯重力补偿（kp=0 纯力矩模式），可手拖机械臂
 * 按回车打印当前末端位姿（正运动学）
 *
 * 依赖封装：
 *   KinematicsSolver   (kinematics_solver.h)   建链 + 正运动学打印
 *   GravityCompensator (gravity_compensator.h) 重力补偿
 * （不需要 LinearPlanner：本节点不做直线运动）
 * ==================================================================== */

#include <ros/ros.h>
#include <ros/package.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <sensor_msgs/JointState.h> // 用于发布关节状态到RViz#include <iostream>
#include <iostream>
#include <cstdio>

#include <kdl/frames.hpp>       // KDL::Frame / Rotation / Vector

#include "kinematics_solver.h"
#include "gravity_compensator.h"

class TeachOneNode {
private:
    /* ---------- ROS 相关 ---------- */
    ros::NodeHandle nh;
    ros::Publisher pub;             // 发布 Arm_tx（下发给 hardware）
    ros::Subscriber sub;            // 订阅 Arm_rx（hardware 回读）
    ros::Publisher joint_state_pub; // 发布 /joint_states（给 RViz）

    /* ---------- 封装模块 ---------- */
    KinematicsSolver   kinematics_;   // 运动学（建链 + FK 打印）
    GravityCompensator gravity_;      // 重力补偿

    /* ---------- 关节角缓冲 ---------- */
    KDL::JntArray q_recv_;            // 回读的实际关节角

    /* ---------- 最近一次状态（供 run() 按回车打印） ---------- */
    arm_control::ArmMsg current_arm_state;

public:
    TeachOneNode() {
        // 1. 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20,
                                                &TeachOneNode::msgCallback, this);
        joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // 2. 运动学：加载新模型 URDF，构建 base_link → link_6 链
        std::string urdf_path =
            ros::package::getPath("arm_control") + "/urdf/miku_dummy.urdf";
        if (!kinematics_.loadURDF(urdf_path, "base_link", "link_6"))
        {
            ROS_ERROR("运动学初始化失败，退出");
            ros::shutdown();
        }

        // 3. 重力补偿：用 KDL 链创建求解器，并设置补偿系数
        gravity_.init(kinematics_.chain());

        // 从参数服务器读取重力补偿系数（由 config/arm_control.yaml 提供）
        std::vector<double> gains;
        if (nh.getParam("gravity_gains", gains) && gains.size() == 6)
        {
            gravity_.setGains(gains);
            ROS_INFO("已从参数读取重力补偿系数: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    gains[0], gains[1], gains[2], gains[3], gains[4], gains[5]);
        }
        else
        {
            // 读不到就兜底（比如直接 rosrun 没走 launch）
            gravity_.setGains({0.5, 0.55, 0.61, 0.6, 0.7, 0.3});
            ROS_WARN("未读到 gravity_gains，使用默认系数");
        }

        // 4. 关节角缓冲分配空间
        q_recv_ = KDL::JntArray(kinematics_.jointCount());

        ROS_INFO("示教节点启动成功");
    }

    /* ==================================================================
     * msgCallback：收到回读 → 更新重力补偿 → 下发（纯力矩模式）
     * ================================================================== */
    // 订阅回调函数，用于实时更新机械臂状态
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg_in) {
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
        arm_control::ArmMsg msg_out;
        // 发布关节控制
        msg_out.Task_status = 1; 
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
        pub.publish(msg_out);

        // 6. 发布 JointState 供 RViz 显示（真实关节角）
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = {"joint_1", "joint_2", "joint_3",
                   "joint_4", "joint_5", "joint_6"};
        js.position = {msg_in->pos_1, msg_in->pos_2, msg_in->pos_3,
                       msg_in->pos_4, msg_in->pos_5, msg_in->pos_6};
        joint_state_pub.publish(js);
    }

    /* ==================================================================
     * run：按回车 → 打印关节角 + 正运动学末端位姿
     * ================================================================== */
    // 运行主循环，控制录制的开始和停止
    void run() {
        while(ros::ok()) {
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
    ros::init(argc, argv, "teach_one_node");

    TeachOneNode node;

    // 启动多线程 ROS 回调
    ros::AsyncSpinner spinner(2); // 2个线程，可根据需要调整
    spinner.start();

    node.run();
    return 0;
}