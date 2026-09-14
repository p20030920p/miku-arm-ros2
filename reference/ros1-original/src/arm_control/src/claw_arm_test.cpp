/* ====================================================================
 * claw_arm_test.cpp
 * 夹爪 + 机械臂同步控制节点
 *   - 机械臂：重力补偿 + 姿态保持（pos=回读值，不被拉走）
 *   - 夹爪  ：ClawController 封装（参数从 config/claw_controller.yaml 读取）
 * 主逻辑模式：mode=1 (MIT)，kp/kd 统一设置
 * ==================================================================== */

#include <ros/ros.h>
#include <ros/package.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <sensor_msgs/JointState.h> // 用于发布关节状态到RViz
#include <iostream>
#include <cstdio>

#include <kdl/frames.hpp>       // KDL::Frame / Rotation / Vector

#include "kinematics_solver.h"
#include "gravity_compensator.h"
#include "claw_controller.h"

/* ------------------------------------------------------------------
 * 从参数服务器读取夹爪参数（config/claw_controller.yaml 提供）
 * 读不到的项保持 ClawParams 里的默认值
 * ------------------------------------------------------------------ */
ClawParams loadClawParams(ros::NodeHandle& nh)
{
    ClawParams p;
    nh.getParam("claw/open_pos_deg",   p.open_pos_deg);
    nh.getParam("claw/open_step_deg",  p.open_step_deg);
    nh.getParam("claw/open_torque",    p.open_torque);
    nh.getParam("claw/close_pos_deg",  p.close_pos_deg);
    nh.getParam("claw/close_step_deg", p.close_step_deg);
    nh.getParam("claw/close_torque",   p.close_torque);
    nh.getParam("claw/hold_step_deg",  p.hold_step_deg);
    nh.getParam("claw/hold_torque",    p.hold_torque);
    nh.getParam("claw/catch_torque",   p.catch_torque);
    return p;
}

class ClawArmNode {
private:
    /* ---------- ROS 相关 ---------- */
    ros::NodeHandle nh;
    ros::Publisher pub;             // 发布 Arm_tx（下发给 hardware）
    ros::Subscriber sub;            // 订阅 Arm_rx（hardware 回读）
    ros::Publisher joint_state_pub; // 发布 /joint_states（给 RViz）

    /* ---------- 封装模块 ---------- */
    KinematicsSolver   kinematics_;   // 运动学（建链 + FK 打印）
    GravityCompensator gravity_;      // 重力补偿
    ClawController     claw_;          // 夹爪状态机封装

    /* ---------- 关节角缓冲 ---------- */
    KDL::JntArray q_recv_;            // 回读的实际关节角

    /* ---------- 最近一次状态（供 run() 打印用） ---------- */
    arm_control::ArmMsg current_arm_state;

public:
    ClawArmNode() {
        // 1. 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20,
                                                &ClawArmNode::msgCallback, this);
        joint_state_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // 2. 运动学：加载新模型 URDF，构建 base_link → link_6 链
        std::string urdf_path =
            ros::package::getPath("arm_control") + "/urdf/miku_dummy.urdf";
        if (!kinematics_.loadURDF(urdf_path, "base_link", "link_6"))
        {
            ROS_ERROR("运动学初始化失败，退出");
            ros::shutdown();
        }

        // 3. 重力补偿：从 yaml 读系数（兜底默认）
        gravity_.init(kinematics_.chain());
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

        // 4. 夹爪参数：从 config/claw_controller.yaml 读取（读不到用默认值）
        ClawParams claw_p = loadClawParams(nh);
        claw_.setParams(claw_p);
        ROS_INFO("夹爪参数: 张开%0.1f°/%0.1f°步进, 夹紧%0.1f°/%0.1f°步进, 阈值%0.2fN·m",
                 claw_p.open_pos_deg, claw_p.open_step_deg,
                 claw_p.close_pos_deg, claw_p.close_step_deg,
                 claw_p.catch_torque);

        // 5. 关节角缓冲分配空间
        q_recv_ = KDL::JntArray(kinematics_.jointCount());

        ROS_INFO("claw_arm_test 启动成功，回车切换夹爪 张开/夹紧");
    }

    /* ==================================================================
     * msgCallback：收到回读 → 更新重力补偿 → 下发
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
        msg_out.kp          = 6.0;     // 零刚度，允许手拖
        msg_out.kd          = 0.6;

        // ---- 机械臂（主逻辑）：目标=当前回读，保持姿态，不掉不拉走 ----
        msg_out.pos_1 = msg_in->pos_1;  msg_out.pos_2 = msg_in->pos_2;  msg_out.pos_3 = msg_in->pos_3;
        msg_out.pos_4 = msg_in->pos_4;  msg_out.pos_5 = msg_in->pos_5;  msg_out.pos_6 = msg_in->pos_6;
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

        // ---- 夹爪（电机7）：一行调用状态机 ----
        claw_.update(msg_in->pos_7, msg_in->tor_7,
                     msg_out.pos_7, msg_out.vel_7, msg_out.tor_7);

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
     * run：按回车 → 切换夹爪状态（张开/夹紧）
     * ================================================================== */
    // 运行主循环，控制录制的开始和停止
    void run() {
        while (ros::ok()) {
            std::cout << "\n按回车执行下一步> ";
            if (std::cin.get() == EOF) {   // stdin 结束（后台启动）→ 退出
                break;
            }

            // 回车：张开 ↔ 夹紧 切换（测试用）
            if (claw_.state() == ClawController::CLAW_OPEN)
                claw_.close();
            else
                claw_.open();

            printf("→ %s\n", claw_.stateName());
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "claw_arm_test");

    ClawArmNode node;

    // 启动多线程 ROS 回调
    ros::AsyncSpinner spinner(2); // 2个线程，可根据需要调整
    spinner.start();

    node.run();
    return 0;
}