#include <ros/ros.h>
#include <ros/package.h> 
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <sensor_msgs/JointState.h>
#include <iostream>

#include <kdl/frames.hpp>       // KDL::Frame / Rotation / Vector

#include "kinematics_solver.h"
#include "gravity_compensator.h"
#include "linear_planner.h"

class ArmControlNode {
private:
    /* ---------- ROS 相关 ---------- */
    ros::NodeHandle nh;
    ros::Publisher  pub;               // 发布 Arm_tx（下发给 hardware） 
    ros::Subscriber sub;               // 订阅 Arm_rx（hardware 回读）
    ros::Publisher  joint_state_pub;   // 发布 /joint_states（给 RViz）

    /* ---------- 三个封装模块 ---------- */
    KinematicsSolver   kinematics_;    // 正逆运动学
    GravityCompensator gravity_;   // 重力补偿
    LinearPlanner      planner_;   // 直线规划

    /* ---------- 关节角缓冲 ---------- */
    KDL::JntArray q_init_;   // IK 的起始种子
    KDL::JntArray q_sol_;    // IK 求出的解（要下发的目标关节角）
    KDL::JntArray q_recv_;   // 硬件回读的实际关节角

    /* ---------- 位姿 ---------- */
    Pose6 cur;    // 当前目标末端位姿（直线运动中被逐步推进）
    Pose6 goal;   // 键盘输入的目标位姿

    /* ---------- 控制参数 ---------- */
    double target_speed_ = 0.005;   // 直线运动速度 (m/s)

public:
    /* ==================================================================
     * 构造函数：装配三个模块 + ROS 收发
     * ================================================================== */
    ArmControlNode() {
        // 1. 初始化发布者和订阅者（实机）
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20,
                                                &ArmControlNode::msgCallback, this);
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

        // 4. 关节角缓冲分配空间，全部置 0
        q_init_ = KDL::JntArray(kinematics_.jointCount());
        q_sol_  = KDL::JntArray(kinematics_.jointCount());
        q_recv_ = KDL::JntArray(kinematics_.jointCount());
        for (unsigned int i = 0; i < q_init_.rows(); ++i)
        {
            q_init_(i) = 0.0;
            q_sol_(i)  = 0.0;
        }

        // 5. 初始末端位姿（原点正上方 0.5m，姿态水平）
        cur.R = 0.0;  cur.P = 0.0;  cur.Y = 0.0;
        cur.x = 0.0;  cur.y = 0.0;  cur.z = 0.5;

        ROS_INFO("arm_control_node 启动成功");
    }

    /* ==================================================================
     * msgCallback：收到硬件回读 → 更新重力补偿 → 下发控制指令
     * ================================================================== */
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg_in) {
        // 1. 回读当前 6 个关节角到 q_recv_
        q_recv_(0) = msg_in->pos_1;
        q_recv_(1) = msg_in->pos_2;
        q_recv_(2) = msg_in->pos_3;
        q_recv_(3) = msg_in->pos_4;
        q_recv_(4) = msg_in->pos_5;
        q_recv_(5) = msg_in->pos_6;

        // 2. 随当前姿态更新重力补偿力矩
        gravity_.update(q_recv_);


        // 3. 组装下发消息
        arm_control::ArmMsg msg_out;
        msg_out.Task_status = 1;
        msg_out.mode       = 1;      // 1-mit 模式
        msg_out.kp         = 6.0;
        msg_out.kd         = 0.6;

        // 目标关节角 = 逆解结果 q_sol_
        msg_out.pos_1 = q_sol_(0);
        msg_out.pos_2 = q_sol_(1);
        msg_out.pos_3 = q_sol_(2);
        msg_out.pos_4 = q_sol_(3);
        msg_out.pos_5 = q_sol_(4);
        msg_out.pos_6 = q_sol_(5);

        // 速度设为 0（不做速度控制）
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

        // 4. 发布控制指令
        pub.publish(msg_out);

        // 5. 发布 JointState 供 RViz 显示（真实关节角）
        sensor_msgs::JointState js;
        js.header.stamp = ros::Time::now();
        js.name = {"joint_1", "joint_2", "joint_3",
                   "joint_4", "joint_5", "joint_6"};
        js.position = {msg_in->pos_1, msg_in->pos_2, msg_in->pos_3,
                       msg_in->pos_4, msg_in->pos_5, msg_in->pos_6};
        joint_state_pub.publish(js);
    }

    /* ==================================================================
     * run：主循环（状态机：等待输入 → 直线推进 → 到达结束）
     * ================================================================== */
    // 运行输入循环
    void run() {
        ros::Rate loop_rate(5); // Hz

        while(ros::ok()) {
            /* ---------- 阶段一：等待输入 或 直线推进 ---------- */
            if (!planner_.isExecuting()) {
                // ---- 等待键盘输入目标位姿 ----
                std::cout << "\n请输入目标位姿: R P Y x y z (回车)，EOF退出: ";
                if (!(std::cin >> goal.R >> goal.P >> goal.Y
                               >> goal.x >> goal.y >> goal.z)) {
                    std::cout << "\n输入结束/失败，退出。\n";
                    break;
                }
                
                // 起点 = 当前末端位姿，目标 = 刚输入的值，速度用默认值
                planner_.setTarget(cur, goal, target_speed_);

                // 用当前实测关节角作为 IK 种子，避免跳变
                q_init_ = q_recv_;

                std::cout << "开始规划直线段 -> 目标: RPY("
                          << goal.R << " " << goal.P << " " << goal.Y
                          << ") XYZ(" << goal.x << " " << goal.y << " " << goal.z << ")\n";
            } else {
                // ---- 正在直线推进 ----
                double dt = loop_rate.expectedCycleTime().toSec();
                bool done = planner_.update(dt, cur);   // cur 更新为这一帧的位姿

                if (done) {
                    // 到达目标：对最终位姿做一次严格 IK 校验
                    KDL::Frame final_pose(
                        KDL::Rotation::RPY(cur.R, cur.P, cur.Y),
                        KDL::Vector(cur.x, cur.y, cur.z));

                    KDL::JntArray sol;
                    if (kinematics_.solveIK(q_recv_, final_pose, sol))
                    {
                        q_sol_ = sol;
                        std::cout << "到达目标。\n";
                    }
                    else
                    {
                        ROS_WARN("最终目标 IK 失败，保持当前姿态");
                    }
                } else {
                    // 还没到：对当前应到位姿求解逆解
                    KDL::Frame target_pose(
                        KDL::Rotation::RPY(cur.R, cur.P, cur.Y),
                        KDL::Vector(cur.x, cur.y, cur.z));

                    if (kinematics_.solveIK(q_init_, target_pose, q_sol_))
                    {
                        q_init_ = q_sol_;   // 用上一解作种子，保证连续平滑
                    }
                    else
                    {
                        ROS_ERROR("IK 失败，本步跳过 (x=%.3f y=%.3f z=%.3f)",
                                  cur.x, cur.y, cur.z);
                    }
                }
            }

            /* ---------- 阶段二：未执行时（等待输入/刚完成）延时并打印正运动学 ---------- */
            if (!planner_.isExecuting()) {
                ros::Duration(3.0).sleep();  // 延时

                // 打印当前实际末端位姿（正运动学）
                KDL::Frame fk = kinematics_.solveFK(q_recv_);
                double r, p, y;
                fk.M.GetRPY(r, p, y);
                ROS_INFO("实际末端位置: x=%.3f, y=%.3f, z=%.3f",
                         fk.p.x(), fk.p.y(), fk.p.z());
                ROS_INFO("实际末端姿态: R=%.3f, P=%.3f, Y=%.3f", r, p, y);

                q_init_ = q_recv_;
            }

            loop_rate.sleep();
        }

    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "arm_control_node");

    ArmControlNode node;

    // 启动多线程 ROS 回调
    ros::AsyncSpinner spinner(2); // 2个线程，可根据需要调整
    spinner.start();

    node.run();
    return 0;
}
