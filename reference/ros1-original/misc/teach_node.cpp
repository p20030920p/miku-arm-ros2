#include <ros/ros.h>
#include <ros/package.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <ctime>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp> 
#include <kdl/chaindynparam.hpp>                 // 动力学（重力）求解

class TeachNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    // arm_control::ArmMsg pub_msg;

    // ----- 用于示教录制的参数 -----
    std::ofstream data_file;
    bool is_recording = false;
    arm_control::ArmMsg current_arm_state;
    ros::Time record_start_time;
    ros::Timer record_timer;
    ros::Time last_msg_time; // 记录上一条消息的时间
    bool initial_data_error_logged = false; // 控制初始错误只报一次
    bool data_update_warning_logged = false; // 控制更新警告只报一次
    // --------------------------

    // ------- 用于重力补偿 -------
    // KDL相关
    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    KDL::JntArray q_recv; // 接收当前关节角
    // 重力补偿
    KDL::ChainDynParam* dyn_solver;   // 动力学求解器（包含重力项）
    KDL::JntArray tau_g;              // 当前姿态下的重力补偿扭矩
    // --------------------------

public:
    TeachNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TeachNode::msgCallback, this);

        // 创建一个定时器用于以固定频率记录数据，但先不启动
        record_timer = nh.createTimer(ros::Duration(0.001), &TeachNode::recordCallback, this, false); // 100Hz

        ROS_INFO("示教节点启动成功。");

        // 读取URDF
        urdf::Model model;
        model.initFile(ros::package::getPath("arm_control") + "/urdf/miku_dummy.urdf");

        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
            ROS_ERROR("Failed to construct KDL tree");
            ros::shutdown();
        }

        // 构造KDL链
        kdl_tree.getChain("base_link", "link_6", kdl_chain); // 根据你的URDF修改起止link名


        // 动力学求解器（重力），注意重力方向
        // 若 base_link 的 Z 轴向上，则使用 (0, 0, -9.81)
        dyn_solver = new KDL::ChainDynParam(kdl_chain, KDL::Vector(0.0, 0.0, -9.81));

        // 接收当前关节角
        q_recv = KDL::JntArray(kdl_chain.getNrOfJoints());

        // 重力扭矩数组初始化
        tau_g = KDL::JntArray(kdl_chain.getNrOfJoints());
    }
    ~TeachNode() {
        delete dyn_solver; // 释放动力学求解器
    }
  
    // 更新重力补偿扭矩（保存在 tau_g 成员中）
    void updateGravityTorques(const KDL::JntArray& q) {
        if (!dyn_solver) return;
        int ret = dyn_solver->JntToGravity(q, tau_g);
        if (ret < 0) {
            ROS_WARN("重力扭矩计算失败（可能 URDF 惯量不完整或链不连续）");
        }
    }
    
    // 订阅回调函数，用于实时更新机械臂状态
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg_in) {
        // ROS_INFO("订阅成功\n ");
        
        // 将收到的最新消息存储在成员变量中
        current_arm_state = *msg_in;
        last_msg_time = ros::Time::now(); // 更新收到消息的时间
        data_update_warning_logged = false; // 收到数据，重置警告状态

        // 回读当前关节角
        q_recv(0) = msg_in->pos_1;
        q_recv(1) = msg_in->pos_2;
        q_recv(2) = msg_in->pos_3;
        q_recv(3) = msg_in->pos_4;
        q_recv(4) = msg_in->pos_5;
        q_recv(5) = msg_in->pos_6;

        // 随当前关节状态更新重力补偿
        updateGravityTorques(q_recv);

        arm_control::ArmMsg msg_out;
        // 发布关节控制
        msg_out.Task_status = 1; 
        msg_out.mode = 1; // 1-mit 2-速度位置        
        msg_out.kp = 0;
        msg_out.kd = 0;

        // 赋值关节角度
        msg_out.pos_1 = 0;
        msg_out.pos_2 = 0;
        msg_out.pos_3 = 0;
        msg_out.pos_4 = 0;
        msg_out.pos_5 = 0;
        msg_out.pos_6 = 0;

        // 速度、力矩可根据实际需求赋值
        msg_out.vel_1 = 0;
        msg_out.vel_2 = 0;
        msg_out.vel_3 = 0;
        msg_out.vel_4 = 0;
        msg_out.vel_5 = 0;
        msg_out.vel_6 = 0;

        // 重力补偿
        msg_out.tor_1 = tau_g(0) * 0.5; 
        msg_out.tor_2 = tau_g(1) * 0.5;
        msg_out.tor_3 = tau_g(2) * 0.56;
        msg_out.tor_4 = tau_g(3) * 0.7;
        msg_out.tor_5 = tau_g(4) * 0.8;
        msg_out.tor_6 = tau_g(5) * 0.8;

        // 发布消息
        pub.publish(msg_out);
        // ROS_INFO("消息已发布!");
    }

    // 定时器回调函数，用于将数据写入文件
    void recordCallback(const ros::TimerEvent&) {
        if (!is_recording) return;

        ros::Time now = ros::Time::now();
        double time_since_last_msg = (now - last_msg_time).toSec();

        // 检查1：开始录制后，如果长时间没收到数据，则报错
        if (last_msg_time.is_zero()) { // is_zero() 表示从未收到过消息
            if (time_since_last_msg > 2.0 && !initial_data_error_logged) {
                ROS_ERROR("开始录制后超过2秒未收到任何机械臂数据，请检查 'Arm_rx' 话题！");
                initial_data_error_logged = true; // 确保只报错一次
            }
            return; // 没有数据，不执行写入
        }

        // 检查2：录制过程中，如果数据长时间不更新，则告警
        if (time_since_last_msg > 2.0 && !data_update_warning_logged) {
            ROS_WARN("机械臂数据超过2秒未更新！");
            data_update_warning_logged = true; // 确保只告警一次，直到下次数据恢复
        }

        // 写入文件
        if (!data_file.is_open()) return;
        ros::Duration time_since_start = now - record_start_time;
        data_file << std::fixed << std::setprecision(4) << time_since_start.toSec() << " "
                  << current_arm_state.pos_1 << " " << current_arm_state.pos_2 << " "
                  << current_arm_state.pos_3 << " " << current_arm_state.pos_4 << " "
                  << current_arm_state.pos_5 << " " << current_arm_state.pos_6 << " "
                  << current_arm_state.vel_1 << " " << current_arm_state.vel_2 << " "
                  << current_arm_state.vel_3 << " " << current_arm_state.vel_4 << " "
                  << current_arm_state.vel_5 << " " << current_arm_state.vel_6 << std::endl;

        // 打印已写入数据，便于调试
        ROS_INFO("\nRecorded data at %.4f s: \n  pos: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n  vel: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  time_since_start.toSec(),
                  current_arm_state.pos_1, current_arm_state.pos_2,
                  current_arm_state.pos_3, current_arm_state.pos_4,
                  current_arm_state.pos_5, current_arm_state.pos_6,
                  current_arm_state.vel_1, current_arm_state.vel_2,
                  current_arm_state.vel_3, current_arm_state.vel_4,
                  current_arm_state.vel_5, current_arm_state.vel_6);
    }

    // 运行主循环，控制录制的开始和停止
    void run() {
        while(ros::ok()) {
            if (!is_recording) {
                std::cout << "\n按下【回车键】开始录制...> ";
            } else {
                std::cout << "\n正在录制... 按下【回车键】停止录制...> ";
            }
            
            std::string input;
            std::getline(std::cin, input);
            if (std::cin.get() == EOF){   // stdin 结束（后台启动）→ 退出，别空转
                break;
            }
            // 切换录制状态
            is_recording = !is_recording;

            if (is_recording) {
                // --- 开始录制 ---
                // pub.publish(pub_msg);
                ROS_INFO("\n开始录制，可以手动拖动机械臂进行示教。");

                // 获取包路径并拼接文件夹路径
                std::string package_path = ros::package::getPath("arm_control");
                std::string folder_path = package_path + "/teach_path/";

                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
                std::string filename = ss.str() + ".txt";
                std::string full_path = folder_path + filename;

                data_file.open(full_path);
                if (!data_file.is_open()) {
                    ROS_ERROR("无法创建文件: %s", full_path.c_str());
                    is_recording = false; // 无法打开文件，重置状态
                    continue;
                }
                
                data_file << "# " << ss.str() << std::endl;
                data_file << "# timestamp pos1 pos2 pos3 pos4 pos5 pos6 vel1 vel2 vel3 vel4 vel5 vel6" << std::endl;
                
                // 重置状态
                last_msg_time = ros::Time(0); // 重置上次消息时间
                initial_data_error_logged = false;
                data_update_warning_logged = false;
                record_start_time = ros::Time::now();
                record_timer.start(); // 启动定时器
                ROS_INFO("开始录制数据到文件: %s", full_path.c_str());

            } else {
                // --- 停止录制 ---
                record_timer.stop(); // 停止定时器
                if (data_file.is_open()) {
                    data_file.close();
                    ROS_INFO("录制结束。");
                }

            }
        }
        // 确保程序退出时文件已关闭
        if (data_file.is_open()) {
            data_file.close();
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "teach_node");
    TeachNode node;

    // 启动多线程 ROS 回调
    ros::AsyncSpinner spinner(2); // 2个线程，可根据需要调整
    spinner.start();

    node.run();
    return 0;
}