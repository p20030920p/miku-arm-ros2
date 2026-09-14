/*
    【控制】自制moveit测试版 v0.1
    1.计算正逆运动学方程
    2.运动数据发布给仿真
    3.按下回车键切换位姿
*/

#include <ros/ros.h>
#include <cmath> 
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include "sensor_msgs/JointState.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    trajectory_msgs::JointTrajectory traj;

    // 最终期望矩阵参数
    double n_x,o_x,a_x,p_x;
    double n_y,o_y,a_y,p_y;
    double n_z,o_z,a_z,p_z;
    
    double theta[7],d[7],a[7],alpha[7]; // 共6个关节，为了可读性，省略数组0号位

    // 回读电机位置
    float pos[6];
    // 状态控制标志
    int flag;

public:
    TestNode() {
        // 初始化发布者和订阅者
        // 实机
        // pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        // sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TestNode::msgCallback, this);
        // 仿真
        pub = nh.advertise<trajectory_msgs::JointTrajectory>("/manipulator_controller/command",10);
        sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, &TestNode::jointstatesCallback, this);

        // 设置关节名称（必须与URDF和控制器配置一致）
        std::vector<std::string> joint_names = {
            "joint_1", "joint_2", "joint_3", 
            "joint_4", "joint_5", "joint_6"
        };

        traj.joint_names = joint_names;
        traj.points.resize(1);  // 单点轨迹

        ROS_INFO("my_moveit_test 启动成功 ");
        
        // 初始化DH建模参数 (单位:米)
        // d 数组
        d[0] = 0;      // 忽略位，不参与计算
        d[1] = 0;      // 0-1
        d[2] = 0;      // 1-2   
        d[3] = 0;      // 2-3
        d[4] = 0;      // 3-4
        d[5] = 0;      // 4-5
        d[6] = 0;      // 5-H
        
        // a 数组 (毫米转米)
        a[0] = 0;          // 忽略位，不参与计算
        a[1] = 0;          // 0-1
        a[2] = 184/1000.0; // 1-2 (184mm -> 0.184m)  
        a[3] = 184/1000.0; // 2-3 (184mm -> 0.184m)
        a[4] = 81.5/1000.0; // 3-4 (81.5mm -> 0.0815m)
        a[5] = 0;          // 4-5
        a[6] = 0;          // 5-H
        
        // alpha 数组 (度转弧度)
        alpha[0] = 0;                 // 忽略位，不参与计算
        alpha[1] = 90 * M_PI/180.0;   // 0-1
        alpha[2] = 180 * M_PI/180.0;  // 1-2   
        alpha[3] = 180 * M_PI/180.0;  // 2-3
        alpha[4] = -90 * M_PI/180.0;  // 3-4
        alpha[5] = 90 * M_PI/180.0;   // 4-5
        alpha[6] = 0;                 // 5-H

        // 初始化位置
        for (int i = 0; i < 6; i++) {
            pos[i] = 0.0f;
        }
        
        // 状态控制标志
        flag=0;
    }

    // 订阅回调函数（实机）
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg) {
        // ROS_INFO("订阅成功\n ");
        // ROS_INFO("Received msg: ");
        // ROS_INFO("【电机1】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_1, msg->vel_1,msg->tor_1);
        // ROS_INFO("【电机2】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_2, msg->vel_2,msg->tor_2);
    }

    // 订阅回调函数（仿真）
    void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
    {
        for (int i = 0; i < 6; ++i) {
            pos[i] = msg->position[i];
        }
        // ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
    }

    // 限定关节角度 (根据关节索引)
    double armLimit(int joint_id, double angle) {
        // 关节角度限制 (弧度)
        const double joint_limits[6][2] = {
            {-M_PI, M_PI},     // 关节1
            {-M_PI/2, M_PI/2}, // 关节2
            {-M_PI/2, M_PI/2}, // 关节3
            {-M_PI/2, M_PI/2}, // 关节4
            {-M_PI/2, M_PI/2}, // 关节5
            {-M_PI/2, M_PI/2}  // 关节6
        };
        
        // 手动实现clamp功能
        double min_limit = joint_limits[joint_id][0];
        double max_limit = joint_limits[joint_id][1];
        
        if (angle < min_limit) {
            return min_limit;
        } else if (angle > max_limit) {
            return max_limit;
        } else {
            return angle;
        }
    }

    // 计算逆运动解
    bool armInvCalc(void) {
        // 单位转换: 毫米转米 (Gazebo使用米为单位)
        double px_m = p_x / 1000.0;
        double py_m = p_y / 1000.0;
        double pz_m = p_z / 1000.0;
        
        // 关节1 (使用原始位置)
        theta[1] = atan2(py_m, px_m);
        
        // 关节2+3+4的和
        double theta_234 = atan2(a_z, cos(theta[1])*a_x + sin(theta[1])*a_y);
        
        // 关节3余弦
        double term1 = px_m*cos(theta[1]) + py_m*sin(theta[1]) - cos(theta_234)*a[4];
        double term2 = pz_m - sin(theta_234)*a[4];
        double cos_3 = (term1*term1 + term2*term2 - a[2]*a[2] - a[3]*a[3]) / (2*a[2]*a[3]);
        
        // 检查cos_3是否在有效范围内
        if (fabs(cos_3) > 1.0) {
            ROS_ERROR("逆运动学无解! cos_3=%.4f 超出范围", cos_3);
            return false;
        }
        
        // 关节3双解处理 (选择最接近当前位置的解)
        double sin_3_pos = sqrt(1 - cos_3*cos_3);
        double sin_3_neg = -sqrt(1 - cos_3*cos_3);
        
        // 计算两种可能的关节3角度
        double theta3_pos = atan2(sin_3_pos, cos_3);
        double theta3_neg = atan2(sin_3_neg, cos_3);
        
        // 选择最接近当前位置的解
        double current_joint3 = pos[2]; // 假设位置索引2对应关节3
        if (fabs(theta3_pos - current_joint3) <= fabs(theta3_neg - current_joint3)) {
            theta[3] = theta3_pos;
        } else {
            theta[3] = theta3_neg;
        }
        
        // 关节2（修正括号和atan2参数）
        double num2 = (a[3]*cos(theta[3]) + a[2]) * term2 - a[3]*sin(theta[3]) * term1;
        double den2 = (a[3]*cos(theta[3]) + a[2]) * term1 + a[3]*sin(theta[3]) * term2;
        theta[2] = atan2(num2, den2);
        
        // 关节4
        theta[4] = theta_234 - theta[2] - theta[3];
        
        // 关节5（修正分母计算）
        double num5 = cos(theta_234)*(cos(theta[1])*a_x + sin(theta[1])*a_y) + sin(theta_234)*a_z;
        double den5 = sin(theta[1])*a_x - cos(theta[1])*a_y; // 修正分母
        theta[5] = atan2(num5, den5);
        
        // 关节6（修正参数）
        double num6 = -sin(theta_234)*(cos(theta[1])*n_x + sin(theta[1])*n_y) + cos(theta_234)*n_z;
        double den6 = -sin(theta_234)*(cos(theta[1])*o_x + sin(theta[1])*o_y) + cos(theta_234)*o_z;
        theta[6] = atan2(num6, den6);
        
        // 关节角度限幅
        theta[1] = armLimit(0, theta[1]); // 关节1
        theta[2] = armLimit(1, theta[2]); // 关节2
        theta[3] = armLimit(2, theta[3]); // 关节3
        theta[4] = armLimit(3, theta[4]); // 关节4
        theta[5] = armLimit(4, theta[5]); // 关节5
        theta[6] = armLimit(5, theta[6]); // 关节6

        ROS_INFO("------关节位置------");
        ROS_INFO("电机1:%.4f rad (%.2f deg)", theta[1], theta[1]*180.0/M_PI);
        ROS_INFO("电机2:%.4f rad (%.2f deg)", theta[2], theta[2]*180.0/M_PI);
        ROS_INFO("电机3:%.4f rad (%.2f deg)", theta[3], theta[3]*180.0/M_PI);
        ROS_INFO("电机4:%.4f rad (%.2f deg)", theta[4], theta[4]*180.0/M_PI);
        ROS_INFO("电机5:%.4f rad (%.2f deg)", theta[5], theta[5]*180.0/M_PI);
        ROS_INFO("电机6:%.4f rad (%.2f deg)", theta[6], theta[6]*180.0/M_PI);
        
        return true;
    }

    // 运行输入循环
    void run() {
        while(ros::ok()) {
            // arm_control::ArmMsg msg;
            sensor_msgs::JointState msg;
            
            // 获取用户输入
            std::cout << "\n<<按下【回车键】执行下一步操作>> ";
            std::string input;
            std::getline(std::cin, input);
            double vel = 0.5;
            double acc = 0;

            // 状态1
            if(flag == 0){
                // 定义目标位姿 (单位: mm)
                n_x = 1;
                n_y = 0;
                n_z = 0;

                o_x = 0;
                o_y = 1;
                o_z = 0;

                a_x = 0;
                a_y = 0;
                a_z = 1;

                p_x = -50;
                p_y = -50;
                p_z = 380;

                flag++;
            }
            // 状态2
            else if(flag == 1){
                
                // 定义目标位姿 (单位: mm)
                n_x = 1;
                n_y = 0;
                n_z = 0;

                o_x = 0;
                o_y = 1;
                o_z = 0;

                a_x = 0;
                a_y = 0;
                a_z = 1;

                p_x = 30;
                p_y = 30;
                p_z = 280;

                flag = 0;
            }
            
            // 计算逆解，失败则跳过
            if(!TestNode::armInvCalc()) {
                ROS_WARN("跳过此位姿");
                continue;
            }

            // 设置目标位置（单位：弧度）    
            traj.points[0].positions = {
                theta[1],  // joint1角度
                theta[2],  // joint2角度
                theta[3],  // joint3角度
                theta[4],  // joint4角度
                theta[5],  // joint5角度
                theta[6]   // joint6角度
            };
                
            // 添加速度和加速度约束
            traj.points[0].velocities = {vel, vel, vel, vel, vel, vel};
            traj.points[0].accelerations = {acc, acc, acc, acc, acc, acc};
            traj.points[0].time_from_start = ros::Duration(2.0);  // 到达目标的时间

            // 发布消息
            pub.publish(traj);
            ROS_INFO("消息已发布!");

            // 处理ROS回调
            ros::spinOnce();
            // 控制循环频率
            usleep(10000);  // 10ms
        }
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "test_node");
    TestNode node;
    node.run();
    return 0;
}