#include <ros/ros.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <sensor_msgs/Joy.h>

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::Subscriber joy_sub;  // 手柄订阅者

    arm_control::ArmMsg msg;

public:
    TestNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TestNode::msgCallback, this);

        // 新增：订阅手柄话题
        joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TestNode::joyCallback, this);

        ROS_INFO("已启用手柄控制");

        // 初始化默认消息
        msg.Task_status = 1;
        msg.mode = 1;

        // ROS_INFO("输入格式：状态   模式   速度    转矩  位置1  位置2");
        // ROS_INFO("输入示例: 1     2     3.14   3.14  3.14  3.14");
    }

    // 订阅回调函数
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg) {
        // ROS_INFO("订阅成功\n ");
        // ROS_INFO("Received msg: ");
        // ROS_INFO("【电机1】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_1, msg->vel_1,msg->tor_1);
        // ROS_INFO("【电机2】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_2, msg->vel_2,msg->tor_2);
    }

    // 新增：手柄回调函数
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        /* 手柄按键映射示例（PS4布局）：
        axes[0]: 左摇杆左右 → 关节1位置
        axes[1]: 左摇杆上下 → 关节2位置
        buttons[0]: X按钮 → 启动/暂停 (0/1)
        buttons[1]: ○按钮 → 急停
        buttons[4]: L1 → 模式1 (MIT)
        buttons[5]: R1 → 模式2 (位置)
        */
        
        ROS_INFO("手柄 订阅成功\n ");

        // 1. 更新任务状态
        if (joy->buttons[2] || joy->buttons[2]) {        // ○按钮：急停
            msg.Task_status = 0; 
        } else if (joy->buttons[0] || joy->buttons[3]) { // X按钮：运行
            msg.Task_status = 1;
        }

        // 2. 更新控制模式
        if (joy->buttons[9]) {        // 左肩键: MIT模式
            msg.mode = 1;
        } else if (joy->buttons[10]) { // 右肩键: 位置模式
            msg.mode = 2;
        }

        // 3. 更新关节位置（摇杆值范围[-1,1] → 转换为[-π,π]）
        if(msg.mode == 1){
            msg.tor_1 = joy->axes[0]; 
            msg.tor_2 = joy->axes[0]; 
            msg.tor_3 = joy->axes[0]; 
            msg.tor_4 = joy->axes[0]; 
            msg.tor_5 = joy->axes[0]; 
            msg.tor_6 = joy->axes[0];  
        }
        else if(msg.mode == 2){
            msg.pos_1 = joy->axes[0]*M_PI; 
            msg.pos_2 = joy->axes[0]*M_PI; 
            msg.pos_3 = joy->axes[0]*M_PI; 
            msg.pos_4 = joy->axes[0]*M_PI; 
            msg.pos_5 = joy->axes[0]*M_PI; 
            msg.pos_6 = joy->axes[0]*M_PI; 
        }

        // 4. 发布更新后的指令
        pub.publish(msg);
        ROS_INFO("手柄指令已发布: 状态=%d, 模式=%d, 关节1=%.2f 关节2=%.2f 关节3=%.2f 关节4=%.2f 关节5=%.2f 关节6=%.2f", 
                msg.Task_status, msg.mode, msg.tor_1, msg.tor_2, msg.tor_3, msg.tor_4, msg.tor_5, msg.tor_6);
    }

    // 运行输入循环
    void run() {
        
        // int flag=0;

        // while(ros::ok()) {
        //     Simple_Arm::ArmMsg msg;
            
        //     // 获取用户输入
        //     std::cout << "\n按下【回车键】执行下一步操作）> ";
        //     std::string input;
        //     std::getline(std::cin, input);

        //     // // 解析输入数据
        //     // int task_status, mode;
        //     // float vel, tor, pos_1, pos_2;

        //     // std::istringstream iss(input);
        //     // try {
        //     //     iss >> task_status;      // 先提取为整数
        //     //     iss >> mode;             // 再提取为整数
        //     //     iss >> vel >> tor >> pos_1 >> pos_2;

        //     //     // 赋值到消息
        //     //     msg.Task_status = task_status;
        //     //     msg.mode = mode;
        //     //     msg.vel = vel;
        //     //     msg.tor = tor;
        //     //     msg.pos_1 = pos_1;
        //     //     msg.pos_2 = pos_2;
        //     // } catch (...) {
        //     //     ROS_ERROR("输入格式错误！");
        //     //     continue;
        //     // }
            
        //     if(flag == 0){
        //         // 赋值消息
        //         msg.Task_status = 1;
        //         msg.mode  = 2;
        //         msg.pos_1 = 3.14;
        //         msg.pos_2 = 3.14;
        //         msg.pos_3 = 3.14;
        //         msg.pos_4 = 3.14;
        //         msg.pos_5 = 3.14;
        //         msg.pos_6 = 3.14;
        //         msg.vel_1 = 1;
        //         msg.vel_2 = 1;
        //         msg.vel_3 = 1;
        //         msg.vel_4 = 1;
        //         msg.vel_5 = 1;
        //         msg.vel_6 = 1;
        //         msg.kp = 6;
        //         msg.kd = 0.6;

        //         flag++;
        //     }
        //     // else if(flag == 1){
        //     //     // 赋值消息
        //     //     msg.Task_status = 1;
        //     //     msg.mode  = 1;
        //     //     msg.pos_1 = 3.14*2/3;
        //     //     msg.pos_2 = 3.14*2/3;
        //     //     msg.vel_1 = 0;
        //     //     msg.vel_2 = 0;
        //     //     msg.tor_1 = 0.5;
        //     //     msg.tor_2 = 0.5;

        //     //     flag++;
        //     // }
        //     // else if(flag == 2){
        //     //     // 赋值消息
        //     //     msg.Task_status = 1;
        //     //     msg.mode  = 1;
        //     //     msg.pos_1 = 3.14/3;
        //     //     msg.pos_2 = 3.14/3;
        //     //     msg.vel_1 = 0;
        //     //     msg.vel_2 = 0;
        //     //     msg.tor_1 = 0.5;
        //     //     msg.tor_2 = 0.5;

        //     //     flag++;
        //     // }
        //     else if(flag == 1){
        //         // 赋值消息
        //         msg.Task_status = 1;
        //         msg.mode  = 2;
        //         msg.pos_1 = 0;
        //         msg.pos_2 = 0;
        //         msg.pos_3 = 0;
        //         msg.pos_4 = 0;
        //         msg.pos_5 = 0;
        //         msg.pos_6 = 0;
        //         msg.vel_1 = 1;
        //         msg.vel_2 = 1;
        //         msg.vel_3 = 1;
        //         msg.vel_4 = 1;
        //         msg.vel_5 = 1;
        //         msg.vel_6 = 1;
        //         msg.kp = 6;
        //         msg.kd = 0.6;

        //         flag = 0;
        //     }
            
        //     // 发布消息
        //     pub.publish(msg);
        //     ROS_INFO("消息已发布!");
        //     ROS_INFO("flag:%d 状态:%d 模式:%d 位置1:%.2f 速度1:%.2f 转矩1:%.2f 位置2:%.2f 速度2:%.2f 转矩2:%.2f"
        //             ,flag,msg.Task_status,msg.mode,msg.pos_1,msg.vel_1,msg.tor_1,msg.pos_2,msg.vel_2,msg.tor_2);

            // 处理ROS回调
            // ros::spinOnce();
            ros::spin();
            // 控制循环频率
            // usleep(10000);  // 10ms
        // }

    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "test_node");
    TestNode node;
    node.run();
    return 0;
}