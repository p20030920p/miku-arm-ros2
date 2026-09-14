#include <ros/ros.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

public:
    TestNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TestNode::msgCallback, this);

        ROS_INFO("Pub_Test 启动成功 ");
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

    // 运行输入循环
    void run() {
        
        int flag=0;

        while(ros::ok()) {
            arm_control::ArmMsg msg;
            
            // 获取用户输入
            std::cout << "\n按下【回车键】执行下一步操作）> ";
            std::string input;
            std::getline(std::cin, input);

            // // 解析输入数据
            // int task_status, mode;
            // float vel, tor, pos_1, pos_2;

            // std::istringstream iss(input);
            // try {
            //     iss >> task_status;      // 先提取为整数
            //     iss >> mode;             // 再提取为整数
            //     iss >> vel >> tor >> pos_1 >> pos_2;

            //     // 赋值到消息
            //     msg.Task_status = task_status;
            //     msg.mode = mode;
            //     msg.vel = vel;
            //     msg.tor = tor;
            //     msg.pos_1 = pos_1;
            //     msg.pos_2 = pos_2;
            // } catch (...) {
            //     ROS_ERROR("输入格式错误！");
            //     continue;
            // }
            float vel = 0.5;
            
            // 全部移动至极限位置
            if(flag == 0){
                // 赋值消息
                msg.Task_status = 1; // 机械臂使能
                msg.mode  = 1;       // 机械臂无力矩模式

                msg.pos_1 = 0;
                msg.pos_2 = -M_PI/2;
                msg.pos_3 = -M_PI/2;
                msg.pos_4 = -M_PI/2;
                msg.pos_5 = -M_PI/2;
                msg.pos_6 = -M_PI/2;

                msg.vel_1 = vel;
                msg.vel_2 = vel;
                msg.vel_3 = vel;
                msg.vel_4 = vel;
                msg.vel_5 = vel;
                msg.vel_6 = vel;

                // msg.tor_1 = 0.01;
                // msg.tor_2 = 0.01;
                // msg.tor_3 = 0.01;
                // msg.tor_4 = 0.01;
                // msg.tor_5 = 0.01;
                // msg.tor_6 = 0.01;

                // 夹爪
                // msg.pos_7 = -M_PI/2;
                // msg.vel_7 = vel;
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0.0;

                // msg.kp = 6;
                // msg.kd = 0.6;

                msg.kp = 0;
                msg.kd = 0;

                flag++;
            }
            // 全部移动到零点
            else if(flag == 1){
                // 赋值消息
                msg.Task_status = 1; // 机械臂使能
                msg.mode  = 2;       // 机械臂位置模式

                msg.pos_1 = 0;
                msg.pos_2 = 0;
                msg.pos_3 = 0;
                msg.pos_4 = 0;
                msg.pos_5 = 0;
                msg.pos_6 = 0;

                msg.vel_1 = vel;
                msg.vel_2 = vel;
                msg.vel_3 = vel;
                msg.vel_4 = vel;
                msg.vel_5 = vel;
                msg.vel_6 = vel;

                // msg.tor_1 = 0.01;
                // msg.tor_2 = 0.01;
                // msg.tor_3 = 0.01;
                // msg.tor_4 = 0.01;
                // msg.tor_5 = 0.01;
                // msg.tor_6 = 0.01;
                
                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = vel;
                // msg.tor_7 = 0.01;

                msg.kp = 6;
                msg.kd = 0.6;

                flag=0;
            }
            // 无力矩模式
            else if(flag == 2){
                // 赋值消息
                msg.Task_status = 1; // 机械臂使能
                msg.mode  = 1;       // 机械臂无力矩模式

                msg.pos_1 = 0;
                msg.pos_2 = 0;
                msg.pos_3 = 0;
                msg.pos_4 = 0;
                msg.pos_5 = 0;
                msg.pos_6 = 0;

                msg.vel_1 = 0;
                msg.vel_2 = 0;
                msg.vel_3 = 0;
                msg.vel_4 = 0;
                msg.vel_5 = 0;
                msg.vel_6 = 0;

                msg.tor_1 = 0;
                msg.tor_2 = 0;
                msg.tor_3 = 0;
                msg.tor_4 = 0;
                msg.tor_5 = 0;
                msg.tor_6 = 0;

                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0;

                msg.kp = 0;
                msg.kd = 0;

                flag ++;
            }
            // 全部再次移动到零点
            else if(flag == 3){
                // 赋值消息
                msg.Task_status = 1; // 机械臂使能
                msg.mode  = 2;       // 机械臂位置模式

                msg.pos_1 = 0;
                msg.pos_2 = 0;
                msg.pos_3 = 0;
                msg.pos_4 = 0;
                msg.pos_5 = 0;
                msg.pos_6 = 0;

                msg.vel_1 = vel;
                msg.vel_2 = vel;
                msg.vel_3 = vel;
                msg.vel_4 = vel;
                msg.vel_5 = vel;
                msg.vel_6 = vel;

                // msg.tor_1 = 0.01;
                // msg.tor_2 = 0.01;
                // msg.tor_3 = 0.01;
                // msg.tor_4 = 0.01;
                // msg.tor_5 = 0.01;
                // msg.tor_6 = 0.01;

                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = vel;
                // msg.tor_7 = 0.01;

                msg.kp = 6;
                msg.kd = 0.6;

                flag++;
            }
            // 机械臂失能
            else if(flag == 4){
                // 赋值消息
                msg.Task_status = 0; // 机械臂失能
                msg.mode  = 1;       // 机械臂无力矩模式
                msg.pos_1 = 0;
                msg.pos_2 = 0;
                msg.pos_3 = 0;
                msg.pos_4 = 0;
                msg.pos_5 = 0;
                msg.pos_6 = 0;

                msg.vel_1 = 0;
                msg.vel_2 = 0;
                msg.vel_3 = 0;
                msg.vel_4 = 0;
                msg.vel_5 = 0;
                msg.vel_6 = 0;

                msg.tor_1 = 0;
                msg.tor_2 = 0;
                msg.tor_3 = 0;
                msg.tor_4 = 0;
                msg.tor_5 = 0;
                msg.tor_6 = 0;

                // 夹爪
                msg.pos_7 = 0;
                msg.vel_7 = 0;
                msg.tor_7 = 0;

                msg.kp = 0;
                msg.kd = 0;

                flag = 0;
            }
            // 发布消息
            pub.publish(msg);
            ROS_INFO("消息已发布!,flag=%d",flag);
            ROS_INFO("位置1:%.2f 位置2:%.2f 位置3:%.2f 位置4:%.2f 位置5:%.2f 位置6:%.2f 位置7:%.2f"
                    ,msg.pos_1,msg.pos_2,msg.pos_3,msg.pos_4,msg.pos_5,msg.pos_6,msg.pos_7);
            ROS_INFO("【夹爪】模式:%d 位置:%.2f° 速度:%.4f 转矩:%.4f",
                        msg.mode,
                        msg.pos_7 / M_PI * 180,   // 弧度转角度，更直观
                        msg.vel_7,
                        msg.tor_7);
                        
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