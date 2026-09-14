#include <ros/ros.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include <cmath> // 包含cmath头文件以使用cosf函数

// 计算实时扭矩
float calculate_t(float L, float theta, float m, float g) {
    return L * cosf(theta) * m * g;
}

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    uint8_t Task_status_rx;
    uint8_t mode_rx;
    float pos_1_rx;
    float pos_2_rx;
    float vel_1_rx;
    float vel_2_rx;
    float tor_1_rx;
    float tor_2_rx;
    float kp_rx;
    float kd_rx;

    float L=0.15;
    float m=0.28;
    float g=9.8;

    arm_control::ArmMsg msg;
    
public:
    TestNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<arm_control::ArmMsg>("Arm_rx", 20, &TestNode::msgCallback, this);

        ROS_INFO("Pub_Test 启动成功 ");

        // // 开启电机力控模式
        // // 赋值消息
        // msg.Task_status = 1;
        // msg.mode  = 1;
        // msg.pos_1 = 3.14/2;
        // msg.pos_2 = 3.14/2;
        // msg.vel_1 = 0;
        // msg.vel_2 = 0;
        // msg.tor_1 = 0.2;
        // msg.tor_2 = 0.01;
        // msg.kp    = 0;
        // msg.kd    = 0;
        // // 发布消息
        // pub.publish(msg);
    }

    // 订阅回调函数
    void msgCallback(const arm_control::ArmMsg::ConstPtr& msg) {
        // ROS_INFO("订阅成功\n ");
        // ROS_INFO("Received msg: ");
        // ROS_INFO("【电机1】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_1, msg->vel_1,msg->tor_1);
        // ROS_INFO("【电机2】位置:%.2f 速度:%.2f 转矩:%.2f", 
        //         msg->pos_2, msg->vel_2,msg->tor_2);

        // Task_status_rx = msg->Task_status;
        // mode_rx  = msg->mode;
        // pos_1_rx = msg->pos_1;
        // pos_2_rx = msg->pos_2;
        // vel_1_rx = msg->vel_1;
        // vel_2_rx = msg->vel_2;
        // tor_1_rx = msg->tor_1;
        // tor_2_rx = msg->tor_2;
        // kp_rx = msg->kp;
        // kd_rx = msg->kd;
    }

    // 运行输入循环
    void run() {
        
        int flag=0;

        msg.Task_status = 1;
        msg.mode  = 1;

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

        msg.kp = 0;
        msg.kd = 0;

        // 发布消息
        pub.publish(msg);

        while(ros::ok()) {
            
            // 获取用户输入
            std::cout << "\n按下【回车键】执行下一步操作）> ";
            std::string input;
            std::getline(std::cin, input);

            // // 提供转矩 补偿重力
            // msg.Task_status = 1;
            // msg.mode  = 1;
            // msg.pos_1 = 3.14/2;
            // msg.pos_2 = 3.14/2;
            // msg.vel_1 = 0;
            // msg.vel_2 = 0;
            // msg.tor_1 = calculate_t(L,pos_1_rx,m,g);
            // msg.tor_2 = 0.01;
            // msg.kp    = 0;
            // msg.kd    = 0;

            msg.kp = 20;
            msg.kd = 5;

            // 全部移动至极限位置
            if(flag == 0){
                // 赋值消息
                msg.pos_6 = 0;

                flag++;
            }

            else if(flag == 1){
                // 赋值消息
                msg.pos_6 = 3;

                flag = 0;
            }

            // 发布消息
            pub.publish(msg);
            ROS_INFO("消息已发布!");
            // ROS_INFO("flag:%d 状态:%d 模式:%d 位置1:%.2f 速度1:%.2f 转矩1:%.2f 位置2:%.2f 速度2:%.2f 转矩2:%.2f"
            //         ,flag,msg.Task_status,msg.mode,msg.pos_1,msg.vel_1,msg.tor_1,msg.pos_2,msg.vel_2,msg.tor_2);
            ROS_INFO("flag:%d 状态:%d 模式:%d 位置6:%.2f"
                    ,flag,msg.Task_status,msg.mode,msg.pos_6);

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