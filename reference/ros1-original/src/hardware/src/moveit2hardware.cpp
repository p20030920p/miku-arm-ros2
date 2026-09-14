#include <ros/ros.h>
#include <arm_control/ArmMsg.h> // 自定义结构体消息头文件
#include "sensor_msgs/JointState.h"

class TestNode {
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;

    arm_control::ArmMsg arm_msg;
    float pos[6];

public:
    TestNode() {
        // 初始化发布者和订阅者
        pub = nh.advertise<arm_control::ArmMsg>("Arm_tx", 20);
        sub = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1000, &TestNode::jointstatesCallback, this);

        ROS_INFO("moveit_to_hardware 启动成功 ");
    }

    // 订阅回调函数
    void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
    {
        // pos=msg.position;
        pos[0]=msg->position[0];
        pos[1]=msg->position[1];
        pos[2]=msg->position[2];
        pos[3]=msg->position[3];
        pos[4]=msg->position[4];
        pos[5]=msg->position[5];
        // ROS_INFO("I heard: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
    }

    // 按照周期发布电机数据
    void run() {
        while(ros::ok()) {
            float vel = 0.5;

            arm_msg.Task_status = 1;
            arm_msg.mode  = 2;
            arm_msg.pos_1 = pos[0];
            arm_msg.pos_2 = pos[1];
            arm_msg.pos_3 = pos[2];
            arm_msg.pos_4 = pos[3];
            arm_msg.pos_5 = pos[4];
            arm_msg.pos_6 = pos[5];

            arm_msg.vel_1 = vel;
            arm_msg.vel_2 = vel;
            arm_msg.vel_3 = vel;
            arm_msg.vel_4 = vel;
            arm_msg.vel_5 = vel;
            arm_msg.vel_6 = vel;

            // 发布消息
            pub.publish(arm_msg);
            ROS_INFO("消息已发布!");
            ROS_INFO("位置1:%.2f 位置2:%.2f 位置3:%.2f 位置4:%.2f 位置5:%.2f 位置6:%.2f"
                    ,arm_msg.pos_1,arm_msg.pos_2,arm_msg.pos_3,arm_msg.pos_4,arm_msg.pos_5,arm_msg.pos_6);

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