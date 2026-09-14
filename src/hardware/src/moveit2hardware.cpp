/* ====================================================================
 * moveit2hardware.cpp —— ROS 2 Jazzy 移植版
 *
 * 作用：把 MoveIt（或任何发布 /joint_states 的规划器）给出的关节角，
 *       以"位置模式"(mode=2) 转发给 hardware 节点（Arm_tx），
 *       从而驱动实机运动。
 *
 * ROS 2 差异：
 *   - 节点类继承 rclcpp::Node；publisher/subscription 用 SharedPtr
 *   - ros::spinOnce() → 单线程 executor 的 spin_some()
 *   - 回调形参 const sensor_msgs::JointState::SharedPtr& → SharedPtr
 * ==================================================================== */

#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp> // 自定义结构体消息头文件
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <memory>

class TestNode : public rclcpp::Node {
private:
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;

    // 单线程 executor：主循环里手动 spin_some()，等价于 ROS 1 的 ros::spinOnce()
    rclcpp::executors::SingleThreadedExecutor executor_;

    arm_control::msg::ArmMsg arm_msg;
    float pos[6];

public:
    TestNode() : rclcpp::Node("test_node") {
        // 初始化发布者和订阅者
        pub = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);
        sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1000,
            std::bind(&TestNode::jointstatesCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "moveit_to_hardware 启动成功 ");
    }

    // 把本节点挂到内部 executor（必须在 run() 之前调用一次）
    void attachExecutor()
    {
        executor_.add_node(this->shared_from_this());
    }

    // 订阅回调函数
    void jointstatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // pos=msg.position;
        pos[0]=msg->position[0];
        pos[1]=msg->position[1];
        pos[2]=msg->position[2];
        pos[3]=msg->position[3];
        pos[4]=msg->position[4];
        pos[5]=msg->position[5];
        // RCLCPP_INFO(this->get_logger(), "I heard: [%f] [%f] [%f] [%f] [%f] [%f]",pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
    }

    // 按照周期发布电机数据
    void run() {
        while(rclcpp::ok()) {
            float vel = 0.5;

            arm_msg.task_status = 1;
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
            pub->publish(arm_msg);
            RCLCPP_INFO(this->get_logger(), "消息已发布!");
            RCLCPP_INFO(this->get_logger(),
                    "位置1:%.2f 位置2:%.2f 位置3:%.2f 位置4:%.2f 位置5:%.2f 位置6:%.2f",
                    arm_msg.pos_1,arm_msg.pos_2,arm_msg.pos_3,arm_msg.pos_4,arm_msg.pos_5,arm_msg.pos_6);

            // 处理ROS回调
            executor_.spin_some();
            // 控制循环频率
            rclcpp::sleep_for(std::chrono::milliseconds(10));  // 10ms
        }

    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL,"");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestNode>();
    node->attachExecutor();
    node->run();
    rclcpp::shutdown();
    return 0;
}
