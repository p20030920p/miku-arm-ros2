// ============================================================================
// sim_motor_board.cpp —— 无串口的电机仿真节点（Gazebo 可视化 + 端到端测试用）
//
// 与 virtual_motor_board.py 的区别：
//   virtual_motor_board.py : 站在**下位机**一侧，用真实串口协议，
//                            用来测 hardware 节点的串口链路。
//   sim_motor_board (本文件): 直接订阅 /Arm_tx（跳过串口），模拟电机响应后
//                            发布 /joint_states 与 Gazebo 的位置指令，
//                            用来在没有硬件时驱动仿真机械臂、验证上层算法。
//
// 行为对齐 hardware.cpp：
//   - mode=2 速度位置模式：位置以 |vel| 为上限一阶趋近目标
//   - mode=1 MIT 模式：kp 拉向目标 + tor 力矩前馈（并叠加重力负载，
//     这样"上层的重力补偿是否正确"在仿真里也能看出来）
//   - 回发 /Arm_rx，让 arm_control / go_home / claw_arm_test 形成闭环
//   - 夹到物体时夹爪位置卡住、力矩上升（供 ClawController 判定）
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <arm_control/msg/arm_msg.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace {

constexpr int   kNJoints   = 6;      // 机械臂关节数
constexpr int   kNMotors   = 7;      // 含夹爪
constexpr double kPi       = 3.14159265358979323846;
constexpr double kGripperLower = -165.0 * kPi / 180.0;

// 简化重力负载（仅关节 2、3 明显），用于让"重力补偿是否生效"在数据上可辨别
double gravityLoad(int idx1based)
{
    if (idx1based == 2) return 0.45;
    if (idx1based == 3) return 0.10;
    return 0.0;
}

class SimMotorBoard : public rclcpp::Node {
public:
    SimMotorBoard() : Node("sim_motor_board")
    {
        joint_names_ = {"joint_1", "joint_2", "joint_3",
                        "joint_4", "joint_5", "joint_6"};

        // 是否把关节角送到 Gazebo（gz_ros2_control 的 forward_position_controller）
        gz_topic_ = this->declare_parameter<std::string>(
            "gz_command_topic", "");
        gravity_load_ = this->declare_parameter<bool>("gravity_load", true);
        grip_object_  = this->declare_parameter<bool>("grip_object", true);
        double rate_hz = this->declare_parameter<double>("rate", 100.0);

        pub_rx_ = this->create_publisher<arm_control::msg::ArmMsg>("Arm_rx", 20);
        pub_js_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 20);

        if (!gz_topic_.empty()) {
            pub_gz_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(gz_topic_, 20);
            RCLCPP_INFO(this->get_logger(), "Gazebo 位置指令话题: %s", gz_topic_.c_str());
        }

        sub_tx_ = this->create_subscription<arm_control::msg::ArmMsg>(
            "Arm_tx", 50,
            std::bind(&SimMotorBoard::onArmTx, this, std::placeholders::_1));

        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&SimMotorBoard::step, this));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(),
                    "sim_motor_board 启动（无串口仿真）。等待 /Arm_tx ...");
    }

private:
    // ------------------------------------------------------------------
    void onArmTx(const arm_control::msg::ArmMsg::SharedPtr msg)
    {
        cmd_ = *msg;
        got_cmd_ = true;
    }

    // ------------------------------------------------------------------
    void step()
    {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0 || dt > 0.5) dt = 0.01;   // 首帧/异常时兜底

        if (!got_cmd_) {
            publishState(dt);
            return;
        }

        const std::array<double, kNMotors> cmd_pos = {
            cmd_.pos_1, cmd_.pos_2, cmd_.pos_3, cmd_.pos_4,
            cmd_.pos_5, cmd_.pos_6, cmd_.pos_7};
        const std::array<double, kNMotors> cmd_vel = {
            cmd_.vel_1, cmd_.vel_2, cmd_.vel_3, cmd_.vel_4,
            cmd_.vel_5, cmd_.vel_6, cmd_.vel_7};
        const std::array<double, kNMotors> cmd_tor = {
            cmd_.tor_1, cmd_.tor_2, cmd_.tor_3, cmd_.tor_4,
            cmd_.tor_5, cmd_.tor_6, cmd_.tor_7};

        for (int i = 0; i < kNMotors; ++i) {
            const bool is_gripper = (i == kNJoints);

            if (cmd_.mode == 2) {
                // ---- 速度位置模式：一阶趋近，速度受 |cmd_vel| 限制 ----
                double vmax = std::fabs(cmd_vel[i]);
                if (vmax < 1e-6) vmax = 0.5;   // 真实电机位置模式也需给定速度
                const double err = cmd_pos[i] - pos_[i];
                const double step = std::max(-vmax * dt, std::min(vmax * dt, err));
                pos_[i] += step;
                vel_[i] = step / dt;
                tor_[i] = 0.0;
            } else {
                // ---- MIT 模式：kp 拉向目标 + 力矩前馈 ----
                const double err = cmd_pos[i] - pos_[i];
                double tau = cmd_.kp * err + cmd_tor[i];
                if (gravity_load_ && !is_gripper) tau -= gravityLoad(i + 1);
                const double acc = tau - cmd_.kd * vel_[i] * 10.0;
                vel_[i] += acc * dt * 20.0;
                pos_[i] += vel_[i] * dt;
                tor_[i] = cmd_tor[i];
            }

            if (is_gripper) {
                // 夹爪：夹到物体则位置卡住、力矩上升；否则受物理限位
                if (grip_object_ && pos_[i] <= -1.2) {
                    pos_[i] = -1.2;
                    vel_[i] = 0.0;
                    tor_[i] = 0.6;
                } else if (pos_[i] < kGripperLower) {
                    pos_[i] = kGripperLower;
                    vel_[i] = 0.0;
                } else if (pos_[i] > 0.0) {
                    pos_[i] = 0.0;
                    vel_[i] = 0.0;
                }
            } else {
                // 关节软限位（±3.2 rad，与虚拟驱动板一致）
                if (pos_[i] < -3.2) { pos_[i] = -3.2; vel_[i] = 0.0; }
                if (pos_[i] >  3.2) { pos_[i] =  3.2; vel_[i] = 0.0; }
            }
        }

        publishState(dt);
    }

    // ------------------------------------------------------------------
    void publishState(double /*dt*/)
    {
        // ---- /joint_states（给 RViz / Gazebo / robot_state_publisher）----
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = joint_names_;
        js.position.assign(pos_.begin(), pos_.begin() + kNJoints);
        js.velocity.assign(vel_.begin(), vel_.begin() + kNJoints);
        js.effort.assign(tor_.begin(), tor_.begin() + kNJoints);
        pub_js_->publish(js);

        // ---- /Arm_rx（回读帧，形成闭环）----
        arm_control::msg::ArmMsg rx;
        rx.task_status = cmd_.task_status;
        rx.mode = cmd_.mode;
        rx.kp = cmd_.kp;
        rx.kd = cmd_.kd;
        rx.pos_1 = pos_[0]; rx.vel_1 = vel_[0]; rx.tor_1 = tor_[0];
        rx.pos_2 = pos_[1]; rx.vel_2 = vel_[1]; rx.tor_2 = tor_[1];
        rx.pos_3 = pos_[2]; rx.vel_3 = vel_[2]; rx.tor_3 = tor_[2];
        rx.pos_4 = pos_[3]; rx.vel_4 = vel_[3]; rx.tor_4 = tor_[3];
        rx.pos_5 = pos_[4]; rx.vel_5 = vel_[4]; rx.tor_5 = tor_[4];
        rx.pos_6 = pos_[5]; rx.vel_6 = vel_[5]; rx.tor_6 = tor_[5];
        rx.pos_7 = pos_[6]; rx.vel_7 = vel_[6]; rx.tor_7 = tor_[6];
        pub_rx_->publish(rx);

        // ---- Gazebo 位置指令 ----
        if (pub_gz_) {
            std_msgs::msg::Float64MultiArray cmd;
            cmd.data.assign(pos_.begin(), pos_.begin() + kNJoints);
            pub_gz_->publish(cmd);
        }
    }

    // ------------------------------------------------------------------
    std::vector<std::string> joint_names_;
    std::array<double, kNMotors> pos_{};
    std::array<double, kNMotors> vel_{};
    std::array<double, kNMotors> tor_{};

    arm_control::msg::ArmMsg cmd_;
    bool got_cmd_ = false;

    std::string gz_topic_;
    bool gravity_load_ = true;
    bool grip_object_ = true;
    rclcpp::Time last_time_;

    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub_rx_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_gz_;
    rclcpp::Subscription<arm_control::msg::ArmMsg>::SharedPtr sub_tx_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimMotorBoard>());
    rclcpp::shutdown();
    return 0;
}
