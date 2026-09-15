// ============================================================================
// trajectory_bridge.cpp —— 把 MoveIt 规划出的轨迹转成机械臂指令
//
// 为什么需要它：
//   MoveIt 的 fake controller manager 只在内部推进轨迹、更新 /joint_states，
//   并不会驱动真实（或仿真）的机械臂。本节点订阅 MoveIt 执行控制器的话题，
//   把轨迹点按时间插值后以 mode=2（限速位置模式）发给 sim_motor_board 或
//   hardware，于是 RViz 里按 Execute 时机械臂真的会动。
//
// 数据流：
//   move_group ──/miku_arm_controller/follow_joint_trajectory──▶ 本节点
//   本节点 ──ArmMsg(mode=2) @ 50 Hz──▶ /Arm_tx ──▶ sim_motor_board / hardware
//
// 参数：
//   trajectory_topic  订阅的轨迹话题（默认 /miku_arm_controller/follow_joint_trajectory）
//   publish_rate      ArmMsg 下发频率（默认 50 Hz）
//   speed_scale       轨迹速度缩放，演示用可调慢（默认 1.0）
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <arm_control/msg/arm_msg.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

namespace {

constexpr int    kNJoints = 6;
constexpr double kDt      = 1.0 / 50.0;   // 固定步长，与下发频率一致
constexpr double kKp      = 6.0;
constexpr double kKd      = 0.6;

struct Point {
    double pos[kNJoints];
    double vel[kNJoints];
    double t;                              // 相对轨迹起点的秒数
};

}  // namespace

class TrajectoryBridge : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    TrajectoryBridge() : Node("trajectory_bridge")
    {
        const std::string topic = this->declare_parameter<std::string>(
            "trajectory_topic", "/miku_arm_controller/follow_joint_trajectory");
        rate_hz_ = this->declare_parameter<double>("publish_rate", 50.0);
        speed_scale_ = this->declare_parameter<double>("speed_scale", 1.0);

        pub_ = this->create_publisher<arm_control::msg::ArmMsg>("Arm_tx", 20);

        // MoveIt 的控制器句柄以 action 形式下发轨迹，因此这里必须是 action server，
        // 否则 move_group 会报 "Unable to identify any set of controllers"。
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this, topic,
            std::bind(&TrajectoryBridge::onGoal, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&TrajectoryBridge::onCancel, this, std::placeholders::_1),
            std::bind(&TrajectoryBridge::onAccepted, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate_hz_),
            std::bind(&TrajectoryBridge::tick, this));

        RCLCPP_INFO(this->get_logger(),
                    "trajectory_bridge 就绪：action %s，下发 %.0f Hz，速度缩放 %.2f",
                    topic.c_str(), rate_hz_, speed_scale_);
    }

private:
    // ---------------------------------------------------------------- action
    rclcpp_action::GoalResponse onGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        if (goal->trajectory.points.empty()) {
            RCLCPP_WARN(this->get_logger(), "收到空轨迹，拒绝");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "收到轨迹：%zu 个点，开始执行",
                    goal->trajectory.points.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse onCancel(const std::shared_ptr<GoalHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "轨迹被取消");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void onAccepted(const std::shared_ptr<GoalHandle> handle)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        buildTrajectory(handle->get_goal()->trajectory);
        handle_ = handle;
        elapsed_ = 0.0;
        finished_ = false;
    }

    // ---------------------------------------------------------------- 轨迹
    void buildTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
    {
        points_.clear();
        for (const auto& p : traj.points) {
            Point pt{};
            for (int i = 0; i < kNJoints && i < static_cast<int>(p.positions.size()); ++i) {
                pt.pos[i] = p.positions[i];
                pt.vel[i] = (i < static_cast<int>(p.velocities.size()))
                                ? p.velocities[i] : 0.0;
            }
            pt.t = static_cast<double>(p.time_from_start.sec) +
                   static_cast<double>(p.time_from_start.nanosec) * 1e-9;
            points_.push_back(pt);
        }
    }

    // 按时间在轨迹点上线性插值
    std::array<double, kNJoints> sample(double t) const
    {
        std::array<double, kNJoints> out{};
        if (points_.empty()) {
            return out;
        }
        if (t <= points_.front().t) {
            for (int i = 0; i < kNJoints; ++i) out[i] = points_.front().pos[i];
            return out;
        }
        for (size_t k = 1; k < points_.size(); ++k) {
            if (t <= points_[k].t) {
                const Point& a = points_[k - 1];
                const Point& b = points_[k];
                const double span = b.t - a.t;
                const double u = (span > 1e-9) ? (t - a.t) / span : 1.0;
                for (int i = 0; i < kNJoints; ++i) {
                    out[i] = a.pos[i] + (b.pos[i] - a.pos[i]) * u;
                }
                return out;
            }
        }
        for (int i = 0; i < kNJoints; ++i) out[i] = points_.back().pos[i];
        return out;
    }

    // ---------------------------------------------------------------- 主循环
    void tick()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!handle_ || finished_ || points_.empty()) {
            return;
        }

        const auto target = sample(elapsed_);
        publish(target);
        elapsed_ += kDt * speed_scale_;

        const double total = points_.back().t;
        if (elapsed_ >= total) {
            publish(sample(total));
            finished_ = true;
            auto result = std::make_shared<FollowJointTrajectory::Result>();
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            handle_->succeed(result);
            RCLCPP_INFO(this->get_logger(), "轨迹执行完成（%.2f s）", total);
            handle_.reset();
        }
    }

    void publish(const std::array<double, kNJoints>& q)
    {
        arm_control::msg::ArmMsg msg;
        msg.task_status = 1;
        msg.mode = 2;                 // 限速位置模式
        msg.kp = kKp;
        msg.kd = kKd;

        msg.pos_1 = q[0];  msg.pos_2 = q[1];  msg.pos_3 = q[2];
        msg.pos_4 = q[3];  msg.pos_5 = q[4];  msg.pos_6 = q[5];
        msg.vel_1 = 1.0;   msg.vel_2 = 1.0;   msg.vel_3 = 1.0;
        msg.vel_4 = 1.0;   msg.vel_5 = 1.0;   msg.vel_6 = 1.0;
        msg.pos_7 = 0.0;   msg.vel_7 = 0.0;   msg.tor_7 = 0.0;
        pub_->publish(msg);
    }

    // ---------------------------------------------------------------- 成员
    rclcpp::Publisher<arm_control::msg::ArmMsg>::SharedPtr pub_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mtx_;
    std::vector<Point> points_;
    std::shared_ptr<GoalHandle> handle_;
    double elapsed_ = 0.0;
    bool finished_ = true;

    double rate_hz_ = 50.0;
    double speed_scale_ = 1.0;
};

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryBridge>());
    rclcpp::shutdown();
    return 0;
}
