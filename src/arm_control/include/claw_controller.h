// claw_controller.h
// 夹爪（电机7）控制封装：命令式接口 + 4 态状态机
// 参数可配置：setParams(ClawParams) 从 yaml 读取，不设则用默认值
#pragma once

/* ---------- 夹爪参数（单位：角度用"度"，yaml 里也是度） ---------- */
struct ClawParams {
    // 张开
    double open_pos_deg     = -5.0;    // 张开到位目标角 (°)
    double open_step_deg    =  1.5;    // 张开步进 (°/帧)
    double open_torque      =  0.8;    // 张开驱动力矩 (N·m)
    // 夹紧
    double close_pos_deg    = -165.0;  // 闭合到位目标角 (°)
    double close_step_deg   =  0.5;    // 夹紧步进 (°/帧)
    double close_torque     = -0.8;    // 夹紧驱动力矩 (N·m)
    // 保持 / 检测
    double hold_step_deg    =  14.5;   // 预压量 (°)
    double hold_torque      = -1.2;    // 保持力矩 (N·m)
    double catch_torque     =  0.5;    // 夹到判定阈值 (N·m)
};

class ClawController {
public:
    enum ClawState {
        CLAW_OPEN,      // 张开（待命）
        CLAW_CLOSING,   // 夹紧中（正在夹取）
        CLAW_HOLD,      // 已夹到（保持夹持）
        CLAW_FAILED,    // 未夹到（已到下限）
    };

    ClawController() = default;

    void setParams(const ClawParams& p);   // 从外部（yaml）设置参数

    void open();     // 张开（松开物体）
    void close();    // 夹紧（开始夹取，自动检测夹到/失败）

    void update(double pos_now, double tor_now,
                double& cmd_pos, double& cmd_vel, double& cmd_tor);

    ClawState state() const { return claw_state_; }
    bool isHolding() const;
    bool isFailed() const;
    const char* stateName() const;

private:
    ClawParams params_;                  // 夹爪参数（默认值，可被 setParams 覆盖）
    ClawState  claw_state_    = CLAW_OPEN;
    int        catch_counter_ = 0;
    double     catch_pos_     = 0.0;
};