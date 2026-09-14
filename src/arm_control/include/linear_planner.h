// linear_planner.h
#pragma once
#include <kdl/frames.hpp>

struct Pose6 { double R=0, P=0, Y=0, x=0, y=0, z=0; };

class LinearPlanner {
public:
    void setTarget(const Pose6& start, const Pose6& goal, double speed);
    // 每步推进，返回当前应到达的位姿；finished=true 表示到终点
    bool update(double dt, Pose6& out);
    bool isExecuting() const;
private:
    Pose6 start_, goal_, current_;
    double target_speed_ = 0.005;
    bool executing_ = false;
};