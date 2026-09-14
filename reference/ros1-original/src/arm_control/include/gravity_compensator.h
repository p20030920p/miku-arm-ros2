// gravity_compensator.h
#pragma once
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <vector>

class GravityCompensator {
public:
    bool init(const KDL::Chain& chain);            // 建 dyn_solver
    void setGains(const std::vector<double>& g);   // 设补偿系数
    void update(const KDL::JntArray& q);           // 算 tau_g
    // 返回 tau_g * gains（就是 msg_out.tor_x 那 6 个数）
    std::vector<double> compensatedTorques() const;
    ~GravityCompensator();   // 释放 dyn_solver
private:
    KDL::ChainDynParam* dyn_solver = nullptr;
    KDL::JntArray tau_g;
    std::vector<double> gains = {0.5, 0.55, 0.61, 0.6, 0.7, 0.3};
};