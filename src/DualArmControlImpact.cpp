#include "DualArmControl.h"

std::pair<
    std::unique_ptr<mc_solver::BoundedSpeedConstr>,
    std::unique_ptr<mc_solver::BoundedSpeedConstr>
    > DualArmControl::boundSpeed(const Eigen::VectorXd & spd){
    
    Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);
    
    auto speedLeftConstr = std::make_unique<mc_solver::BoundedSpeedConstr>();
    auto speedRightConstr = std::make_unique<mc_solver::BoundedSpeedConstr>();
    
    speedLeftConstr->addBoundedSpeed(solver(),eeName_,Eigen::Vector3d::Zero(),dof,-spd,spd);
    speedRightConstr->addBoundedSpeed(solver(),eeName_,Eigen::Vector3d::Zero(),dof,-spd,spd);
    
    return {std::move(speedLeftConstr), std::move(speedRightConstr)};
}