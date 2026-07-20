#pragma once

#include <Eigen/Core>
#include <eigen-qld/QLD.h>

class QLDSolver
{
public:

    bool solve(
        const Eigen::MatrixXd & H,
        const Eigen::VectorXd & g,
        const Eigen::MatrixXd & Aeq,
        const Eigen::VectorXd & beq,
        const Eigen::MatrixXd & Aineq,
        const Eigen::VectorXd & bineq,
        const Eigen::VectorXd & xl,
        const Eigen::VectorXd & xu);

    const Eigen::VectorXd & result() const;

private:

    Eigen::QLD solver_;
};