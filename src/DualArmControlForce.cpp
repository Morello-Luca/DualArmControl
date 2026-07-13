#include "DualArmControl.h"

void DualArmControl::filtering(){
       // =====================================================================================
       // 4. ALPHA-BETA FILTER (Kalman-like tracking for Position and Velocity)
       // -------------------------------------------------------------------------------------
       // This steady-state filter smooths the noisy force measurement (lambdaMeasured_)
       // and estimates its first derivative (lambdaDotFiltered_) without a severe lag.
       // Tuning Guide:
       // - alpha_k (0 to 1): Controls position tracking. Higher = faster response but noisy.
       //                                                 Lower  = smoother but introduces lag.
       // - beta_k  (0 to 1): Controls velocity tracking. Higher = reactive but shaky derivative.
       // =====================================================================================
       smoothFilter_.alpha =   0.10;        smoothFilter_.beta  =   0.005;
       balancedFilter_.alpha = 0.25;        balancedFilter_.beta  = 0.02;
       fastFilter_.alpha =     0.45;        fastFilter_.beta  =     0.10;   

       smoothFilter_.update(lambdaMeasured_, timeStep);
       balancedFilter_.update(lambdaMeasured_, timeStep);
       fastFilter_.update(lambdaMeasured_, timeStep);
       // =====================================================================================
       // 4.1 FIRST-ORDER LOW-PASS FILTER (for comparison only)
       // -------------------------------------------------------------------------------------
       // Classical first-order low-pass filter:
       // y(k) = y(k-1) + alpha * (x(k) - y(k-1))
       // Smaller alpha -> stronger smoothing           Larger alpha  -> faster response
       // =====================================================================================
       lowPass(lambdaFiltered_lowpass_moltosmooth, lambdaMeasured_, 0.05);
       lowPass(lambdaFiltered_lowpass_smooth,      lambdaMeasured_, 0.20);
       lowPass(lambdaFiltered_lowpass_mid,         lambdaMeasured_, 0.30);
       lowPass(lambdaFiltered_lowpass_high,        lambdaMeasured_, 0.70);
       double tau = 1.0 / (2.0 * M_PI * gains.cutoffPeriod);
       double alpha_lpcut = timeStep / (tau + timeStep);
       lowPass(lambdaFiltered_lowpass_cut, lambdaMeasured_, alpha_lpcut);
}

void DualArmControl::updateContactForces(){
       // Rotation Matrix
              auto RL = robots().robot(leftRobotIndex_).bodyPosW(eeName_).rotation();
              auto RR = robots().robot(rightRobotIndex_).bodyPosW(eeName_).rotation();
       // World Frame FORCES
              Eigen::Matrix<double, 6, 1> fL_world;
              Eigen::Matrix<double, 6, 1> fR_world;
              Eigen::Matrix<double, 12, 1> f_meas;
       // Contact Force Measure
              sva::ForceVecd WL = leftImpedanceTask_->measuredWrench();
              sva::ForceVecd WR = rightImpedanceTask_->measuredWrench();
              
              fL_world.head<3>() = RL * WL.couple();
              fL_world.tail<3>() = RL * WL.force();
              fR_world.head<3>() = RR * WR.couple();
              fR_world.tail<3>() = RR * WR.force();
       // Group into one vector
              f_meas.segment<6>(0) = fL_world;
              f_meas.segment<6>(6) = fR_world;
       // Null-Space Projection Matrix
              Eigen::Matrix<double, 12, 12> Pint = Eigen::Matrix<double, 12, 12>::Identity() - Gpinv_ * G_;
       //  Squeeze direction
              Eigen::Matrix<double, 12, 1> n_s;
              n_s << 0, 0, 0, 0, 1, 0,
                     0, 0, 0, 0, -1, 0;


              sva::ForceVecd target_{Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -gains.lambda_desired)};
              sva::ForceVecd min_{Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -10)};
              Eigen::Matrix<double, 12, 1> internalForce = Pint * f_meas;
       // Null Space projection of squeeze
              Eigen::JacobiSVD<Eigen::MatrixXd> svd(G_, Eigen::ComputeFullV);
              int rank = svd.rank();
              Eigen::MatrixXd N = svd.matrixV().rightCols(G_.cols() - rank);
              Eigen::Matrix<double, 12, 1> n_squeeze = N * (N.transpose() * n_s);
              n_squeeze.stableNormalize();
              lambdaMeasured_ = n_squeeze.transpose() * internalForce;



       // =====================================================================================
       // 5. INTERNAL SQUEEZE FORCE CONTROL
       filtering();
       error_internalForce = gains.lambda_desired - lambdaFiltered_lowpass_cut;
       double Kor   = 40.0;   // N
       double phi = 15.0;   // N
       double Kp = 2;
       double correction = Kor * std::tanh(Kp*error_internalForce / phi);
       double lambda_cmd = gains.lambda_desired + correction;
       double lambda_min = 10.0; // [N] Hard floor: never command less than a 5N push
              if (lambda_cmd < lambda_min) {
              lambda_cmd = lambda_min; 
              }
       target_.force().z() = -lambda_cmd;
       lambdaCommand_ = std::clamp(lambda_cmd, -80.0, 80.0);

       // =====================================================================================
       // 6. PROIETTA NEL WRENCH
       //    ----------------------------------------------------------------------------------
       Eigen::Matrix<double, 12, 1> f_input = n_squeeze * lambdaCommand_;

  
       Eigen::Matrix<double, 6, 1> wL_world = f_input.segment<6>(0);
       Eigen::Matrix<double, 6, 1> wR_world = f_input.segment<6>(6);

       Eigen::Matrix<double, 6, 1> wL_local;
       Eigen::Matrix<double, 6, 1> wR_local;


       

       wL_local.head<3>() = RL.transpose() * wL_world.head<3>();
       wL_local.tail<3>() = RL.transpose() * wL_world.tail<3>();

       wR_local.head<3>() = RR.transpose() * wR_world.head<3>();
       wR_local.tail<3>() = RR.transpose() * wR_world.tail<3>();

      

       //sva::ForceVecd cmdLeft(target_.vector() + wL_local);
       //sva::ForceVecd cmdRight(target_.vector() + wR_local);

       cmdLeft.couple() = wL_local.head<3>();
       cmdLeft.force()  = wL_local.tail<3>();

       cmdRight.couple() = wR_local.head<3>();
       cmdRight.force()  = wR_local.tail<3>();


        //mc_rtc::log::info("Force left {} , force right {} ", cmdLeft.force(),cmdRight.force());
       // =====================================================================================
       // 7. SEND WRENCHES
       //    ----------------------------------------------------------------------------------=
       leftImpedanceTask_->targetWrench(cmdLeft);
       rightImpedanceTask_->targetWrench(cmdRight);
       //leftImpedanceTask_->targetWrench(target_);
       //rightImpedanceTask_->targetWrench(target_);
       
}