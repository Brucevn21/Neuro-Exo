#pragma once
#include <string>
#include <eigen3/Eigen/Dense>
struct TestGlobals {
  bool END_STAGE = false, EMERGENCY_STOP = false, IMAGINE_MOVEMENT = false;
  bool FIXATE = false, MOVE = false, END_OF_TRIAL = false, MOVEMENT_PREDICTED = false;
  std::string THERAPY_STAGE, TASK, POSITION = "0";
};
inline TestGlobals gV;
// TEST SUBSTITUTE: identity H-infinity stage, not the missing real algorithm.
struct FilterResult { Eigen::MatrixXd Pt, wh; Eigen::VectorXd shsh; };
inline FilterResult Hinf_RT_Filter_v2016_2_11(
    const Eigen::VectorXd& y, const Eigen::MatrixXd&, double,
    const Eigen::MatrixXd& pt, const Eigen::MatrixXd& wh, double) {
  return {pt, wh, y};
}
