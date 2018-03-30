#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  /**
  TODO: done
    * Calculate the RMSE here.
  */

  VectorXd rmse(4);
  rmse.setZero();

  if (!estimations.size()) {
    std::cout << "Error: the size of Estimations Vector is Zero." << endl;
    return rmse;
  }
  else if (estimations.size() != ground_truth.size()) {
    std::cout << "There is a Dimenstional mismatch" << endl;
    return rmse;
  }

  for (int i = 0; i < estimations.size(); i++) {
    VectorXd res = estimations[i] - ground_truth[i];
    res = res.array().pow(2);
    rmse += res;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();

  return rmse;
}