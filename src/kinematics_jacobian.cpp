#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  Eigen::VectorXd pos_tips = transformed_tips(skeleton, b);
  double h = 10e-7;

  for (int i = 0; i < skeleton.size(); i++){
    for (int j = 0; j < 3; j++){
      Skeleton skeleton_h = skeleton;
      skeleton_h[i].xzx[j] += h;
      J.col(3*i + j) = (transformed_tips(skeleton_h,b) - pos_tips) / h;
    }
  }
  /////////////////////////////////////////////////////////////////////////////
}
