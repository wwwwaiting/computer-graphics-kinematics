#include "end_effectors_objective_and_gradient.h"
#include "transformed_tips.h"
#include "kinematics_jacobian.h"
#include "copy_skeleton_at.h"
#include <iostream>

void end_effectors_objective_and_gradient(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  const Eigen::VectorXd & xb0,
  std::function<double(const Eigen::VectorXd &)> & f,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  std::function<void(Eigen::VectorXd &)> & proj_z)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  f = [&](const Eigen::VectorXd & A)->double
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    // calculate the transformed positions
    Eigen::VectorXd tips = transformed_tips(copy, b);
    // get least-squars objective value
    return (tips - xb0).dot(tips - xb0);
  };
  grad_f = [&](const Eigen::VectorXd & A)->Eigen::VectorXd
  {
    Skeleton copy = copy_skeleton_at(skeleton, A);
    // calculate the transformed positions
    Eigen::VectorXd tips = transformed_tips(copy, b);
    // get the jacobian matrix
    Eigen::MatrixXd J;
    kinematics_jacobian(copy, b, J);

    // calculate gradient descent
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(A.size());
    // using chain rule J.T * d(E(x)), dE(x) is 2*(x_bi - x_bi_hat)
    gradient = J.transpose() * 2 * (tips - xb0);
    return gradient;
  };
  proj_z = [&](Eigen::VectorXd & A)
  {
    for (int i=0; i<skeleton.size(); i++){
			A[3*i] = std::fmax(skeleton[i].xzx_min[0], std::fmin(skeleton[i].xzx_max[0], A[3*i]));
			A[3*i+1] = std::fmax(skeleton[i].xzx_min[1], std::fmin(skeleton[i].xzx_max[1], A[3*i+1]));
			A[3*i+2] = std::fmax(skeleton[i].xzx_min[2], std::fmin(skeleton[i].xzx_max[2], A[3*i+2]));  	
  	}
    assert(skeleton.size()*3 == A.size());
  };
  /////////////////////////////////////////////////////////////////////////////
}
