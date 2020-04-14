#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::VectorXd pos_tips = Eigen::VectorXd::Zero(3*b.size());

  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > T;
  forward_kinematics(skeleton, T);

  for (int i = 0; i < b.size(); i++){
    int bone_idx = b[i];
    Eigen::VectorXd tip = T[bone_idx] * skeleton[bone_idx].rest_T * Eigen::Vector4d(skeleton[bone_idx].length, 0, 0, 1);
    pos_tips[3*i] =  tip[0];
    pos_tips[3*i + 1] = tip[1];
    pos_tips[3*i + 2] = tip[2];
  }
  return pos_tips;
  /////////////////////////////////////////////////////////////////////////////
}
