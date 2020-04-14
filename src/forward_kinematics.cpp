#include "forward_kinematics.h"
#include "euler_angles_to_transform.h"
#include <functional> // std::function

void forward_kinematics(
  const Skeleton & skeleton,
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  T.resize(skeleton.size(),Eigen::Affine3d::Identity());
  // using the formula: T_i = T_pi * T_i_hat * R_i_hat * T_i_hat_inverse
  for (int i = 0; i < skeleton.size(); i++){
    Eigen::Affine3d parent = Eigen::Affine3d::Identity();
    
    if (skeleton[i].parent_index != -1) {
      parent = T[skeleton[i].parent_index];
    }

    T[i] = parent * skeleton[i].rest_T * euler_angles_to_transform(skeleton[i].xzx) * (skeleton[i].rest_T).inverse();
  }
  /////////////////////////////////////////////////////////////////////////////
}
