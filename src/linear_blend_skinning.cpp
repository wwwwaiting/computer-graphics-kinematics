#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  U.resize(V.rows(), 3);

  for(int i = 0; i < V.rows(); i++){
    Eigen::RowVector3d v = V.row(i);
  	Eigen::Vector3d u = Eigen::Vector3d::Zero();
    
    // using formula v_i = sum(Wi,j * Tj * v.T)
  	for(int j = 0; j < skeleton.size(); j++){
  		if (skeleton[j].weight_index != -1){
  		  u += W(i, skeleton[j].weight_index) * (T[j] * v.transpose()).transpose();
  		}
  	}
    U.row(i) = u;
  }
  /////////////////////////////////////////////////////////////////////////////
}
