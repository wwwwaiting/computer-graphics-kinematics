#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  Eigen::Affine3d A, B, C;
  double theta1 =  xzx[0] / 180.0 * M_PI;
  double theta2 =  xzx[1] / 180.0 * M_PI;
  double theta3 =  xzx[2] / 180.0 * M_PI;
  A.matrix() << 
    1,0,0,0,
    0,cos(theta1),-sin(theta1),0,
    0,sin(theta1),cos(theta1),0,
    0,0,0,1;
  B.matrix() << 
    cos(theta2),-sin(theta2),0,0,
    sin(theta2),cos(theta2),0,0,
    0,0,1,0,
    0,0,0,1;
  C.matrix() << 
    1,0,0,0,
    0,cos(theta3),-sin(theta3),0,
    0,sin(theta3),cos(theta3),0,
    0,0,0,1;
  return C * B * A;
  /////////////////////////////////////////////////////////////////////////////
}
