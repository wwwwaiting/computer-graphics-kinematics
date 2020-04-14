#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double sigma = max_step;
  Eigen::VectorXd z_moved = z - sigma*dz;
  proj_z(z_moved);

  while (f(z_moved) > f(z)){
    sigma *= 0.5;
    z_moved = z - sigma*dz;
    proj_z(z_moved);
    if (z_moved == z){
      return 0;
    }
  }
  return sigma;
  /////////////////////////////////////////////////////////////////////////////
}
