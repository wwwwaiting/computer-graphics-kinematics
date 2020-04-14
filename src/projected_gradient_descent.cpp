#include "projected_gradient_descent.h"
#include "line_search.h"

void projected_gradient_descent(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<Eigen::VectorXd(const Eigen::VectorXd &)> & grad_f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const int max_iters,
  Eigen::VectorXd & z)
{
  /////////////////////////////////////////////////////////////////////////////
  int i = 0;
   for (int i=0; i<max_iters; i++){
    Eigen::VectorXd dz = grad_f(z);

    // stop if dz is 0
    if (dz.norm() == 0){
      return;
    }
    
    double sigma = line_search(f, proj_z, z, dz, 10000);
    z = z - sigma*dz;
    proj_z(z);
  }
  /////////////////////////////////////////////////////////////////////////////
}
