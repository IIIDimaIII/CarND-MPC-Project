#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuators.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, int& x_direction, double& dt);
};

#endif /* MPC_H */
