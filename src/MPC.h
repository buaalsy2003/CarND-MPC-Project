#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Result from the Solver
struct Results {
    vector<double>  X;      // X coordinate
    vector<double>  Y;      // Y coordinate
    vector<double>  Psi;    // Orientation angle
    vector<double>  V;      // Velocity
    vector<double>  CTE;    // Cross Track Error
    vector<double>  EPsi;   // Orientation angle Error
    vector<double>  Delta;  // Steering angle
    vector<double>  A;      // Acceleration (throttle)
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

	int latency_dt {2};
  double delta_previous {0};
	double acc_previous {0.1};

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Results Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
