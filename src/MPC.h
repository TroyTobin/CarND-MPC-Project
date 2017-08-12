#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// Lf measures the distance between the front of the vehicle and its center of gravity. 
// The larger the vehicle, the slower the turn rate.

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
#define Lf    (2.67)

class MPC {
 public:
  MPC();

  virtual ~MPC();  

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, 
                       Eigen::VectorXd coeffs);

};

#endif /* MPC_H */
