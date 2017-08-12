#include "MPC.h"
#include "util.hpp"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <limits.h>

using CppAD::AD;

// TODO: Set the timestep length and duration

// Initial values from "Solution: Mind The Line" lecture
#define NUM_SAMPLES              (15)
#define NUM_RETURN_SAMPLES       (10)
#define TIME_DELTA             (0.05)

// velocity to use
#define VELOCITY                 (65)

// Vars is all one long vector, 
// so if N is 25 then the indices are assigned as follows:
// vars[0], ..., vars[24] -> x1,...,x25
// vars[25], ..., vars[49] -> y1,...,y25
// vars[50], ..., vars[74] -> ψ1,...,ψ25
// vars[75], ..., vars[99] -> v1,...,v25
// vars[100], ..., vars[124] -> cte1,...,cte25
// vars[125], ..., vars[149] -> eψ1,...,eψ25
// vars[150], ..., vars[173] -> δ1,...,δ24
// vars[174], ..., vars[197] -> a1,...,a24
#define X_INDEX                           (0)
#define Y_INDEX                 (NUM_SAMPLES)
#define PSI_INDEX             (NUM_SAMPLES*2)
#define VEL_INDEX             (NUM_SAMPLES*3)
#define CTE_INDEX             (NUM_SAMPLES*4)
#define ESTEERANGLE_INDEX     (NUM_SAMPLES*5)
#define STEERANGLE_INDEX      (NUM_SAMPLES*6)
#define ACCEL_INDEX       (NUM_SAMPLES*7 - 1)

// Index in vehicle state vector
#define COST_INDEX                        (0)
#define STATE_INDEX                       (1)

class FG_eval 
{
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) 
  { 
    this->coeffs = coeffs; 
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector& fg, const ADvector& vars) 
  {
    int i = 0;
    /********************************************************
     *
     *                     COST FUNCTION
     *
     *********************************************************/
    // Cost tally
    AD<double>  cost = 0;

    // Cost function
    for (i = 0; i < NUM_SAMPLES; i++) 
    {
      // Penalise the actual errors
      //    - Cross track error
      //    - Orientation error
      {
        AD<double> cte    = vars[CTE_INDEX + i];
        AD<double> esteer = vars[ESTEERANGLE_INDEX + i];
        cost += CppAD::pow(cte, 2) + CppAD::pow(esteer, 2);
      }

      // Penalise not maintaining the desired velocity
      {
        AD<double>  velocity = vars[VEL_INDEX + i];
        cost += CppAD::pow(velocity - VELOCITY, 2);
      }

      // Penalise large actuations - prefer smaller adjustments
      if ((i + 1) < NUM_SAMPLES)
      {
        AD<double>  steering = vars[STEERANGLE_INDEX + i];
        AD<double>  accel    = vars[ACCEL_INDEX + i];
        cost += CppAD::pow(steering, 2) + CppAD::pow(accel, 2);
      }

      // Penalise large changes in actuations - creates a "smoother" experience
      if ((i + 2) < NUM_SAMPLES)
      {
        AD<double>  steering      = vars[STEERANGLE_INDEX + i];
        AD<double>  steering_next = vars[STEERANGLE_INDEX + i + 1];
        AD<double>  accel         = vars[ACCEL_INDEX + i];
        AD<double>  accel_next    = vars[ACCEL_INDEX + i + 1];
        cost += 5000*CppAD::pow(steering_next - steering, 2) + CppAD::pow(accel_next - accel, 2);
      }
    }

    // finally set the calculated cost
    fg[COST_INDEX] = cost;

    /********************************************************
     *
     *                   VEHICLE MODEL
     *
     *********************************************************/
    // Set the "current" model state
    fg[STATE_INDEX + X_INDEX]           = vars[X_INDEX];
    fg[STATE_INDEX + Y_INDEX]           = vars[Y_INDEX];
    fg[STATE_INDEX + PSI_INDEX]         = vars[PSI_INDEX];
    fg[STATE_INDEX + VEL_INDEX]         = vars[VEL_INDEX];
    fg[STATE_INDEX + CTE_INDEX]         = vars[CTE_INDEX];
    fg[STATE_INDEX + ESTEERANGLE_INDEX] = vars[ESTEERANGLE_INDEX];

    // Calculate the vehicle model through time
    for (i = 0; (i + 1) < NUM_SAMPLES; ) 
    {
      // Current time-step's state
      AD<double>  x      = vars[X_INDEX + i];
      AD<double>  y      = vars[Y_INDEX + i];
      AD<double>  psi    = vars[PSI_INDEX + i];
      AD<double>  vel    = vars[VEL_INDEX + i];
      AD<double>  cte    = vars[CTE_INDEX + i];
      AD<double>  steer  = vars[STEERANGLE_INDEX + i];
      AD<double>  accel  = vars[ACCEL_INDEX + i];
      AD<double>  esteer = vars[ESTEERANGLE_INDEX + i];

      // Go to next time-step's state
      i++;

      AD<double>  x_next      = vars[X_INDEX + i];
      AD<double>  y_next      = vars[Y_INDEX + i];
      AD<double>  psi_next    = vars[PSI_INDEX + i];
      AD<double>  vel_next    = vars[VEL_INDEX + i];
      AD<double>  cte_next    = vars[CTE_INDEX + i];
      AD<double>  esteer_next = vars[ESTEERANGLE_INDEX + i];

      // Evaluate the polynomial fit at the current state
      AD<double> f = (polyeval(this->coeffs, x));

      // Calculate the tangent to the polynomial to get the desired steering angle.
      // This is just the derivative of the polynomial
      AD<double>  steer_desired = CppAD::atan(polyeval_derivative(this->coeffs, x));

      // Equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      AD<double> x_next_      = x + vel * CppAD::cos(psi) * TIME_DELTA;
      AD<double> y_next_      = y + vel * CppAD::sin(psi) * TIME_DELTA;
      AD<double> psi_next_    = psi + vel * steer / Lf * TIME_DELTA;
      AD<double> vel_next_    = vel + accel * TIME_DELTA;
      AD<double> cte_next_    = f - y + vel * CppAD::sin(esteer) * TIME_DELTA;
      AD<double> esteer_next_ = psi - steer_desired + vel * steer / Lf * TIME_DELTA;


      fg[STATE_INDEX + X_INDEX + i]           = x_next - x_next_;
      fg[STATE_INDEX + Y_INDEX + i]           = y_next - y_next_;
      fg[STATE_INDEX + PSI_INDEX + i]         = psi_next - psi_next_;
      fg[STATE_INDEX + VEL_INDEX + i]         = vel_next - vel_next_;
      fg[STATE_INDEX + CTE_INDEX + i]         = cte_next - cte_next_;
      fg[STATE_INDEX + ESTEERANGLE_INDEX + i] = esteer_next - esteer_next_;
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, 
                          Eigen::VectorXd coeffs) 
{
  bool ok = true;
  int i = 0;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = (state.size() * NUM_SAMPLES) + (2 * (NUM_SAMPLES - 1));
  // TODO: Set the number of constraints
  size_t n_constraints = (state.size() * NUM_SAMPLES);

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) 
  {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (i = 0; i < STEERANGLE_INDEX; i++) 
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (i = STEERANGLE_INDEX; i < ACCEL_INDEX; i++) 
  {
    vars_lowerbound[i] = -DEG2RAD(25);
    vars_upperbound[i] = DEG2RAD(25);
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (i = ACCEL_INDEX; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) 
  {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[X_INDEX] = x;
  constraints_lowerbound[Y_INDEX] = y;
  constraints_lowerbound[PSI_INDEX] = psi;
  constraints_lowerbound[VEL_INDEX] = v;
  constraints_lowerbound[CTE_INDEX] = cte;
  constraints_lowerbound[ESTEERANGLE_INDEX] = epsi;

  constraints_upperbound[X_INDEX] = x;
  constraints_upperbound[Y_INDEX] = y;
  constraints_upperbound[PSI_INDEX] = psi;
  constraints_upperbound[VEL_INDEX] = v;
  constraints_upperbound[CTE_INDEX] = cte;
  constraints_upperbound[ESTEERANGLE_INDEX] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> output;

  output.push_back(solution.x[STEERANGLE_INDEX]);
  output.push_back(solution.x[ACCEL_INDEX]);

  for (i = 0; i < NUM_RETURN_SAMPLES - 2; i++)
  {
    output.push_back(solution.x[X_INDEX + i + 1]);
    output.push_back(solution.x[Y_INDEX + i + 1]);
  }

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return output;
}
