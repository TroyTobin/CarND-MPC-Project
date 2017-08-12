/**
 *
 *  Helper utility functions
 *
 */

#ifndef __UTIL_H__
#define __UTIL_H__

#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>

 // For converting back and forth between radians and degrees.
#define DEG2RAD(deg) (deg * M_PI / 180)
#define RAD2DEG(rad) (rad * 180 / M_PI)

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, 
                double x);

// Evaluate a polynomial derivative.
double polyeval_derivative(Eigen::VectorXd coeffs, 
                           double x);

// Evaluate a polynomial.
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, 
                           CppAD::AD<double> x);

// Evaluate a polynomial derivative.
CppAD::AD<double> polyeval_derivative(Eigen::VectorXd coeffs, 
                                      CppAD::AD<double> x);
// Fit a polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, 
                        Eigen::VectorXd yvals,
                        int order);

#endif