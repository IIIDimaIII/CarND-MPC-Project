#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
//telemetry comes at the average frequency of 155 milliseconds + lag
size_t N = 20;
double dt = 0.1;

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
const double Lf = 2.67;

//indexes for the vector used in the MPC solver
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {    
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;    
    // The part of the cost based on the reference state.
    double ref_v = 70.;
    
    //adjusting contribution of different cost components to the total
    double k_cte = 3;
    double k_epsi = 4;
    double k_v = 1e-5;

    double k_d1 = 5e-3;
    double k_a1 = 5e-3;
    
    double k_d2 = 2e-1;
    double k_a2 = 1e-2;

    AD<double> cte_error = 0;
    AD<double> psi_error = 0;
    AD<double> vel_error = 0;
    AD<double> delta_cum = 0;
    AD<double> a_cum = 0;
    AD<double> delta_der = 0;
    AD<double> a_der = 0;
    
    for (size_t t = 0; t < N; t++) {
      cte_error += k_cte * CppAD::pow(vars[cte_start + t], 2);
      psi_error += k_epsi * CppAD::pow(vars[epsi_start + t], 2);
      vel_error += k_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }    
    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      delta_cum += k_d1 * CppAD::pow(vars[delta_start + t], 2);
      a_cum += k_a1 * CppAD::pow(vars[a_start + t], 2);
    }
    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      delta_der += k_d2 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      a_der += k_a2 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    fg[0] = cte_error +  psi_error + vel_error + delta_cum + a_cum + delta_der + a_der;
    cout << "cte_error " << cte_error << endl; 
    cout << "psi_error " << psi_error << endl;
    cout << "vel_error " << vel_error << endl;
    cout << "delta_cum " << delta_cum << endl;
    cout << "a_cum " << a_cum << endl;
    cout << "delta_der " << delta_der << endl;    
    cout << "a_der " << a_der << endl;    
    

    // Setup Constraints
    // current state: no need to calculate just grap the inputs
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];        
    // The rest of the constraints - future steps       
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .      
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];      

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];     

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];         
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0  + 3 * coeffs[3] * CppAD::pow(x0,2));     
      
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0  / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);      
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt);                   
    }    
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;  
  typedef CPPAD_TESTVECTOR(double) Dvector;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  
  // (number of independent variables) * (number of timesteps predicted)
  // +
  // (number of actuators) * (number of timesteps predicted - 1)
  size_t n_vars = N * 6 + (N - 1) * 2;

  // the number of constraints: timesteps * number of independent variables
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);  
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;   
  }  
  // Set the initial variable values  
  vars[x_start] = x;  
  vars[y_start] = y;  
  vars[psi_start] = psi;  
  vars[v_start] = v;  
  vars[cte_start] = cte;  
  vars[epsi_start] = epsi;   

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (size_t i = 0; i < y_start; i++) {
    vars_lowerbound[i] = 0;
    vars_upperbound[i] = 1.0e19;
  } 
  
  
  for (size_t i = y_start; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }  
  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; i++) {

    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }  
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  } 
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;  

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;  

  // object that computes objective and constraints
  //FG_eval fg_eval(coeffs); 
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

  //cout << "solution" << endl;
  //for (int i = 0; i < n_vars; i++) {    
  //  cout << solution.x[i] << endl;  
  //}
 

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> mpc_output;
  mpc_output.push_back(solution.x[delta_start]);
  mpc_output.push_back(solution.x[a_start]);

  for (size_t i = 0; i < N; i++) {
    mpc_output.push_back(solution.x[x_start + i]);
    mpc_output.push_back(solution.x[y_start + i]);
  }

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return mpc_output;
}
