#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t N = 12;
double dt = 0.1;
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
    fg[0] = 0;    

    double ref_v = 100. / 0.62137 * 1000./ 3600. ;    
    //adjusting contribution of different cost components to the total
    double k_cte = 2;
    double k_epsi = 2000;
    double k_v = 0.00015;
    double k_d1 = 0;
    double k_a1 = 0;    
    double k_d2 = 20; 
    double k_a2 = 0;
    
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
    fg[0] += cte_error +  psi_error + vel_error;
    for (size_t t = 0; t < N - 1; t++) {
      delta_cum += k_d1 * CppAD::pow(vars[delta_start + t], 2);
      a_cum += k_a1 * CppAD::pow(vars[a_start + t], 2);
    }
    fg[0] += delta_cum + a_cum;
    for (size_t t = 0; t < N - 2; t++) {
      delta_der += k_d2 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      a_der += k_a2 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
    fg[0] +=  delta_der + a_der;

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
      AD<double> throttle_to_acceleration = -0.1132 * v0 + 5.3603;      
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0  + 3 * coeffs[3] * CppAD::pow(x0,2));     
      
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0  / Lf * delta0 * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * throttle_to_acceleration * dt);      
      fg[1 + cte_start + t] = cte1 - ((y0 - v0 * CppAD::sin(epsi0) * dt) - f0);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - - v0/Lf * delta0 * dt) - psides0);                   
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
  
  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];
  const double delta_prev = state[6];
  const double a_prev = state[7];

  
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
  double latency = 0.1; //sec
  vars[x_start] = x + v * CppAD::cos(psi) * latency;
  vars[y_start] = y + v * CppAD::sin(psi) * latency;  
  vars[psi_start] = psi - v  / Lf * delta_prev * latency;
  double throttle_to_acceleration = -0.1132 * v + 5.3603;   
  vars[v_start] = v + a_prev * throttle_to_acceleration * latency;  
  vars[cte_start] = (y - v * CppAD::sin(psi) * latency) - 
                    coeffs[0] + coeffs[1] * vars[x_start] + coeffs[2] * CppAD::pow(vars[x_start],2) + coeffs[3] * CppAD::pow(vars[x_start],3);
  vars[epsi_start] = psi - (CppAD::atan(coeffs[1] + 2 * coeffs[2] * vars[x_start]  + 3 * coeffs[3] * CppAD::pow(vars[x_start],2)));
 

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
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
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }  
  // Acceleration/decceleration upper and lower limits.
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
  FG_eval fg_eval(coeffs); 
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
  //auto cost = solution.obj_value;

  vector<double> mpc_output;
  mpc_output.push_back(solution.x[delta_start]);
  mpc_output.push_back(solution.x[a_start]);

  for (size_t i = 0; i < N; i++) {
    mpc_output.push_back(solution.x[x_start + i]);
    mpc_output.push_back(solution.x[y_start + i]);
  }
  return mpc_output;
}
