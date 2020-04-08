#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

/**
 *  设置时间间隔和预测的步数
 */
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
//设置半周长
const double Lf = 2.67;

//设置目标车速
double ref_v = 40.0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
// 定义每一个state的起始点,因为函数的需要，只能传入一个数组，所以将误差以及各种状态的点都放在一个数组中展开
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
  VectorXd coeffs;
  FG_eval(VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
// The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost

    // 损失函数,目的是将损失函数降到最小，求最优化解
    //基于参考状态的损失，求状态的最优化，最小的距离，最小的夹角，最小的速度
    for (unsigned int i = 0; i < N; i++)
    {
      fg[0] += 500 * CppAD::pow(vars[cte_start + i], 2);
      fg[0] += 500 * CppAD::pow(vars[epsi_start + i],2);
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v,2);
    }

    //执行器的损失，求执行器的最优化值
    for (unsigned int i = 0; i < N - 1; i++)
    {
      fg[0] += 50 * CppAD::pow(vars[delta_start + i], 2);
      fg[0] += 50 * CppAD::pow(vars[a_start + i],2);
      // try adding penalty for speed + steer
      // fg[0] += 700*CppAD::pow(vars[delta_start + i] * vars[v_start+i], 2);
    
    }

    // 求两次动作的最优化值
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 200*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += 10*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    //
    // Setup Constraints
    // 设置约束，在这里设置约束条件
    //

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    //因为fg【0】是损失，其余的状态从fg【1】开始
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; ++t) {

      //t时刻的状态值
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      //t-1时刻的状态
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      //只计算t-1时刻
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      if (t > 1) {   // use previous actuations (to account for latency)
        a0 = vars[a_start + t - 2];
        delta0 = vars[delta_start + t - 2];
      }

      AD<double> f0 = coeffs[0] + coeffs[1] * x0  + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // The idea here is to constraint this value to be 0.
      //
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // CppAD can compute derivatives and pass these to the solver.

      //约束条件，目的是让该值等于0 ，所以前一时刻的状态经过变换应该等于下一时刻的状态
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);


    }


  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &state, const VectorXd &coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  /**
   * Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
   // 独立变量的个数
  // N timesteps == N - 1 actuations
  size_t n_vars = state.size() * 10 + (N - 1) * 2;
  /**
   * 设置变量个数
   */
  size_t n_constraints = N * 6;

  // 初始化所有独立变量的值
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }

  // Set the initial variable values
  //设置初始状态
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;


  /**
   * TODO: Set lower and upper limits for variables.
   */
  // Lower and upper limits for x
  //所有x的极限值
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  //将所有非执行器的状态量的最大最小值分别都拉到最大
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  //将所有的delta的限制设置为±25°

  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // 加速/减速上限和下限。
  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // 约束的下限和上限
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
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

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
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

  /**
   * TODO: Return the first actuator values. The variables can be accessed with
   *   `solution.x[i]`.
   *
   * {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
   *   creates a 2 element double vector.
   */

  std::vector<double> res;
  res.push_back(solution.x[delta_start]);
  res.push_back(solution.x[a_start]);
  for (size_t i = 0; i < N - 2; i++)
  {
    res.push_back(solution.x[x_start + i + 1 ]);
    res.push_back(solution.x[y_start + i + 1 ]);
  }
  
  return res;
}