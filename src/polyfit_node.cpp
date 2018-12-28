#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <sstream>

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }
  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "polyfit_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  //test
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y = {4.00, 6.40, 8.00, 8.80, 9.22, 9.50};
  for (int i = 1; i <= 6; i++) {
    waypoints_x.push_back(i);
  }
  double* ptrx = &waypoints_x[0];
  double* ptry = &waypoints_y[0];
  Eigen::Map<Eigen::VectorXd> waypoints_x_eig(ptrx, 6);
  Eigen::Map<Eigen::VectorXd> waypoints_y_eig(ptry, 6);
  auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);
  double cte = polyeval(coeffs, 0);
  std::cout<<"coeffs is "<<coeffs<<std::endl;
  return 0;
}
