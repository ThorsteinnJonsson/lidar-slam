#include <Eigen/Core>
#include <iostream>
#include <print>

int main() {
  std::println("lidar-slam initializing...");
  Eigen::Matrix3d mat = Eigen::Matrix3d::Random();
  std::cout << "Random 3x3 matrix:\n" << mat << std::endl;
  return 0;
}
