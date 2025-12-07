#include <iostream>
#include <Eigen/Dense>

int main(){
Eigen::Matrix2d A;

A<<1, 2,
   3, 4;

Eigen::Vector2d b(5, 6);

Eigen::Vector2d result = A * b;

std::cout << "A\n" << A << std::endl;
std::cout << "b\n" << b << std::endl;
std::cout << "Result \n" << result << std::endl;

}