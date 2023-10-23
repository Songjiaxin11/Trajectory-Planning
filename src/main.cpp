#include <iostream>
#include <cmath>
// #undef __ARM_NEON__ //在VEXCode中打开这两个注释
// #undef __ARM_NEON //在VEXCode中打开这两个注释
#include <eigen3/Eigen/Dense> // Include the Eigen library
using namespace Eigen;
using namespace std;

#include "getCircle.h"
#include "parameters.h"

int main()
{
    double R = 50.0;
    double targetSpeed = 100.0;
    double controllerFreq = 1.0;
    double initX = 0.0;
    double initY = 0.0;
    double initZ = 0.0;

    MatrixXd result = getCircle(R, targetSpeed, controllerFreq, initX, initY, initZ);

    cout << result << endl;
    return 0;
}