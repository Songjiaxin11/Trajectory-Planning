#include "parameters.h"
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
MatrixXd getCircle(double R, double targetSpeed, double controllerFreq, double initX, double initY, double initZ)
{
    double stepDis = targetSpeed / controllerFreq;
    double stepTheta = stepDis / R;
    int numSteps = static_cast<int>(2 * PI / stepTheta + 1);

    VectorXd theta = VectorXd::LinSpaced(numSteps, 0, 2 * PI);

    VectorXd x = R * theta.array().cos() + initX;
    VectorXd y = R * theta.array().sin() + initY;
    VectorXd z = VectorXd::Ones(numSteps) * initZ;

    MatrixXd speedMatrix = MatrixXd::Identity(numSteps, numSteps) * (-1);
    for (int i = 0; i < numSteps; ++i)
    {
        speedMatrix(i, (i - 1 + numSteps) % numSteps) = 1;
    }
    speedMatrix.transposeInPlace();

    VectorXd vx = speedMatrix * x * controllerFreq;
    VectorXd vy = speedMatrix * y * controllerFreq;
    VectorXd vz = speedMatrix * z * controllerFreq;

    MatrixXd trajectory(numSteps,6);
    trajectory << x, y, z, vx, vy, vz;
    trajectory.transposeInPlace();

    return trajectory;
}