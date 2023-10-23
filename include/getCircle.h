#ifndef GETCIRCLE_H
#define GETCIRCLE_H
#include <eigen3/Eigen/Dense>
#include "parameters.h"
using namespace Eigen;  
MatrixXd getCircle(double R, double targetSpeed, double controllerFreq, double initX, double initY, double initZ);
#endif