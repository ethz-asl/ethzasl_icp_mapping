#ifndef __ALIASES_H
#define __ALIASES_H

#include "pointmatcher/PointMatcher.h"

typedef float Scalar;

typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

typedef PointMatcher<Scalar> PM;

typedef PM::TransformationParameters TP;
typedef PM::DataPoints DP;

#endif
