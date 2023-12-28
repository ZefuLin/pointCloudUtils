#ifndef TINY_ICP_H_
#define TINY_ICP_H_

#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include "pointcloud.h"
#include "icpPointToPoint.h"

class ICP2D 
{
private:
    int maxIterations;
    double maxDistanceThreshold;
    double transformationEpsilon;
    double fitnessEpsilon;
    double fitnessScore;
    bool isConverge = false;
    Eigen::Affine2d transform = Eigen::Affine2d::Identity();

public:
    ICP2D(int maxIterations = 100, double maxDistanceThreshold = 1.0,
          double transformationEpsilon = 1e-6, double fitnessEpsilon = 1e-6):
    maxIterations(maxIterations), maxDistanceThreshold(maxDistanceThreshold),
    transformationEpsilon(transformationEpsilon), fitnessEpsilon(fitnessEpsilon){}

    ~ICP2D(){}

    bool align(const PointCloud& source, const PointCloud& target);

    double getFitnessScore() const {
        return fitnessScore;
    }

    bool hasConverged() const {
        return isConverge;
    }

    Eigen::Affine2d getTransformation() const {
        return transform;
    }

    void setMaxIterations(int maxIterations) {
        this->maxIterations = maxIterations;
    }

    void setMaxDistanceThreshold(double maxDistanceThreshold) {
        this->maxDistanceThreshold = maxDistanceThreshold;
    }

    void setTransformationEpsilon(double transformationEpsilon) {
        this->transformationEpsilon = transformationEpsilon;
    }

    void setFitnessEpsilon(double fitnessEpsilon) {
        this->fitnessEpsilon = fitnessEpsilon;
    }

};

#endif // * TINY_ICP_H_