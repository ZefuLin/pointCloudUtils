/*
 * @Author: Zefu Lin linzefu0826@outlook.com
 * @Date: 2023-11-01 21:05:08
 * @LastEditors: Zefu linzefu0826@outlook.com
 * @LastEditTime: 2023-11-14 15:39:01
 * @FilePath: /pointCloudUtils/include/icp.h
 * @Description: 2D ICP
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef ICP_H_
#define ICP_H_

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "pointcloud.h"

class ICP {
private:
    int maxIterations;
    double maxDistanceThreshold;
    double transformationEpsilon;
    double fitnessEpsilon;
    double fitnessScore;
    double lastFitnessScore;
    bool isConverge = false;
    Eigen::Affine2d transform = Eigen::Affine2d::Identity();

public:
    ICP(int maxIterations = 100, double maxDistanceThreshold = 1.0, 
        double transformationEpsilon = 1e-6, double fitnessEpsilon = 1e-6)
        : maxIterations(maxIterations), maxDistanceThreshold(maxDistanceThreshold),
          transformationEpsilon(transformationEpsilon), fitnessEpsilon(fitnessEpsilon) {}

    void align(const PointCloud& source, const PointCloud& target);

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

#endif // * ICP_H_