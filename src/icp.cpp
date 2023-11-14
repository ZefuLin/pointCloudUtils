#include "icp.h"

void ICP::align(const PointCloud& source, const PointCloud& target) {
        
    PointCloud sourceCopy = source.copy();
    PointCloud targetCopy = target.copy();
    isConverge = false;

    for (int i = 0; i < maxIterations; i++) {
        // 找到对应点
        PointCloud correspondences;
        PointCloud closestPoints;
        for (const auto& point : sourceCopy.getPoints()) {
            Point targetPoint = targetCopy.findClosestPoint(point);
            if (point.distanceTo(targetPoint) < maxDistanceThreshold) {
                correspondences.addPoint(targetPoint);
                closestPoints.addPoint(point);
            }
        }
        
        // 计算质心
        Point sourceMean = sourceCopy.meanPoint();
        Point targetMean = targetCopy.meanPoint();

        // 移动点云到质心 (归一化)
        sourceCopy.centerPointCloud();
        targetCopy.centerPointCloud();

        // 构造协方差矩阵H
        Eigen::MatrixXd H = Eigen::Matrix2d::Zero(2, 2);
        for (size_t j = 0; j < closestPoints.size(); j++) {
            const Point& p1 = closestPoints.getPoint(j);
            const Point& p2 = correspondences.getPoint(j); 
            Eigen::Vector2d d1(p1.x, p1.y);
            Eigen::Vector2d d2(p2.x, p2.y);
            H += d1 * d2.transpose();
        }

        // 使用SVD分解求解R和t
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // ! 2D情况下 V*U.transpose 和 U*V.transpose 对应的角度是一样的，R的行列式也一样
        // ! 区别就是 V*U.transpose代表逆时针， U*V.transpose是顺时针
        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Vector2d t = Eigen::Vector2d(targetMean.x, targetMean.y) - R * Eigen::Vector2d(sourceMean.x, sourceMean.y);

        // 更新变换矩阵和源点云
        Eigen::Affine2d newTransform = Eigen::Translation2d(t) * R;
        transform = newTransform * transform;
        sourceCopy.transform(newTransform);

        // 检查收敛条件
        fitnessScore = 0.0;
        for (size_t j = 0; j < closestPoints.size(); j++) {
            fitnessScore += std::pow(closestPoints.getPoint(j).distanceTo(correspondences.getPoint(j)),2);
        }
        fitnessScore /= closestPoints.size();

        if (newTransform.affine().squaredNorm() < transformationEpsilon ||
            std::abs(fitnessScore - lastFitnessScore) < fitnessEpsilon) 
        {
            isConverge = true;
            break;
        }
        lastFitnessScore = fitnessScore;
    }
}