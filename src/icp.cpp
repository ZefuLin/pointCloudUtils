#include "icp.h"

void ICP::align(const PointCloud& source, const PointCloud& target) {
        
    PointCloud sourceCopy = source.copy();

    for (int i = 0; i < maxIterations; i++) {
        // 找到对应点
        PointCloud correspondences;
        for (const auto& point : sourceCopy.getPoints()) {
            Point targetPoint = target.findClosestPoint(point);
            if (point.distanceTo(targetPoint) < maxDistanceThreshold) {
                correspondences.addPoint(targetPoint);
            }
        }

        // 计算质心
        Point sourceMean = sourceCopy.meanPoint();
        Point targetMean = correspondences.meanPoint();

        // 构造协方差矩阵H
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (int j = 0; j < sourceCopy.size(); j++) {
            const Point& p1 = sourceCopy.getPoint(j);
            const Point& p2 = correspondences.getPoint(j);
            Eigen::Vector2d d1(p1.x - sourceMean.x, p1.y - sourceMean.y);
            Eigen::Vector2d d2(p2.x - targetMean.x, p2.y - targetMean.y);
            H += d1 * d2.transpose();
        }

        // 使用SVD分解求解R和t
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
        Eigen::Vector2d t = Eigen::Vector2d(targetMean.x, targetMean.y) - R * Eigen::Vector2d(sourceMean.x, sourceMean.y);

        // 更新变换矩阵和源点云
        Eigen::Affine2d newTransform = Eigen::Translation2d(t) * R;
        transform = newTransform * transform;
        sourceCopy.transform(newTransform);

        // 检查收敛条件
        fitnessScore = 0.0;
        for (int j = 0; j < sourceCopy.size(); j++) {
            fitnessScore += sourceCopy.getPoint(j).distanceTo(correspondences.getPoint(j));
        }
        fitnessScore /= sourceCopy.size();

        if (newTransform.affine().squaredNorm() < transformationEpsilon ||
            std::abs(fitnessScore - lastFitnessScore) < fitnessEpsilon) {
            break;
        }
        lastFitnessScore = fitnessScore;
    }
}