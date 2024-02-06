#include "ICP2D.h"

bool ICP2D::align(const PointCloud& source, const PointCloud& target)
{
    isConverge = false;

    int32_t dim = 2;
    size_t num_src = source.size();
    size_t num_tgt = target.size();

    double* M = (double*)calloc(dim*num_tgt,sizeof(double));
    double* T = (double*)calloc(dim*num_src,sizeof(double));

    for (int k = 0; k < num_src; ++k){
        T[k*dim+0] = source.points[k].x;
        T[k*dim+1] = source.points[k].y;
    }

    for (int k = 0; k < num_tgt; ++k){
        M[k*dim+0] = target.points[k].x;
        M[k*dim+1] = target.points[k].y;
    }

    Matrix R = Matrix::eye(2);
    Matrix t(2,1);

    IcpPointToPoint icp(M,num_tgt,dim);

    icp.setMaxIterations(maxIterations);
    icp.setMinDeltaParam(transformationEpsilon);

    double residual = icp.fit(T,num_src,R,t,maxDistanceThreshold);

    free(M);
    free(T);

    if (residual > fitnessEpsilon) return isConverge;

    isConverge = true;

    Eigen::Matrix2d icp_rot = Eigen::Matrix2d::Identity();
    icp_rot(0,0) = R.val[0][0];
    icp_rot(0,1) = R.val[0][1];
    icp_rot(1,0) = R.val[1][0];
    icp_rot(1,1) = R.val[1][1];
    Eigen::Translation2d icp_trans;
    icp_trans.x() = t.val[0][0];
    icp_trans.y() = t.val[1][0];

    transform = icp_trans * icp_rot;
    fitnessScore = residual;
    return isConverge;
}