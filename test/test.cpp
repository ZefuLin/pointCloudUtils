#include <iostream>
#include <vector>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "icp.h"
#include "pointcloud.h"

int main(int argc, char const *argv[])
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test/000000.pcd", *cloud_in) == -1)
    {             
        PCL_ERROR ("Couldn't read file ./test/000000.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << cloud_in->points.size () 
    << " data points from ./test/000000.pcd" << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("../test/000100.pcd", *cloud_out) == -1)
    {
        PCL_ERROR ("Couldn't read file ../test/000001.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << cloud_out->points.size () 
    << " data points from ../test/000001.pcd" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(1);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-2);
    icp.setEuclideanFitnessEpsilon(1e-2);
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_result (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*pcl_result);
    ICP myicp;
    PointCloud pc_in;
    PointCloud pc_out;

    for (const auto& point : cloud_in->points){
        Point p;
        p.x = point.x;
        p.y = point.y;
        pc_in.addPoint(p);
    }
    for (const auto& point : cloud_out->points){
        Point p;
        p.x = point.x;
        p.y = point.y;
        pc_out.addPoint(p);
    }
    myicp.setMaxIterations(100);
    myicp.setMaxDistanceThreshold(1);
    myicp.setTransformationEpsilon(1e-2);
    myicp.setFitnessEpsilon(1e-2);
    myicp.align(pc_in, pc_out);

    std::cout << "-------------------------------------------------------" << std::endl;
    std::cout << "my icp " << std::endl;
    std::cout << "Fitness score: " << myicp.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix: " << std::endl << myicp.getTransformation().matrix() << std::endl;

    std::cout << "pcl icp " << std::endl;
    std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix: " << std::endl << icp.getFinalTransformation() << std::endl;
    
    PointCloud my_result(pc_in.copy());
    my_result.transform(myicp.getTransformation());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : my_result.getPoints()){
        pcl::PointXYZ p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        cloud3->push_back(p);
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(255, 255, 255);
    // 源点云 红色 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_in, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_in, red, "cloud_in");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "cloud_in");
    // // PCL ICP点云 绿色
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(pcl_result, 0, 255, 0);
    // viewer->addPointCloud<pcl::PointXYZ>(pcl_result, green, "pcl_result");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "pcl_result");
    // my ICP点云 黑色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> yellow(cloud3, 0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud3, yellow, "cloud3");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "cloud3");  
    // 目标点云 蓝色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(cloud_out, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_out, blue, "cloud_out");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 9, "cloud_out");
    
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    while (!viewer->wasStopped())
    {
    viewer->spinOnce(100);
    }
    return 0;       
}
