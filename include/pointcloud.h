/*
 * @Author: Zefu Lin linzefu0826@outlook.com
 * @Date: 2023-11-01 21:03:50
 * @LastEditors: Zefu linzefu0826@outlook.com
 * @LastEditTime: 2023-11-13 18:12:33
 * @FilePath: /pointCloudUtils/include/pointcloud.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef POINTCLOUD_H_
#define POINTCLOUD_H_

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <Eigen/Dense>

class Point {
public:
    double x;  
    double y;  
    int intensity;  
    int timestamp;  
    int tag;  
    int priority;  
    bool isCenter;  

    // 构造函数
    Point() :
        x(0.0), y(0.0), intensity(0), timestamp(0), tag(0), priority(0), isCenter(false){}

    Point(double x, double y) :
        x(x), y(y), intensity(0), timestamp(0), tag(0), priority(0), isCenter(false) {}

    Point(double x, double y, int intensity, int timestamp, int tag, int priority, bool isCenter) :
        x(x), y(y), intensity(intensity), timestamp(timestamp), tag(tag), priority(priority), isCenter(isCenter) {}

    double distanceTo(const Point& other) const {
        return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y) , 2));
    }
};

class PointCloud {
private:
    std::vector<Point> points;
public:
    PointCloud() {}
    // 用一个点的向量初始化
    PointCloud(const std::vector<Point> &points): points(points) {}
    ~PointCloud() {}

    void addPoint(const Point& point);
    bool removePoint(int index);
    size_t size() const;
    Point getPoint(int index) const;
    bool isEmpty() const;
    void clear();
    void saveToFile(const std::string& filename) const;
    void loadFromFile(const std::string& filename);
    void printPointInfo(int index) const;
    void merge(const PointCloud& other);
    void transform(const Eigen::Matrix2d& rotation, const Eigen::Vector2d& translation);
    void transform(const Eigen::Matrix3d& matrix);
    void transform(const Eigen::Affine2d& transform);
    void transformClockwise(double radian, const Eigen::Vector2d& translation);
    void transformCounterClockwise(double radian, const Eigen::Vector2d& translation);
    void voxelDownsample(double voxelSize);
    void centerPointCloud();
    
    const std::vector<Point>& getPoints() const;
    // ! 可能要修改计算点云中心的方式，源点云和目标点云中心的平移会很大程度影响最终的平移
    Point meanPoint() const;
    Point findClosestPoint(const Point& target) const;
    PointCloud copy() const;

    // TODO
    // 搜索和查询操作：例如找到距离某个点最近的点，或者找到满足某个条件（例如强度在某个范围内）的所有点。
    // 点云的滤波和去噪：例如实现一个函数来移除强度低于某个阈值的点，或者移除离群点。
    // 获取点云的统计信息：例如获取点云的中心、获取点云的边界（最大和最小的x和y值）、获取点云的大小（点的数量）等
    
};

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // 使用大质数和位操作来混合哈希值
        return h1 ^ (h2 << 1) ^ (h1 >> 1) * 73856093;
    }
};

struct Voxel {
    Point center;
    Point closestPoint;
    double minDistance = std::numeric_limits<double>::max();
    Voxel() = default;
    Voxel(const Point& center) : center(center) {}
};

#endif /* POINTCLOUD_H_ */