/*
 * @Author: Zefu Lin linzefu0826@outlook.com
 * @Date: 2023-11-01 21:03:50
 * @LastEditors: Zefu linzefu0826@outlook.com
 * @LastEditTime: 2024-01-30 17:29:02
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
    double theta;
    int intensity;  

    // 构造函数
    Point() :
        x(0.0), y(0.0), theta(0), intensity(0){}

    Point(double x, double y) :
        x(x), y(y), theta(0), intensity(0) {}

    Point(double x, double y, double theta) :
        x(x), y(y), theta(theta), intensity(0){}

    Point(double x, double y, double theta, int intensity) :
        x(x), y(y), theta(theta), intensity(intensity){}

    double distanceTo(const Point& other) const {
        return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y) , 2));
    }
};

class PointCloud {
// private:
    
public:

    std::vector<Point> points;

    PointCloud() {}
    // 用一个点的向量初始化
    PointCloud(const std::vector<Point> &points): points(points) {}
    ~PointCloud() {}

    void addPoint(const Point& point);
    bool removePoint(int index);
    size_t size() const;
    Point getPoint(int index) const;
    const std::vector<Point>& getPoints() const;
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

    PointCloud& operator=(const PointCloud& other) {
        // 避免自我赋值
        if (this != &other) {
            points = other.points;
        }
        return *this;
    }
    
    
    Point meanPoint() const;
    Point findClosestPoint(const Point& target) const;
    PointCloud copy() const;   
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