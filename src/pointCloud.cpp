/*
 * @Author: Zefu linzefu0826@outlook.com
 * @Date: 2023-11-10 19:38:23
 * @LastEditors: Zefu linzefu0826@outlook.com
 * @LastEditTime: 2023-11-11 13:46:52
 * @FilePath: /pointCloudUtils/src/pointCloud.cpp
 * @Description: 
 */

#include "pointcloud.h"

void PointCloud::addPoint(const Point& point){
    points.push_back(point);
}

bool PointCloud::removePoint(int index){
    if(index < points.size()){
        points.erase(points.begin() + index);
        return true;
    }else{
        std::cerr << "index overflow" << std::endl;
        return false;
    } 
}

size_t PointCloud::size() const {
    return points.size();
}

Point PointCloud::getPoint(int index) const {
    return points.at(index);
}

bool PointCloud::isEmpty()const {
    return points.empty();
}

void PointCloud::clear(){
    points.clear();
}
void PointCloud::saveToFile(const std::string& filename) const {
    std::ofstream outFile(filename);
    if (outFile.is_open()) {
        outFile << "Point Cloud Utils v1.0\n";
        outFile << "point cloud size: " << points.size() << "\n";
        outFile << "x y intensity timestamp tag priority isCenter\n";
        for (const auto& point : points) {
            outFile << point.x << " "
                    << point.y << " "
                    << point.intensity << " "
                    << point.timestamp << " "
                    << point.tag << " "
                    << point.priority << " "
                    << point.isCenter << "\n";
        }
        outFile.close();
    } else {
        std::cout << "Unable to open file";
    }
}


void PointCloud::loadFromFile(const std::string& filename){
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        std::string line;
        // 忽略前三行
        for (int i = 0; i < 3; ++i) {
            std::getline(inFile, line);
        }
        // 清空原有点云数据
        points.clear();
        // 逐行读取点的信息
        while (std::getline(inFile, line)) {
            std::istringstream iss(line);
            double x, y;
            int intensity, timestamp, tag, priority;
            bool isCenter;
            if (!(iss >> x >> y >> intensity >> timestamp >> tag >> priority >> isCenter)) {
                std::cerr << "data type error!" << std::endl;
                break; // 出错，跳出循环
            }
            points.emplace_back(x, y, intensity, timestamp, tag, priority, isCenter);
        }
        inFile.close();
    } else {
        std::cout << "Unable to open file";
    }
}
void PointCloud::printPointInfo(int index) const {
    if(index < points.size()){
        const Point& point = points[index];
        std::cout << "x: " << point.x << ", "
                << "y: " << point.y << ", "
                << "intensity: " << point.intensity << ", "
                << "timestamp: " << point.timestamp << ", "
                << "tag: " << point.tag << ", "
                << "priority: " << point.priority << ", "
                << "isCenter: " << (point.isCenter ? "true" : "false") << "\n";
    } else {
        std::cout << "Index out of range!\n";
    }
}

void PointCloud::merge(const PointCloud& other){
    for (const auto& point : other.getPoints()) {
        points.push_back(point);
    }
}

void PointCloud::transform(const Eigen::Matrix2d& rotation, const Eigen::Vector2d& translation) {
    for (auto& point : points) {
        Eigen::Vector2d vec(point.x, point.y);
        vec = rotation * vec + translation;
        point.x = vec(0);
        point.y = vec(1);
    }
}

void PointCloud::transform(const Eigen::Matrix3d& matrix) {
    for (auto& point : points) {
        Eigen::Vector3d vec(point.x, point.y, 1.0);
        vec = matrix * vec;
        point.x = vec(0) / vec(2);
        point.y = vec(1) / vec(2);
    }
}

void PointCloud::transform(const Eigen::Affine2d& transform) {
    for (auto& point : points) {
        Eigen::Vector2d vec(point.x, point.y);
        vec = transform * vec;
        point.x = vec(0);
        point.y = vec(1);
    }
}

void PointCloud::transformClockwise(double radian, const Eigen::Vector2d& translation) {
    Eigen::Matrix2d rotation;
    rotation <<  cos(radian), sin(radian),
                -sin(radian), cos(radian);
    transform(rotation, translation);
}

void PointCloud::transformCounterClockwise(double radian, const Eigen::Vector2d& translation) {
    Eigen::Matrix2d rotation;
    rotation << cos(radian), -sin(radian),
                sin(radian), cos(radian);
    transform(rotation, translation);
}

void PointCloud::voxelDownsample(double voxelSize){
    std::unordered_map<std::pair<int, int>, Voxel, pair_hash> voxelMap;

    // 将每个点分配到对应的体素中
    for (const auto& point : points) {
        int i = std::floor(point.x / voxelSize);
        int j = std::floor(point.y / voxelSize);
        Point voxelCenter((i + 0.5) * voxelSize, (j + 0.5) * voxelSize);
        auto key = std::make_pair(i, j);

        if (voxelMap.count(key) == 0) {
            voxelMap[key] = Voxel(voxelCenter);
        }

        double distance = point.distanceTo(voxelCenter);
        if (distance < voxelMap[key].minDistance) {
            voxelMap[key].minDistance = distance;
            voxelMap[key].closestPoint = point;
        }
    }

    // 用体素中心最近的点替换原来的点云
    points.clear();
    for (const auto& item : voxelMap) {
        points.push_back(item.second.closestPoint);
    }
}

void PointCloud::centerPointCloud(){
    Point mean = meanPoint();
    Eigen::Vector2d translation(-mean.x, -mean.y);
    Eigen::Matrix2d identityMatrix = Eigen::Matrix2d::Identity();
    transform(identityMatrix, translation);
}

const std::vector<Point>& PointCloud::getPoints() const{
    return points;
}

Point PointCloud::meanPoint() const {
    double meanX = 0.0, meanY = 0.0;
    for (const auto& point : points) {
        meanX += point.x;
        meanY += point.y;
    }
    if (!points.empty()) {
        meanX /= points.size();
        meanY /= points.size();
    }
    return Point(meanX, meanY);
}

Point PointCloud::findClosestPoint(const Point& target) const{
    if (points.empty()) {
        throw std::runtime_error("Point cloud is empty.");
    }
    
    double minDistance = std::numeric_limits<double>::max();
    Point closestPoint;
    // TODO 可以改成四叉树加速搜索
    for (const auto& point : points) {
        double distance = point.distanceTo(target);
        if (distance < minDistance) {
            minDistance = distance;
            closestPoint = point;
        }
    }
    return closestPoint;
}

PointCloud PointCloud::copy() const{
    return PointCloud(points);
}