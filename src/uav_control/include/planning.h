#ifndef PLANNING_H
#define PLANNING_H
// author: MacManhHai
// email: hai372002@gmail.com

#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <iostream>
#include <tinyxml2.h>
#include <sstream>

struct Obstacle {
    std::string name;
    double x_min, x_max, y_min, y_max, z_min, z_max;
};

struct Point3D {
    double x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    bool operator==(const Point3D& other) const {
        return std::abs(x - other.x) < 1e-6 &&
               std::abs(y - other.y) < 1e-6 &&
               std::abs(z - other.z) < 1e-6;
    }
};

struct TreeNode {
    Point3D point;
    int parent_idx;
    TreeNode(const Point3D& pt, int parent) : point(pt), parent_idx(parent) {}
};

class Planning {
public:
    Planning(const std::vector<Obstacle>& obsList);
    static std::vector<Obstacle> loadObstaclesFromWorld(const std::string& world_file, double safety_margin = 1.0);

    bool isInCollision(const Point3D& point) const;
    bool isPathInCollision(const Point3D& p1, const Point3D& p2, double step = 0.15) const;
    Point3D randomPoint() const;
    double distance(const Point3D& p1, const Point3D& p2) const;
    Point3D steer(const Point3D& from, const Point3D& to, double step = 1.0) const;

    std::vector<Point3D> retracePath(const std::vector<TreeNode>& tree, int idx) const;
    std::vector<Point3D> buildPath(const std::vector<TreeNode>& tree_start, int idx_start,
                                   const std::vector<TreeNode>& tree_goal, int idx_goal) const;
    std::vector<Point3D> simplifyPath(const std::vector<Point3D>& path) const;
    std::vector<Point3D> densifyPath(const std::vector<Point3D>& path, int factor) const;
    std::vector<Point3D> biRRT(const Point3D& start, const Point3D& goal, int max_iter = 5000) const;

private:
    std::vector<Obstacle> obstacles;
    mutable std::mt19937 rng;
};

#endif // PLANNING_H
