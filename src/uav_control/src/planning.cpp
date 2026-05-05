#include "planning.h"
#include <algorithm>
#include <limits>

// Constructor
Planning::Planning(const std::vector<Obstacle>& obsList)
    : obstacles(obsList), rng(std::random_device{}()) {}

std::vector<Obstacle> Planning::loadObstaclesFromWorld(const std::string& world_file, double safety_margin) {
    std::vector<Obstacle> obstacles;
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLError result = doc.LoadFile(world_file.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        std::cerr << "Cannot load world file: " << world_file << ". Error code: " << result << std::endl;
        return obstacles;
    }
    tinyxml2::XMLElement* root = doc.RootElement();
    tinyxml2::XMLElement* world = root->FirstChildElement("world");
    if (!world) {
        std::cerr << "Không tìm thấy thẻ <world> trong file SDF!" << std::endl;
        return obstacles;
    }
    // Loop through models in <world>
    for (tinyxml2::XMLElement* model = world->FirstChildElement("model"); model; model = model->NextSiblingElement("model")) {
        const char* name = model->Attribute("name");
        tinyxml2::XMLElement* pose = model->FirstChildElement("pose");
        tinyxml2::XMLElement* size = nullptr;
        // Follow the correct structure: link -> collision -> geometry -> box -> size
        tinyxml2::XMLElement* geometry = model->FirstChildElement("link");
        if (geometry) geometry = geometry->FirstChildElement("collision");
        if (geometry) geometry = geometry->FirstChildElement("geometry");
        if (geometry) geometry = geometry->FirstChildElement("box");
        if (geometry) size = geometry->FirstChildElement("size");

        if (pose && size) {
            double x, y, z, dx, dy, dz;
            std::istringstream iss_pose(pose->GetText());
            iss_pose >> x >> y >> z;
            std::istringstream iss_size(size->GetText());
            iss_size >> dx >> dy >> dz;

            obstacles.push_back({
                name ? name : "unknown",
                x - dx/2 - safety_margin,
                x + dx/2 + safety_margin,
                y - dy/2 - safety_margin,
                y + dy/2 + safety_margin,
                z - dz/2 - safety_margin,
                z + dz/2 + safety_margin
            });
        }
    }
    return obstacles;
}
    
// Check point in collision?
bool Planning::isInCollision(const Point3D& point) const {
    for (const auto& obs : obstacles) {
        if (obs.x_min <= point.x && point.x <= obs.x_max &&
            obs.y_min <= point.y && point.y <= obs.y_max &&
            obs.z_min <= point.z && point.z <= obs.z_max) {
            return true;
        }
    }
    return false;
}

// Check path in collision?
bool Planning::isPathInCollision(const Point3D& p1, const Point3D& p2, double step) const {
    double dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
    double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (norm < step) return isInCollision(p2);

    int num_steps = static_cast<int>(norm / step);
    for (int i = 1; i <= num_steps; ++i) {
        double t = static_cast<double>(i) / num_steps;
        Point3D inter_point(p1.x + dx * t, p1.y + dy * t, p1.z + dz * t);
        if (isInCollision(inter_point)) return true;
    }
    return false;
}

// Random point in free space
Point3D Planning::randomPoint() const {
    std::uniform_real_distribution<double> xdist(-10.0, 10.0);
    std::uniform_real_distribution<double> ydist(-10.0, 10.0);
    std::uniform_real_distribution<double> zdist(0.0, 4.0);
    while (true) {
        Point3D point(xdist(rng), ydist(rng), zdist(rng));
        if (!isInCollision(point)) return point;
    }
}

// Calculate Euclidean distance between two points
double Planning::distance(const Point3D& p1, const Point3D& p2) const {
    double dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// Steer from 'from' to 'to' by 'step' distance
Point3D Planning::steer(const Point3D& from, const Point3D& to, double step) const {
    double dx = to.x - from.x, dy = to.y - from.y, dz = to.z - from.z;
    double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (norm > 1e-6) {
        if (norm > step) {
            double scale = step / norm;
            return Point3D(from.x + dx * scale, from.y + dy * scale, from.z + dz * scale);
        }
        return to;
    }
    return from;
}

// Retrace path from tree
std::vector<Point3D> Planning::retracePath(const std::vector<TreeNode>& tree, int idx) const {
    std::vector<Point3D> path;
    while (idx >= 0 && idx < static_cast<int>(tree.size())) {
        path.push_back(tree[idx].point);
        idx = tree[idx].parent_idx;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// Build full path from start and goal trees
std::vector<Point3D> Planning::buildPath(
    const std::vector<TreeNode>& tree_start, int idx_start,
    const std::vector<TreeNode>& tree_goal, int idx_goal) const
{
    auto path_start = retracePath(tree_start, idx_start);
    auto path_goal = retracePath(tree_goal, idx_goal);
    if (!path_goal.empty()) path_goal.pop_back(); // remove duplicate point
    std::reverse(path_goal.begin(), path_goal.end());
    path_start.insert(path_start.end(), path_goal.begin(), path_goal.end());
    return path_start;
}

std::vector<Point3D> Planning::simplifyPath(const std::vector<Point3D>& path) const {
    if (path.empty()) return {};

    std::vector<Point3D> simplified_path;
    simplified_path.push_back(path[0]);
    int current_idx = 0;

    for (int next_idx = 1; next_idx < (int)path.size(); ++next_idx) {
        if (isPathInCollision(path[current_idx], path[next_idx])) {
            // if collision, add the last valid point
            simplified_path.push_back(path[next_idx - 1]);
            current_idx = next_idx - 1;
        }
    }
    // Add the last point
    simplified_path.push_back(path.back());
    return simplified_path;
}

std::vector<Point3D> Planning::densifyPath(const std::vector<Point3D>& path, int target_points) const {
    std::vector<Point3D> dense_path;
    if (path.size() < 2 || target_points <= 2) return path;

    // 1. Compute lengths of each segment and total length
    std::vector<double> segment_lengths;
    std::vector<double> cumulative_lengths;
    double total_length = 0.0;

    for (size_t i = 1; i < path.size(); ++i) {
        double len = distance(path[i - 1], path[i]);
        segment_lengths.push_back(len);
        total_length += len;
        cumulative_lengths.push_back(total_length);
    }

    // 2. Compute spacing between points to interpolate
    double spacing = total_length / (target_points - 1);
    dense_path.push_back(path[0]); // start point

    double current_dist = spacing;
    size_t seg_idx = 0;

    for (int i = 1; i < target_points - 1; ++i) {
        // Find the segment containing the insertion point
        while (seg_idx < segment_lengths.size() &&
               current_dist > cumulative_lengths[seg_idx]) {
            ++seg_idx;
        }

        if (seg_idx >= segment_lengths.size()) break;

        const Point3D& p1 = path[seg_idx];
        const Point3D& p2 = path[seg_idx + 1];

        double seg_start_dist = (seg_idx == 0) ? 0.0 : cumulative_lengths[seg_idx - 1];
        double segment_len = segment_lengths[seg_idx];
        double ratio = (current_dist - seg_start_dist) / segment_len;

        Point3D interp(
            p1.x + ratio * (p2.x - p1.x),
            p1.y + ratio * (p2.y - p1.y),
            p1.z + ratio * (p2.z - p1.z)
        );

        dense_path.push_back(interp);
        current_dist += spacing;
    }

    dense_path.push_back(path.back()); // end point
    return dense_path;
}

// Bi-RRT algorithm (bidirectional RRT)
std::vector<Point3D> Planning::biRRT(const Point3D& start, const Point3D& goal, int max_iter) const {
    std::vector<TreeNode> tree_start;
    std::vector<TreeNode> tree_goal;
    tree_start.emplace_back(start, -1);
    tree_goal.emplace_back(goal, -1);

    for (int iter = 0; iter < max_iter; ++iter) {
        Point3D rand_point_start = randomPoint();
        Point3D rand_point_goal = randomPoint();

        // Find the nearest point in each tree
        int nearest_start = 0;
        double min_dist_start = std::numeric_limits<double>::max();
        for (int i = 0; i < (int)tree_start.size(); ++i) {
            double d = distance(tree_start[i].point, rand_point_start);
            if (d < min_dist_start) {
                min_dist_start = d;
                nearest_start = i;
            }
        }
        int nearest_goal = 0;
        double min_dist_goal = std::numeric_limits<double>::max();
        for (int i = 0; i < (int)tree_goal.size(); ++i) {
            double d = distance(tree_goal[i].point, rand_point_goal);
            if (d < min_dist_goal) {
                min_dist_goal = d;
                nearest_goal = i;
            }
        }

        // Expand trees
        Point3D new_start = steer(tree_start[nearest_start].point, rand_point_start);
        Point3D new_goal = steer(tree_goal[nearest_goal].point, rand_point_goal);

        if (!isPathInCollision(tree_start[nearest_start].point, new_start)) {
            tree_start.emplace_back(new_start, nearest_start);
        }
        if (!isPathInCollision(tree_goal[nearest_goal].point, new_goal)) {
            tree_goal.emplace_back(new_goal, nearest_goal);
        }

        // Check connection
        double d_connect = distance(new_start, new_goal);
        if (d_connect < 2.0 && !isPathInCollision(new_start, new_goal)) {
            int idx_start = (int)tree_start.size() - 1;
            int idx_goal = (int)tree_goal.size() - 1;
            return buildPath(tree_start, idx_start, tree_goal, idx_goal);
        }
    }
    // No path found
    return std::vector<Point3D>();
}
