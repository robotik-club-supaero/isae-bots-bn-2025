#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <queue>
#include <unordered_map>
#include <tuple>   // pour std::tie
#include <algorithm> // pour std::reverse
#include <functional>

struct GridLocation {
    int x, y;
    GridLocation(int x=0, int y=0) : x(x), y(y) {}

    bool operator==(const GridLocation& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const GridLocation& other) const {
        return std::tie(x, y) < std::tie(other.x, other.y);
    }
};

// Hash spécial pour GridLocation (utile dans unordered_map)
struct GridLocationHash {
    std::size_t operator()(const GridLocation& p) const {
        return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
    }
};

// Fonctions
int heuristic(const GridLocation& a, const GridLocation& b);

std::vector<GridLocation> reconstructPath(
    std::unordered_map<GridLocation, GridLocation, GridLocationHash>& cameFrom,
    GridLocation current
);

std::vector<GridLocation> pathfinding_Astar(
    const std::vector<std::vector<int>>& grid,
    const GridLocation& start,
    const GridLocation& goal
);

bool crosses(const GridLocation& obstacle, const GridLocation& a, const GridLocation& b) ;

bool can_go_straight(const std::vector<std::vector<int>>& grid, const GridLocation& init, const GridLocation& goal) ;

vector<GridLocation> reduce_path(const std::vector<std::vector<int>>& grid, const std::vector<GridLocation>& path) ;

void add_obstacle(int pos_x, int pos_y, int theta_robot, int list_distances[16], Grid& grid) ;

void suppr_obstacle(int pos_x, int pos_y, int theta_robot, int list_distances[16], Grid& grid) ;

void init_grid(Grid grid) ;

#endif // PATHFINDER_H