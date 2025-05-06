#include <Pathfinder.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <cmath>
#include <algorithm> // pour std::reverse
#include "define.h"

using namespace std;

typedef struct GridLocation GridLocation ;
typedef std::vector<std::vector<int>> Grid ;

void init_grid(Grid grid) {
    /*Construire grille avec les obstacles connus*/
}

void add_obstacle(int pos_x, int pos_y, int theta_robot, int list_distances[16], Grid& grid) {
    // Champ de vision theta. angle dtheta = theta/16.
    vector<GridLocation> directions = { {0,1}, {1,0}, {0,-1}, {-1,0}, {1,1}, {-1,-1}, {1,-1}, {-1,1} };

    for (int i = 0; i<16; i++) {
        int dist = list_distances[i] ;
        //A modifier pour prendre en compte taille robot.? dist_secu suffit?
        int k = floor(pos_x + sin(theta_robot+ ANGLE_VISION/2 - i*ANGLE_VISION/16)*(dist - DSECU)) ;
        int j = floor(pos_y + cos(theta_robot+ ANGLE_VISION/2 - i*ANGLE_VISION/16)*(dist - DSECU)) ;
        grid[k][j] = 1 ;
        for (const GridLocation& dir : directions) {
            grid[k+ dir.x][j+dir.y] == 1 ;
        }
    }
}
void suppr_obstacle(int pos_x, int pos_y, int theta_robot, int list_distances[16], Grid& grid) {
    // Champ de vision theta. angle dtheta = theta/16.
    vector<GridLocation> directions = { {0,1}, {1,0}, {0,-1}, {-1,0}, {1,1}, {-1,-1}, {1,-1}, {-1,1} };

    for (int i = 0; i<16; i++) {
        int dist = list_distances[i] ;
        //A modifier pour prendre en compte taille robot.? dist_secu suffit?
        int k = floor(pos_x + sin(theta_robot+ ANGLE_VISION/2 - i*ANGLE_VISION/16)*(dist - DSECU)) ;
        int j = floor(pos_y + cos(theta_robot+ ANGLE_VISION/2 - i*ANGLE_VISION/16)*(dist - DSECU)) ;
        grid[k][j] = 1 ;
        for (const GridLocation& dir : directions) {
            //On remet les obstacles à zéro.
            grid[k+ dir.x][j+dir.y] == 0 ;
        }
    }
}


// Structure pour représenter un GridLocation dans la grille
struct GridLocation {
    int x, y;
    GridLocation(int x=0, int y=0) : x(x), y(y) {}

    bool operator==(const GridLocation& other) const {
        return x == other.x && y == other.y;
    }

    bool operator<(const GridLocation& other) const {
        return tie(x, y) < tie(other.x, other.y);
    }
};

// Pour utiliser GridLocation dans un unordered_map
struct GridLocationHash {
    size_t operator()(const GridLocation& p) const {
        return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
    }
};

// Fonction pour calculer l'heuristique (distance de Manhattan)
int heuristic(const GridLocation& a, const GridLocation& b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

// Fonction pour reconstruire le chemin depuis l'arrivée jusqu'au départ
vector<GridLocation> reconstructPath(unordered_map<GridLocation, GridLocation, GridLocationHash>& cameFrom, GridLocation current) {
    vector<GridLocation> path;
    while (cameFrom.find(current) != cameFrom.end()) {
        path.push_back(current);
        current = cameFrom[current];
    }
    path.push_back(current); // Ajouter le GridLocation de départ
    reverse(path.begin(), path.end());
    return path;
}

// Fonction principale A*
vector<GridLocation> pathfinding_Astar(const Grid& grid, const GridLocation& start, const GridLocation& goal) {
    // Déplacements possibles : droite, bas, gauche, haut
    vector<GridLocation> directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };

    // Priority queue : (fScore, GridLocation)
    priority_queue<pair<int, GridLocation>, vector<pair<int, GridLocation>>, greater<pair<int, GridLocation>>> openSet;
    openSet.push({0, start});

    unordered_map<GridLocation, GridLocation, GridLocationHash> cameFrom;
    unordered_map<GridLocation, int, GridLocationHash> gScore;
    gScore[start] = 0;

    while (!openSet.empty()) {
        GridLocation current = openSet.top().second;
        openSet.pop();

        if (current == goal) {
            return reconstructPath(cameFrom, current);
        }

        for (const GridLocation& dir : directions) {
            GridLocation neighbor(current.x + dir.x, current.y + dir.y);

            // Vérifier que le voisin est dans la grille
            if (neighbor.x < 0 || neighbor.y < 0 || neighbor.x >= grid.size() || neighbor.y >= grid[0].size())
                continue;
            // Vérifier qu'on ne traverse pas un obstacle
            if (grid[neighbor.x][neighbor.y] == 0)
                continue;

            int tentativeGScore = gScore[current] + 1; // Coût pour avancer d'une case

            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                int fScore = tentativeGScore + heuristic(neighbor, goal);
                openSet.push({fScore, neighbor});
            }
        }
    }

    // Aucun chemin trouvé
    return {};
}


bool crosses(const GridLocation& obstacle, const GridLocation& a, const GridLocation& b) {
    // Vérifie si l'obstacle est sur le segment [a,b] 
    int minX = std::min(a.x, b.x);
    int maxX = std::max(a.x, b.x);
    int minY = std::min(a.y, b.y);
    int maxY = std::max(a.y, b.y);

    // Colinéarité 
    int dx1 = b.x - a.x;
    int dy1 = b.y - a.y;
    int dx2 = obstacle.x - a.x;
    int dy2 = obstacle.y - a.y;

    int cross = dx1 * dy2 - dy1 * dx2;

    if (cross != 0)
        return false; // pas colinéaire

    if (obstacle.x >= minX && obstacle.x <= maxX && obstacle.y >= minY && obstacle.y <= maxY)
        return true;

    return false;
}

bool can_go_straight(const std::vector<std::vector<int>>& grid, const GridLocation& init, const GridLocation& goal) {
    if (init.x == goal.x && init.y == goal.y)
        return false;
    int dx = std::abs(goal.x - init.x);
    int dy = std::abs(goal.y - init.y);

    int x = init.x;
    int y = init.y;

    int n = 1 + dx + dy;
    int x_inc = (goal.x > init.x) ? 1 : (goal.x < init.x) ? -1 : 0;
    int y_inc = (goal.y > init.y) ? 1 : (goal.y < init.y) ? -1 : 0;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;
    for (; n > 0; --n) {
        if (x < 0 || y < 0 || x >= grid.size() || y >= grid[0].size())
            return false;
        if (grid[x][y] == 0)
            return false;

        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
    }

    return true;
}

vector<GridLocation> reduce_path(const std::vector<std::vector<int>>& grid, const std::vector<GridLocation>& path) {
    if (path.size() <= 2)
        return path;

    if (can_go_straight(grid, path[0], path.back())) {
        return {path[0], path.back()};
    } else {
        int i = 0;
        int j = path.size();

        while (j - i > 1) {
            int h = (i + j) / 2;
            if (can_go_straight(grid, path[0], path[h])) {
                i = h;
            } else {
                j = h;
            }
        }

        std::vector<GridLocation> result = {path[0]};
        std::vector<GridLocation> reduced_subpath(path.begin() + i, path.end());
        std::vector<GridLocation> reduced_tail = reduce_path(grid, reduced_subpath);
        result.insert(result.end(), reduced_tail.begin(), reduced_tail.end());
        return result;
    }
}
