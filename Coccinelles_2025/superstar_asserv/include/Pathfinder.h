#include <vector>
#include "defines/math.hpp"

struct point {float x ; float y ;};
struct list_points {point first ; };

list_points pathfinding_Astar(int* grid[100][100], point pos, point target, int m_vision[16] );
