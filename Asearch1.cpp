#include <iostream>      // For printing output to console
#include <vector>        // For using 2D grid representation
#include <queue>         // For priority queue (used in open list)
#include <unordered_map> // (not used here but useful for hashing states if needed)
#include <cmath>         // For abs() function in heuristic
#include <algorithm>     // For reverse() when reconstructing path

// -------------------- NODE STRUCT --------------------
// Node represents one cell/state in the grid
struct Node {
    int x, y;       // Grid coordinates
    double g;       // Cost from start node to this node
    double h;       // Heuristic estimate (distance to goal)
    double f;       // Evaluation function f = g + h
    Node* parent;   // Backpointer to reconstruct path later

    // Constructor to initialize a Node
    Node(int x_, int y_, double g_=0, double h_=0, Node* parent_=nullptr)
        : x(x_), y(y_), g(g_), h(h_), f(g_+h_), parent(parent_) {}
};

// -------------------- PRIORITY QUEUE COMPARATOR --------------------
// This is used by the priority queue to sort nodes by f value (lowest first)
struct CompareNode {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;  // "greater than" means min-heap
    }
};

// -------------------- HEURISTIC FUNCTION --------------------
// Manhattan distance (good for 4-directional grid movement)
double heuristic(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

// -------------------- PATH RECONSTRUCTION --------------------
// Builds path by following parent pointers back from goal to start
std::vector<Node*> reconstruct_path(Node* goal) {
    std::vector<Node*> path;
    Node* current = goal;

    // Follow parent links back to start
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }

    // Reverse path so it goes from start -> goal
    std::reverse(path.begin(), path.end());
    return path;
}

// -------------------- A* SEARCH FUNCTION --------------------
std::vector<Node*> a_star(std::vector<std::vector<int>>& grid, Node* start, Node* goal) {
    int rows = grid.size();         // number of rows in grid
    int cols = grid[0].size();      // number of columns in grid

    // Open list: priority queue sorted by lowest f value
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> open;

    // Closed list: keeps track of visited states
    std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));

    // Initialize start node with heuristic
    start->h = heuristic(start->x, start->y, goal->x, goal->y);
    start->f = start->g + start->h;
    open.push(start); // Push start into open list

    // Movement directions: up, down, left, right
    int dx[4] = {-1, 1, 0, 0};
    int dy[4] = {0, 0, -1, 1};

    // While open list is not empty
    while (!open.empty()) {
        // Pick node with lowest f
        Node* current = open.top();
        open.pop();

        // Skip if we already expanded this node
        if (closed[current->x][current->y]) continue;

        // Mark node as visited
        closed[current->x][current->y] = true;

        // Check if we reached the goal
        if (current->x == goal->x && current->y == goal->y) {
            return reconstruct_path(current); // Return full path
        }

        // Expand neighbors
        for (int i = 0; i < 4; i++) {
            int nx = current->x + dx[i]; // new x position
            int ny = current->y + dy[i]; // new y position

            // Skip out-of-bounds positions
            if (nx < 0 || ny < 0 || nx >= rows || ny >= cols) continue;

            // Skip obstacles
            if (grid[nx][ny] == 1) continue;

            // Skip visited cells
            if (closed[nx][ny]) continue;

            // Cost to reach neighbor = current cost + 1
            double g_new = current->g + 1.0;

            // Heuristic for neighbor
            double h_new = heuristic(nx, ny, goal->x, goal->y);

            // Create neighbor node
            Node* neighbor = new Node(nx, ny, g_new, h_new, current);

            // Add neighbor to open list
            open.push(neighbor);
        }
    }

    // If open list becomes empty → no path exists
    return {};
}

// -------------------- MAIN: SIMULATED ROBOT LOOP --------------------
int main() {
    // Initialize a 5x5 empty grid (0 = free, 1 = obstacle)
    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };

    // Start node (top-left corner)
    Node* start = new Node(0, 0);

    // Goal node (bottom-right corner)
    Node* goal  = new Node(4, 4);

    // Current robot position starts at "start"
    Node* currentPos = start;

    // Robot keeps moving until it reaches the goal
    while (!(currentPos->x == goal->x && currentPos->y == goal->y)) {
        // Run A* from the robot's current position to the goal
        auto path = a_star(grid, new Node(currentPos->x, currentPos->y), goal);

        // If no path exists, stop
        if (path.empty()) {
            std::cout << "No path available!\n";
            break;
        }

        // Move one step along the path (second node in path)
        if (path.size() > 1) {
            currentPos = path[1];
        } else {
            break;
        }

        // Print robot's movement
        std::cout << "Robot moved to (" << currentPos->x << "," << currentPos->y << ")\n";

        // Simulate dynamic obstacle detection with sensors
        if (currentPos->x == 2 && currentPos->y == 2) {
            std::cout << "Sensor detected obstacle at (3,2)\n";
            grid[3][2] = 1; // Add obstacle to grid dynamically
        }
    }

    std::cout << "Reached goal or no path.\n";
    return 0;
}
