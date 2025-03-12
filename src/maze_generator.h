#ifndef __MAZE_GENERATOR_AGENT__H
#define __MAZE_GENERATOR_AGENT__H 

#include "enviro.h"

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <random>
#include <numeric>
#include <queue>
#include <functional>

using namespace enviro;

// Constants for maze dimensions and wall size.
const int MAZE_WIDTH = 15;
const int MAZE_HEIGHT = 15;
const double WALL_SIZE = 40.0;
const double MAZE_START_X = -MAZE_WIDTH * WALL_SIZE / 2.0 + WALL_SIZE / 2.0;
const double MAZE_START_Y = -MAZE_HEIGHT * WALL_SIZE / 2.0 + WALL_SIZE / 2.0;
const double MAZE_CELL_SIZE = WALL_SIZE;
const double EPSILON = 1e-3;

// Represents a single cell in the maze.
class MazeCell {
public:
    bool topWall = true;
    bool rightWall = true;
    bool bottomWall = true;
    bool leftWall = true;
    int row, col;

    MazeCell() : row(0), col(0) {}
    MazeCell(int r, int c) : row(r), col(c) {}
    
    void removeWall(int direction) {
        switch(direction) {
            case 0: topWall = false; break;    // Top
            case 1: bottomWall = false; break; // Bottom
            case 2: leftWall = false; break;   // Left
            case 3: rightWall = false; break;  // Right
        }
    }
    
    bool isBorderCell() const {
        return row == 0 || row == MAZE_HEIGHT - 1 || col == 0 || col == MAZE_WIDTH - 1;
    }
    
    int getBorderDirection() const {
        if (row == 0) return 0;                // Top border
        if (row == MAZE_HEIGHT - 1) return 1;  // Bottom border
        if (col == 0) return 2;                // Left border
        if (col == MAZE_WIDTH - 1) return 3;   // Right border
        return -1;                             // Not a border cell
    }
};

// Represents a cell as x-y coordinates in the maze.
class Point {
public:
    int x, y;
    Point(): x(0), y(0) {}
    Point(int x, int y) : x(x), y(y) {}

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Point& other) const {
        return !(*this == other);
    }
};

// Specialization of std::hash for Point.
namespace std {
    template <>
    struct hash<Point> {
        size_t operator()(const Point& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
};

class MazeGeneratorController : public Process, public AgentInterface {
public:
    MazeGeneratorController() : Process(), AgentInterface(), id(0) {}

    void init() {
        initializeMaze();
        generateMaze();
        addExits();
        placeWalls();
    }

    void start() {
        Point start(MAZE_HEIGHT / 2, MAZE_WIDTH / 2);
        std::vector<Point> path = bfs(start);
        json pathData = generatePathData(path);
        emit(Event("path_found", pathData));
    }
    
    void update() {}
    void stop() {}

private:
    std::unordered_map<int, std::unordered_set<MazeCell*>> sets;
    int id;
    std::vector<std::vector<MazeCell>> maze;
    std::default_random_engine rng;
    std::unordered_set<MazeCell*> exits;

    void initializeMaze() {
        maze = std::vector<std::vector<MazeCell>>(MAZE_HEIGHT, std::vector<MazeCell>(MAZE_WIDTH));
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            for (int x = 0; x < MAZE_WIDTH; ++x) {
                maze[y][x] = MazeCell(y, x);
            }
        }
        rng = std::default_random_engine(std::random_device{}());
    }

    void generateMaze() {
        id = 0;
        sets.clear();

        for (int row = 0; row < MAZE_HEIGHT; ++row) {
            initializeRowCells(row);
            
            if (row < MAZE_HEIGHT - 1) {
                createHorizontalConnections(row, true);
                createVerticalConnections(row);
            } else {
                createHorizontalConnections(row, false);
            }
        }
    }

    void initializeRowCells(int row) {
        for (int col = 0; col < MAZE_WIDTH; ++col) {
            if (findSet(maze[row][col]) == -1) {
                sets[id].insert(&maze[row][col]);
                ++id;
            }
        }
    }

    void createHorizontalConnections(int row, bool allowRandom) {
        for (int col = 0; col < MAZE_WIDTH - 1; ++col) {
            MazeCell& cell = maze[row][col];
            MazeCell& rightNeighbor = maze[row][col + 1];
            
            bool shouldConnect = !allowRandom || std::uniform_int_distribution<>(0, 1)(rng) == 0;
            
            if (findSet(cell) != findSet(rightNeighbor) && shouldConnect) {
                cell.rightWall = false;
                rightNeighbor.leftWall = false;
                mergeSets(findSet(cell), findSet(rightNeighbor));
            }
        }
    }

    void createVerticalConnections(int row) {
        std::unordered_set<int> currRowSetIds;
        
        for (int col = 0; col < MAZE_WIDTH; ++col) {
            currRowSetIds.insert(findSet(maze[row][col]));
        }
        
        for (int col = 0; col < MAZE_WIDTH; ++col) {
            MazeCell& cell = maze[row][col];
            int setId = findSet(cell);
            
            bool shouldLinkDown = std::uniform_int_distribution<>(0, 1)(rng) == 0 || 
                                 currRowSetIds.find(setId) != currRowSetIds.end();
            
            if (shouldLinkDown) {
                MazeCell& downNeighbor = maze[row + 1][col];
                cell.bottomWall = false;
                downNeighbor.topWall = false;
                sets[setId].insert(&downNeighbor);
                
                if (currRowSetIds.find(setId) != currRowSetIds.end()) {
                    currRowSetIds.erase(setId);
                }
            }
        }
    }

    void addExits() {
        std::unordered_map<int, std::vector<MazeCell*>> borderCells;
        collectBorderCells(borderCells);

        for (auto& [direction, cells] : borderCells) {
            std::uniform_int_distribution<int> dist(0, cells.size() - 1);
            int exitIndex = dist(rng);
            MazeCell* exitCell = cells[exitIndex];
            exits.insert(exitCell);
            
            exitCell->removeWall(direction);
        }
    }

    void placeWalls() {
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            for (int x = 0; x < MAZE_WIDTH; ++x) {
                placeWallsForCell(y, x);
            }
        }
        
        placeBorderWalls();
    }
    
    void placeWallsForCell(int y, int x) {
        double _x = MAZE_START_X + x * WALL_SIZE;
        double _y = MAZE_START_Y + y * WALL_SIZE;

        if (maze[y][x].topWall) {
            addWall("HWall", _x, _y - WALL_SIZE / 2.0);
        }
        if (maze[y][x].leftWall) {
            addWall("VWall", _x - WALL_SIZE / 2.0, _y);
        }
    }
    
    void addWall(const std::string& type, double x, double y) {
        add_agent(type, x, y, 0, {{"fill", "gray"}, {"stroke", "None"}});
    }
    
    void placeBorderWalls() {
        for (int x = 0; x < MAZE_WIDTH; ++x) {
            double _x = MAZE_START_X + x * WALL_SIZE;
            if (maze[MAZE_HEIGHT - 1][x].bottomWall) {
                addWall("HWall", _x, MAZE_START_Y + MAZE_HEIGHT * WALL_SIZE - WALL_SIZE / 2.0);
            }
        }
        
        for (int y = 0; y < MAZE_HEIGHT; ++y) {
            double _y = MAZE_START_Y + y * WALL_SIZE;
            if (maze[y][MAZE_WIDTH - 1].rightWall) {
                addWall("VWall", MAZE_START_X + MAZE_WIDTH * WALL_SIZE - WALL_SIZE / 2.0, _y);
            }
        }
    }

    std::vector<Point> bfs(Point start) {
        std::queue<Point> q;
        std::unordered_map<Point, Point> came_from;
        std::vector<Point> path;

        q.push(start);
        came_from[start] = start;

        while (!q.empty()) {
            Point current = q.front();
            q.pop();

            MazeCell* currentCell = &maze[current.y][current.x];
            if (exits.find(currentCell) != exits.end()) {
                return reconstructPath(start, current, came_from);
            }

            for (Point next : getNeighbors(current)) {
                if (came_from.find(next) == came_from.end()) {
                    q.push(next);
                    came_from[next] = current;
                }
            }
        }

        return path;
    }
    
    std::vector<Point> reconstructPath(const Point& start, const Point& end, 
                                       const std::unordered_map<Point, Point>& came_from) {
        std::vector<Point> path;
        Point at = end;
        
        while (at != start) {
            path.push_back(at);
            at = came_from.at(at);
        }
        
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        
        return path;
    }

    json generatePathData(const std::vector<Point>& path) {
        if (path.empty()) return json::array();
        
        json pathData = json::array();
        
        for (const Point& p : path) {
            pathData.push_back(getCellCoordinates(p));
        }

        // Add the point outside maze if we reached an exit
        Point exit = path.back();
        MazeCell exitCell = maze[exit.y][exit.x];
        
        if (exitCell.isBorderCell()) {
            pathData.push_back(getExitCoordinates(exit, exitCell));
        }

        return pathData;
    }
    
    json getCellCoordinates(const Point& p) {
        double realX = MAZE_START_X + p.x * MAZE_CELL_SIZE;
        double realY = MAZE_START_Y + p.y * MAZE_CELL_SIZE;
        return {{"x", realX}, {"y", realY}};
    }
    
    json getExitCoordinates(const Point& exit, const MazeCell& exitCell) {
        double realX = MAZE_START_X + exit.x * MAZE_CELL_SIZE;
        double realY = MAZE_START_Y + exit.y * MAZE_CELL_SIZE;
        
        if (exit.x == 0 && !exitCell.leftWall) {
            return {{"x", realX - MAZE_CELL_SIZE}, {"y", realY}};
        } else if (exit.x == MAZE_WIDTH - 1 && !exitCell.rightWall) {
            return {{"x", realX + MAZE_CELL_SIZE}, {"y", realY}};
        } else if (exit.y == 0 && !exitCell.topWall) {
            return {{"x", realX}, {"y", realY - MAZE_CELL_SIZE}};
        } else if (exit.y == MAZE_HEIGHT - 1 && !exitCell.bottomWall) {
            return {{"x", realX}, {"y", realY + MAZE_CELL_SIZE}};
        }
        
        // This should not happen if the exit cell is properly set
        return {{"x", realX}, {"y", realY}};
    }

    int findSet(MazeCell& cell) {
        for (auto const& x : sets) {
            if (x.second.find(&cell) != x.second.end()) {
                return x.first;
            }
        }
        return -1;
    }

    void mergeSets(int set1, int set2) {
        if (sets.find(set1) != sets.end() && sets.find(set2) != sets.end()) {
            sets[set1].insert(sets[set2].begin(), sets[set2].end());
            sets.erase(set2);
        }
    }

    void collectBorderCells(std::unordered_map<int, std::vector<MazeCell*>>& borderCells) {
        // Top and Bottom borders
        for (int col = 0; col < MAZE_WIDTH; ++col) {
            borderCells[0].push_back(&maze[0][col]);
            borderCells[1].push_back(&maze[MAZE_HEIGHT - 1][col]);
        }

        // Left and Right borders (excluding corners)
        for (int row = 1; row < MAZE_HEIGHT - 1; ++row) {
            borderCells[2].push_back(&maze[row][0]);
            borderCells[3].push_back(&maze[row][MAZE_WIDTH - 1]);
        }
    }

    std::vector<Point> getNeighbors(Point p) {
        std::vector<Point> neighbors;
        std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};

        for (auto &dir : directions) {
            int newX = p.x + dir.first;
            int newY = p.y + dir.second;

            if (isValidPosition(newX, newY) && isAccessible(p.x, p.y, newX, newY)) {
                neighbors.push_back(Point(newX, newY));
            }
        }

        return neighbors;
    }
    
    bool isValidPosition(int x, int y) {
        return x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT;
    }

    bool isAccessible(int x, int y, int newX, int newY) {
        MazeCell& cell = maze[y][x];
        MazeCell& neighbor = maze[newY][newX];

        if (x == newX) {  // Vertical movement
            return (y > newY) ? 
                   (!cell.topWall && !neighbor.bottomWall) : 
                   (!cell.bottomWall && !neighbor.topWall);
        } else {  // Horizontal movement
            return (x < newX) ? 
                   (!cell.rightWall && !neighbor.leftWall) : 
                   (!cell.leftWall && !neighbor.rightWall);
        }
    }
};

// Agent class that adds the controller process
class MazeGenerator : public Agent {
public:
    MazeGenerator(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
private:
    MazeGeneratorController c;
};

DECLARE_INTERFACE(MazeGenerator)

#endif