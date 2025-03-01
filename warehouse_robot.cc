#define SDL_MAIN_HANDLED
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <climits>
#include <string>

const int SCREEN_WIDTH = 800;
const int SCREEN_HEIGHT = 600;
const int GRID_SIZE = 40;
const int ROWS = SCREEN_HEIGHT / GRID_SIZE;
const int COLS = SCREEN_WIDTH / GRID_SIZE;
const int ROBOT_RADIUS = GRID_SIZE / 3;

// Global SDL variables
SDL_Window* window = nullptr;
SDL_Renderer* renderer = nullptr;
TTF_Font* font = nullptr; // For on-screen text

// Flag to toggle between BFS and A* pathfinding
bool useAStar = false;

// Structure definitions
struct Point {
    int x, y;
    Point(int a, int b) : x(a), y(b) {}
    Point() : x(0), y(0) {}
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
    
    int heuristic(const Point& goal) const {
        return std::abs(x - goal.x) + std::abs(y - goal.y);
    }
};

struct Robot {
    double x, y;  // Continuous position for smooth animation
    Point gridPos; // Position on the grid

    Robot(int gridX, int gridY)
        : x(gridX * GRID_SIZE + GRID_SIZE / 2),
          y(gridY * GRID_SIZE + GRID_SIZE / 2),
          gridPos(gridX, gridY) {}

    void moveToward(Point target, double speed) {
        double targetX = target.x * GRID_SIZE + GRID_SIZE / 2;
        double targetY = target.y * GRID_SIZE + GRID_SIZE / 2;

        double dx = targetX - x;
        double dy = targetY - y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist > speed) {
            x += speed * (dx / dist);
            y += speed * (dy / dist);
        } else {
            x = targetX;
            y = targetY;
            gridPos = target;
        }
    }
};

// Global simulation variables
std::vector<std::vector<int>> warehouseGrid(ROWS, std::vector<int>(COLS, 0)); // 0: Free, 1: Obstacle
Robot robot(0, 0);
Point destination(0, 0);
bool hasDestination = false;

// Function prototypes
bool initSDL();
void destroySDL();
void renderGrid();
void renderObstacles();
void renderRobot();
void renderDestination();
void renderPath(const std::vector<Point>& path);
void renderText(const std::string& message, int x, int y, SDL_Color color);
void renderInstructions();

bool isValidGridPosition(int x, int y);

// Pathfinding functions
std::vector<Point> findPath(Point start, Point end);
std::vector<Point> findPathA(Point start, Point end);

// Layout file I/O
void saveLayout(const std::string& filename);
void loadLayout(const std::string& filename);

int main() {
    if (!initSDL()) return 1;

    SDL_Event e;
    bool quit = false;
    std::vector<Point> path;
    size_t currentPathIndex = 0;

    while (!quit) {
        // Event handling
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
            // Mouse events
            else if (e.type == SDL_MOUSEBUTTONDOWN) {
                int gridX = e.button.x / GRID_SIZE;
                int gridY = e.button.y / GRID_SIZE;

                // Left click: set destination and calculate path
                if (e.button.button == SDL_BUTTON_LEFT) {
                    if (isValidGridPosition(gridX, gridY)) {
                        destination = {gridX, gridY};
                        hasDestination = true;
                        if (useAStar)
                            path = findPathA(robot.gridPos, destination);
                        else
                            path = findPath(robot.gridPos, destination);
                        currentPathIndex = 0;
                    }
                }
                // Right click: toggle obstacle & re-plan if needed
                else if (e.button.button == SDL_BUTTON_RIGHT) {
                    warehouseGrid[gridY][gridX] = 1 - warehouseGrid[gridY][gridX];
                    // If destination is active, re-calc path in case it’s affected
                    if (hasDestination) {
                        if (useAStar)
                            path = findPathA(robot.gridPos, destination);
                        else
                            path = findPath(robot.gridPos, destination);
                        currentPathIndex = 0;
                    }
                }
            }
            // Keyboard events
            else if (e.type == SDL_KEYDOWN) {
                // Reset destination and path
                if (e.key.keysym.sym == SDLK_r) {
                    hasDestination = false;
                    path.clear();
                }
                // Toggle pathfinding algorithm
                else if (e.key.keysym.sym == SDLK_t) {
                    useAStar = !useAStar;
                    // Re-calc path if destination exists
                    if (hasDestination) {
                        if (useAStar)
                            path = findPathA(robot.gridPos, destination);
                        else
                            path = findPath(robot.gridPos, destination);
                        currentPathIndex = 0;
                    }
                }
                // Save layout to file
                else if (e.key.keysym.sym == SDLK_s) {
                    saveLayout("warehouse_layout.txt");
                }
                // Load layout from file
                else if (e.key.keysym.sym == SDLK_l) {
                    loadLayout("warehouse_layout.txt");
                    // Recalculate path if necessary
                    if (hasDestination) {
                        if (useAStar)
                            path = findPathA(robot.gridPos, destination);
                        else
                            path = findPath(robot.gridPos, destination);
                        currentPathIndex = 0;
                    }
                }
            }
        }

        // Robot movement along path
        if (hasDestination && currentPathIndex < path.size()) {
            robot.moveToward(path[currentPathIndex], 2.0);
            if (robot.gridPos == path[currentPathIndex]) {
                ++currentPathIndex;
            }
        }

        // Rendering
        SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
        SDL_RenderClear(renderer);

        renderGrid();
        renderObstacles();
        renderPath(path);       // Draw the computed path
        renderRobot();
        renderDestination();
        renderInstructions();   // Draw instructions & current algorithm

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    destroySDL();
    return 0;
}

bool initSDL() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " 
                  << SDL_GetError() << std::endl;
        return false;
    }
    
    if(TTF_Init() == -1) {
        std::cerr << "SDL_ttf could not initialize! TTF_Error: " 
                  << TTF_GetError() << std::endl;
        return false;
    }
    
    window = SDL_CreateWindow("Automated Warehouse Robot", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 
                              SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Window could not be created! SDL_Error: " 
                  << SDL_GetError() << std::endl;
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created! SDL_Error: " 
                  << SDL_GetError() << std::endl;
        return false;
    }
    
    // Load a font (ensure the font file is in your working directory)
    font = TTF_OpenFont("ARIAL.TTF", 16);
    if (!font) {
        std::cerr << "Failed to load font! TTF_Error: " 
                  << TTF_GetError() << std::endl;
        return false;
    }
    
    return true;
}

void destroySDL() {
    TTF_CloseFont(font);
    font = nullptr;
    TTF_Quit();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void renderGrid() {
    SDL_SetRenderDrawColor(renderer, 50, 50, 50, 255);
    for (int i = 0; i <= ROWS; ++i) {
        SDL_RenderDrawLine(renderer, 0, i * GRID_SIZE, SCREEN_WIDTH, i * GRID_SIZE);
    }
    for (int i = 0; i <= COLS; ++i) {
        SDL_RenderDrawLine(renderer, i * GRID_SIZE, 0, i * GRID_SIZE, SCREEN_HEIGHT);
    }
}

void renderObstacles() {
    SDL_SetRenderDrawColor(renderer, 200, 50, 50, 255);
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            if (warehouseGrid[i][j] == 1) {
                SDL_Rect rect = {j * GRID_SIZE, i * GRID_SIZE, GRID_SIZE, GRID_SIZE};
                SDL_RenderFillRect(renderer, &rect);
            }
        }
    }
}

void renderRobot() {
    SDL_SetRenderDrawColor(renderer, 50, 200, 50, 255);
    SDL_Rect rect = {(int)(robot.x - ROBOT_RADIUS), (int)(robot.y - ROBOT_RADIUS), ROBOT_RADIUS * 2, ROBOT_RADIUS * 2};
    SDL_RenderFillRect(renderer, &rect);
}

void renderDestination() {
    if (hasDestination) {
        SDL_SetRenderDrawColor(renderer, 50, 50, 200, 255);
        SDL_Rect rect = {destination.x * GRID_SIZE + GRID_SIZE / 4, destination.y * GRID_SIZE + GRID_SIZE / 4, 
                         GRID_SIZE / 2, GRID_SIZE / 2};
        SDL_RenderFillRect(renderer, &rect);
    }
}

void renderPath(const std::vector<Point>& path) {
    if (path.empty()) return;
    // Draw small rectangles on each cell along the path
    SDL_SetRenderDrawColor(renderer, 255, 215, 0, 255); // Gold color
    for (const auto& p : path) {
        SDL_Rect rect = {p.x * GRID_SIZE + GRID_SIZE / 3, p.y * GRID_SIZE + GRID_SIZE / 3, 
                         GRID_SIZE / 3, GRID_SIZE / 3};
        SDL_RenderFillRect(renderer, &rect);
    }
}

void renderText(const std::string& message, int x, int y, SDL_Color color) {
    SDL_Surface* surface = TTF_RenderText_Blended(font, message.c_str(), color);
    if (!surface) {
        std::cerr << "Failed to render text surface! TTF_Error: " << TTF_GetError() << std::endl;
        return;
    }
    SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_Rect dstRect = {x, y, surface->w, surface->h};
    SDL_FreeSurface(surface);
    SDL_RenderCopy(renderer, texture, nullptr, &dstRect);
    SDL_DestroyTexture(texture);
}

void renderInstructions() {
    SDL_Color white = {255, 255, 255, 255};
    std::string algo = useAStar ? "A*" : "BFS";
    renderText("Left Click: Set Destination   Right Click: Toggle Obstacle", 10, 5, white);
    renderText("R: Reset   T: Toggle Algorithm   (Current: " + algo + ")", 10, 25, white);
    renderText("S: Save Layout   L: Load Layout", 10, 45, white);
}

bool isValidGridPosition(int x, int y) {
    return x >= 0 && x < COLS && y >= 0 && y < ROWS && warehouseGrid[y][x] == 0;
}

std::vector<Point> findPath(Point start, Point end) {
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<Point>> parents(ROWS, std::vector<Point>(COLS, {-1, -1}));

    std::queue<Point> queue;
    queue.push(start);
    visited[start.y][start.x] = true;

    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    while (!queue.empty()) {
        Point current = queue.front();
        queue.pop();

        if (current == end) {
            std::vector<Point> path;
            while (!(current == start)) {
                path.push_back(current);
                current = parents[current.y][current.x];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (isValidGridPosition(nx, ny) && !visited[ny][nx]) {
                queue.push({nx, ny});
                visited[ny][nx] = true;
                parents[ny][nx] = current;
            }
        }
    }

    return {}; // No path found
}

std::vector<Point> findPathA(Point start, Point end) {
    std::vector<std::vector<bool>> visited(ROWS, std::vector<bool>(COLS, false));
    std::vector<std::vector<Point>> parents(ROWS, std::vector<Point>(COLS, {-1, -1}));

    // Priority queue with custom comparator for f-cost
    auto comparator = [](const std::pair<Point, int>& a, const std::pair<Point, int>& b) {
        return a.second > b.second;
    };
    std::priority_queue<std::pair<Point, int>, std::vector<std::pair<Point, int>>, decltype(comparator)> openList(comparator);
    
    // gCost tracking
    std::vector<std::vector<int>> gCost(ROWS, std::vector<int>(COLS, INT_MAX));
    
    openList.push({start, 0});
    gCost[start.y][start.x] = 0;

    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    while (!openList.empty()) {
        Point current = openList.top().first;
        openList.pop();

        if (current == end) {
            std::vector<Point> path;
            while (!(current == start)) {
                path.push_back(current);
                current = parents[current.y][current.x];
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        visited[current.y][current.x] = true;

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (isValidGridPosition(nx, ny) && !visited[ny][nx]) {
                int newGCost = gCost[current.y][current.x] + 1;
                int hCost = std::abs(nx - end.x) + std::abs(ny - end.y);
                int fCost = newGCost + hCost;

                if (newGCost < gCost[ny][nx]) {
                    parents[ny][nx] = current;
                    gCost[ny][nx] = newGCost;
                    openList.push({{nx, ny}, fCost});
                }
            }
        }
    }
    return {}; // No path found
}

void saveLayout(const std::string& filename) {
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "Error saving layout to file!" << std::endl;
        return;
    }
    for (const auto& row : warehouseGrid) {
        for (int cell : row) {
            ofs << cell << " ";
        }
        ofs << "\n";
    }
    ofs.close();
    std::cout << "Layout saved to " << filename << std::endl;
}

void loadLayout(const std::string& filename) {
    std::ifstream ifs(filename);
    if (!ifs) {
        std::cerr << "Error loading layout from file!" << std::endl;
        return;
    }
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            ifs >> warehouseGrid[i][j];
        }
    }
    ifs.close();
    std::cout << "Layout loaded from " << filename << std::endl;
}
