# Automated Warehouse Robot Simulation

This project is a C++ based simulation using SDL2 library to visualize a robot navigating a warehouse grid. Users can interactively define obstacles on the grid, set a destination for the robot, and observe the robot find a path using either Breadth-First Search (BFS) or A* pathfinding algorithm. The simulation visually represents the grid, obstacles, robot, calculated path, and provides real-time instructions. It also supports saving and loading warehouse layouts to and from files.

## Getting Started

### Prerequisites
Ensure you have the following installed:
- C++ Compiler (e.g., g++)
-  [SDL2](https://www.libsdl.org/)
-  [SDL2_ttf](https://www.libsdl.org/projects/SDL_ttf/)


## Project Structure

-  `warehourse_robot.cc`: Main C++ source file containing the simulation logic and rendering.
-  `Makefile`: Build script for compiling the project.
-  `src/`: Source code directory (currently not utilized for separate source and headers, but included in template for potential expansion).
    *   `include/`:  Header files

### Installation
1. Clone the repository:
    ```bash
    git clone git@github.com:Luke23-45/Visualization-of-path-finding-algorithm-with-SDL2.git
    ```
## Building

This project uses `make` for building. To build the project, run the following command in your terminal:

1. Navigate to the project directory:
    ```bash
    cd Visualization-of-path-finding-algorithm-with-SDL2
    ```
3. Compile the code:
    ```bash
     make
    ```
4. Run the executable:
    ```bash
    ./main
    ```

5. In window (if Makefile produces `main.exe` for windows, otherwise use the executable name from Makefile, likely `colorfull_ball.exe` or similar):
    ```bash
    main.exe
    ```
6. To clean up the build artifacts
    ```bash
     make clean
    ```

## Features
- **Interactive Obstacle Placement:**  Define warehouse obstacles in real-time using right-click on the grid.
- **Destination Setting:** Set a target destination for the robot by left-clicking on any valid grid cell.
- **Pathfinding Algorithms:** Implements both Breadth-First Search (BFS) and A* algorithm for path planning.
- **Algorithm Toggling:** Dynamically switch between BFS and A* pathfinding using the `T` key to compare pathfinding strategies.
- **Path Visualization:**  Visually displays the calculated path on the grid for easy understanding of the robot's movement.
- **Real-time Robot Movement:** Simulates the robot moving smoothly along the calculated path towards the destination.
- **Warehouse Layout Saving/Loading:** Persist and reuse warehouse layouts by saving the current obstacle configuration to a file and loading it later using `S` and `L` keys respectively.
- **User-Friendly Instructions:** On-screen text provides clear instructions on how to interact with the simulation and use different features.
- **Grid-based Visualization:** Clear grid representation of the warehouse environment, robot, obstacles, and destination using SDL2 graphics.

## Key Controls

| Action                | Key / Mouse Button | Description                                           |
| --------------------- | ------------------ | ----------------------------------------------------- |
| Exit simulation       | Window Close Button| Close the simulation window to exit.                  |
| Set Destination       | Left Click         | Click on any empty grid cell to set the robot's target destination. |
| Toggle Obstacle       | Right Click        | Click on any grid cell to toggle an obstacle. Right-click again to remove it. |
| Reset Path/Destination| `R` key            | Clears the current path and deselects the destination. |
| Toggle Algorithm      | `T` key            | Switches between Breadth-First Search (BFS) and A* pathfinding algorithms. |
| Save Layout           | `S` key            | Saves the current warehouse layout (obstacles) to `warehouse_layout.txt`. |
| Load Layout           | `L` key            | Loads a warehouse layout from `warehouse_layout.txt`.  |

## Code Structure
The project is primarily contained within the `colorfull_ball.cc` file, which includes all the source code for the simulation.

- **`warehourse_robot.cc`**: This file serves as the main source file and encompasses:
    - **SDL Initialization and Setup**: Functions `initSDL()` and `destroySDL()` handle the initialization and cleanup of SDL2, including window, renderer, and font systems.
    - **Rendering Functions**:  Functions like `renderGrid()`, `renderObstacles()`, `renderRobot()`, `renderDestination()`, `renderPath()`, `renderText()`, and `renderInstructions()` are responsible for drawing different elements of the simulation on the screen using SDL2 rendering API.
    - **Pathfinding Algorithms**:  Implements both Breadth-First Search (BFS) in `findPath()` and A* search algorithm in `findPathA()` to calculate paths for the robot.
    - **Robot and Point Structures**: Defines `Point` and `Robot` structs to represent grid locations and the robot object with its position and movement logic.
    - **Grid and Simulation Logic**: Manages the `warehouseGrid` which represents the warehouse environment, handles user input events (mouse clicks, key presses), robot movement along the path, and simulation state.
    - **File I/O**: Functions `saveLayout()` and `loadLayout()` handle saving and loading the obstacle layout to and from text files.
    - **`main()` Function**: The entry point of the program, containing the main simulation loop, event handling, game logic updates, and rendering calls.



## Demo Video
Check out the project demo video on YouTube: https://youtu.be/HDTzYFRzg50
## License

This project is licensed under the MIT License. Feel free to use, modify, and distribute the code.

## Acknowledgements

- SDL2 for graphics rendering.
