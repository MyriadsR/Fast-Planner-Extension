# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

T-PRM (Temporal Probabilistic Roadmap) is a C++ library for path planning in dynamic environments. It extends the traditional PRM algorithm with temporal awareness to handle moving obstacles. The library is designed for robotics applications, particularly for autonomous navigation in cluttered, dynamic scenes.

## Build System & Common Commands

### Basic Build
```bash
mkdir build
cd build
cmake ..
make
```

### Build with Benchmarking
```bash
mkdir build
cd build
cmake -DBUILD_BENCHMARKING=ON ..
make
```

### Dependencies
- **Core**: Eigen3 (auto-downloaded if not found), OpenMP
- **Benchmarking**: OMPL library (`sudo apt install libompl-dev` or `sudo apt install ros-noetic-ompl`)

### Examples
Run basic examples:
```bash
./examples/static_avoidance
./examples/dynamic_avoidance
```

### Benchmarking
Run performance studies:
```bash
./benchmarking/study_dynamic_obstacles
./benchmarking/study_static_obstacles
./benchmarking/narrow_gap
```

## Code Architecture

### Core Components

**Main Library** (`include/tprm/`, `src/`):
- `TemporalPRM`: Main planning class that orchestrates the entire algorithm
- `TemporalGraph`: Time-augmented graph structure for representing the temporal roadmap
- `HolonomicRobot`: Robot model with configurable movement speed (default 0.5 m/s)
- `ObstacleBase`/`ObstacleImpl`: Obstacle representation system supporting both static and dynamic obstacles

### Key Design Patterns

**Temporal Extension**: The library extends traditional PRM by adding time as a dimension. Each node in the roadmap has both spatial coordinates and temporal information, enabling collision-free planning around moving obstacles.

**Obstacle System**: Uses inheritance hierarchy with `ObstacleBase` as interface and concrete implementations like `DynamicSphereObstacle`. Obstacles can have velocity vectors for predictable motion.

**Graph Structure**: `TemporalGraph` maintains the augmented graph where edges are validated for collision-free motion over time intervals, not just spatial connectivity.

### Library Usage Pattern

1. Set robot speed: `tprm::HolonomicRobot::movement_speed = 0.1`
2. Create planner: `tprm::TemporalPRM tprm`
3. Add obstacles: `tprm.addDynamicObstacle(...)`
4. Build roadmap: `tprm.placeSamples(150); tprm.buildPRM(0.25)`
5. Query paths: `tprm.getShortestPath(start, goal, start_time)`

### Project Structure

- `include/tprm/`: Public headers for the library API
- `src/`: Implementation files (.cpp)
- `examples/`: Demonstration programs showing basic usage
- `benchmarking/`: Performance comparison framework against OMPL planners
- `cmake/`: CMake modules for dependency management
- `docs/`: Documentation (refer to https://vis4rob-lab.github.io/t_prm)

### Testing & Validation

The library uses the benchmarking framework to validate against OMPL's state-of-the-art planners. Performance metrics include path quality, planning time, and success rates in various dynamic scenarios.