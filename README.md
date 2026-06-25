# VsimFGO

A learning derivative project based on [gnssFGO](https://github.com/hz658832/gnssFGO)

## Project Description

This project is a learning-oriented extension of the gnssFGO algorithm. Building upon the original multi-constellation GNSS factor graph optimization implementation, this project explores integrating visual information into the positioning framework.

## Project Goals

- Understand and learn factor graph optimization for GNSS positioning
- Add **image recognition factors** to the factor graph
- Apply **Gaussian interpolation** to image factors
- Explore GNSS + vision sensor fusion positioning

## Relationship with Original Project

| Project | Description |
|---------|-------------|
| [gnssFGO](https://github.com/hz658832/gnssFGO) | Original algorithm code for multi-constellation GNSS factor graph optimization |
| **VsimFGO** | Derivative learning project extending visual factor support |

## How to start
**1. Build the container**
```bash
git clone https://github.com/yukinosome/VsimFGO.git
cd gnssFGO/docker
docker build -t haomingac/gnssfgo:latest
docker compose up -d
```

**2. How to start the container:**
```bash
docker start gnssfgo
```

You can access the container interactively:
```bash
docker exec -it gnssfgo bash
```
**3. Start the program**
```bash
cd workspace/fgo_ws
colcon build
source install/setup.bash
ros2 launch online_fgo aachen_lc_all.launch.py