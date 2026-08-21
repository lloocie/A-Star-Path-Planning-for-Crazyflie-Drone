# Final Project: A* Planning Through a Physical Course

This folder contains a complete final-project implementation for planning and flying a Crazyflie through a 2 x 2 m course with obstacle inflation based on measured tracking error.

## What is included

- `project_config.py`: course, drone, and inflation settings
- `planner.py`: occupancy grid, obstacle inflation, A* search, path smoothing, and plotting
- `flight_runner.py`: optional Crazyflie execution of the planned path
- `analyze_tracking.py`: computes `e_track`, repeatability, and a recommended `r_eff` from your log folders

## Project formula

The implementation follows:

`r_eff = 0.15 + sigma(T) + e_track`

Where:

- `0.15 m` is the drone radius term from the project handout
- `sigma(T)` is repeatability/drift spread from repeated trials
- `e_track` is straight-line control error

## Recommended values from your logs

From the straight-line experiments already analyzed:

- `e_track ~= 0.08 m`
- `sigma(T) ~= 0.04 m`
- recommended conservative `r_eff ~= 0.27 m`

These values are set as defaults in `project_config.py`, and you can recompute them with:

```bash
python3 /Users/lloocie/ARCS-ai/final_project/analyze_tracking.py
```

## Quick start

1. Edit the obstacle list in `project_config.py` using your tape-measured obstacle positions.
2. Run the planner on the 5 cm grid:

```bash
python3 /Users/lloocie/ARCS-ai/final_project/planner.py
```

3. If a valid path exists, optionally fly it:

```bash
python3 /Users/lloocie/ARCS-ai/final_project/flight_runner.py
```

## Notes

- The planner is configured to use only `delta = 0.05 m` (5 cm grid cells).
- The path is planned on the grid, then smoothed to reduce sharp corners.
- If the fine grid produces a path that hugs the inflation boundary too closely, the coarser grid can sometimes be more flyable in practice because it produces a simpler path with larger effective clearance.
