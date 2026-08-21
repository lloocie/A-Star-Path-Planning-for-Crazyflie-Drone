import heapq
import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle as PatchRectangle

from project_config import (
    CourseConfig,
    DEFAULT_GOAL,
    DEFAULT_OBSTACLES,
    DEFAULT_START,
    PlanningConfig,
    Point,
    Rectangle,
    load_layout,
)


GridIndex = Tuple[int, int]


@dataclass
class PlannedPath:
    raw_points_m: List[Point]
    smoothed_points_m: List[Point]
    grid_resolution_m: float
    inflation_radius_m: float
    raw_length_m: float
    smoothed_length_m: float


def metres_to_grid(point: Point, resolution_m: float) -> GridIndex:
    x, y = point
    return int(y / resolution_m), int(x / resolution_m)


def grid_to_metres(cell: GridIndex, resolution_m: float) -> Point:
    row, col = cell
    return (col + 0.5) * resolution_m, (row + 0.5) * resolution_m


def build_grid(course: CourseConfig, planning: PlanningConfig, obstacles: Sequence[Rectangle]) -> np.ndarray:
    rows = int(math.ceil(course.height_m / planning.grid_resolution_m))
    cols = int(math.ceil(course.width_m / planning.grid_resolution_m))
    grid = np.zeros((rows, cols), dtype=np.uint8)

    inflated_obstacles = [inflate_rectangle(rect, planning.inflation_radius_m, course) for rect in obstacles]

    for row in range(rows):
        for col in range(cols):
            x, y = grid_to_metres((row, col), planning.grid_resolution_m)
            if any(point_inside_rectangle((x, y), rect) for rect in inflated_obstacles):
                grid[row, col] = 1

    return grid


def inflate_rectangle(rect: Rectangle, radius_m: float, course: CourseConfig) -> Rectangle:
    x_min, y_min, x_max, y_max = rect
    return (
        max(0.0, x_min - radius_m),
        max(0.0, y_min - radius_m),
        min(course.width_m, x_max + radius_m),
        min(course.height_m, y_max + radius_m),
    )


def point_inside_rectangle(point: Point, rect: Rectangle) -> bool:
    x, y = point
    x_min, y_min, x_max, y_max = rect
    return x_min <= x <= x_max and y_min <= y <= y_max


def astar(grid: np.ndarray, start: GridIndex, goal: GridIndex) -> List[GridIndex]:
    def heuristic(a: GridIndex, b: GridIndex) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    neighbors = [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ]

    open_heap: List[Tuple[float, GridIndex]] = [(0.0, start)]
    came_from: Dict[GridIndex, GridIndex] = {}
    g_score: Dict[GridIndex, float] = {start: 0.0}

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            return reconstruct_path(came_from, current)

        for dr, dc, step_cost in neighbors:
            nxt = (current[0] + dr, current[1] + dc)
            if not is_free_cell(grid, nxt):
                continue

            if dr != 0 and dc != 0:
                side_a = (current[0] + dr, current[1])
                side_b = (current[0], current[1] + dc)
                if not is_free_cell(grid, side_a) or not is_free_cell(grid, side_b):
                    continue

            tentative = g_score[current] + step_cost
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f_score = tentative + heuristic(nxt, goal)
                heapq.heappush(open_heap, (f_score, nxt))

    raise RuntimeError("No flyable A* path found with the current inflation radius and grid resolution.")


def is_free_cell(grid: np.ndarray, cell: GridIndex) -> bool:
    row, col = cell
    return 0 <= row < grid.shape[0] and 0 <= col < grid.shape[1] and grid[row, col] == 0


def reconstruct_path(came_from: Dict[GridIndex, GridIndex], current: GridIndex) -> List[GridIndex]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def simplify_path(points: Sequence[Point]) -> List[Point]:
    if len(points) <= 2:
        return list(points)

    simplified = [points[0]]
    for index in range(1, len(points) - 1):
        prev_point = simplified[-1]
        current_point = points[index]
        next_point = points[index + 1]

        v1 = (current_point[0] - prev_point[0], current_point[1] - prev_point[1])
        v2 = (next_point[0] - current_point[0], next_point[1] - current_point[1])
        cross = v1[0] * v2[1] - v1[1] * v2[0]

        if abs(cross) > 1e-9:
            simplified.append(current_point)

    simplified.append(points[-1])
    return simplified


def smooth_polyline(points: Sequence[Point], samples_per_segment: int) -> List[Point]:
    if len(points) < 3:
        return list(points)

    smoothed = [points[0]]
    for index in range(len(points) - 2):
        p0 = points[index]
        p1 = points[index + 1]
        p2 = points[index + 2]

        midpoint_a = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        midpoint_b = ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0)

        for sample_index in range(1, samples_per_segment + 1):
            t = sample_index / samples_per_segment
            x = (1 - t) ** 2 * midpoint_a[0] + 2 * (1 - t) * t * p1[0] + t**2 * midpoint_b[0]
            y = (1 - t) ** 2 * midpoint_a[1] + 2 * (1 - t) * t * p1[1] + t**2 * midpoint_b[1]
            smoothed.append((x, y))

    smoothed.append(points[-1])
    return smoothed


def polyline_length(points: Sequence[Point]) -> float:
    return sum(math.dist(points[index], points[index + 1]) for index in range(len(points) - 1))


def plan_path(
    start_m: Point = DEFAULT_START,
    goal_m: Point = DEFAULT_GOAL,
    obstacles: Sequence[Rectangle] = DEFAULT_OBSTACLES,
    planning: PlanningConfig = PlanningConfig(),
    course: CourseConfig = CourseConfig(),
) -> PlannedPath:
    grid = build_grid(course, planning, obstacles)
    start_cell = metres_to_grid(start_m, planning.grid_resolution_m)
    goal_cell = metres_to_grid(goal_m, planning.grid_resolution_m)

    if not is_free_cell(grid, start_cell):
        raise ValueError("Start lies inside an inflated obstacle.")
    if not is_free_cell(grid, goal_cell):
        raise ValueError("Goal lies inside an inflated obstacle.")

    grid_path = astar(grid, start_cell, goal_cell)
    raw_points = [grid_to_metres(cell, planning.grid_resolution_m) for cell in grid_path]
    simplified = simplify_path(raw_points)
    smoothed = smooth_polyline(simplified, planning.smoothing_samples_per_segment)

    return PlannedPath(
        raw_points_m=simplified,
        smoothed_points_m=smoothed,
        grid_resolution_m=planning.grid_resolution_m,
        inflation_radius_m=planning.inflation_radius_m,
        raw_length_m=polyline_length(simplified),
        smoothed_length_m=polyline_length(smoothed),
    )


def plot_plan(
    path: PlannedPath,
    obstacles: Sequence[Rectangle],
    planning: PlanningConfig,
    course: CourseConfig,
    start_m: Point,
    goal_m: Point,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, course.width_m)
    ax.set_ylim(0, course.height_m)
    ax.set_aspect("equal")
    ax.set_xticks(np.arange(0.0, course.width_m + 1e-6, planning.grid_resolution_m))
    ax.set_yticks(np.arange(0.0, course.height_m + 1e-6, planning.grid_resolution_m))
    ax.grid(True, linewidth=0.25)

    for obstacle in obstacles:
        x_min, y_min, x_max, y_max = obstacle
        raw_patch = PatchRectangle((x_min, y_min), x_max - x_min, y_max - y_min, alpha=0.5, label="Obstacle")
        ax.add_patch(raw_patch)

        inflated = inflate_rectangle(obstacle, planning.inflation_radius_m, course)
        ix0, iy0, ix1, iy1 = inflated
        inflated_patch = PatchRectangle(
            (ix0, iy0),
            ix1 - ix0,
            iy1 - iy0,
            fill=False,
            linestyle="--",
            linewidth=2,
            label="Inflated obstacle",
        )
        ax.add_patch(inflated_patch)

    raw_x = [p[0] for p in path.raw_points_m]
    raw_y = [p[1] for p in path.raw_points_m]
    smooth_x = [p[0] for p in path.smoothed_points_m]
    smooth_y = [p[1] for p in path.smoothed_points_m]

    ax.plot(raw_x, raw_y, marker="o", linewidth=2, label="A* path")
    ax.plot(smooth_x, smooth_y, linewidth=2, label="Smoothed path")
    ax.scatter([start_m[0]], [start_m[1]], s=120, marker="o", label="Start")
    ax.scatter([goal_m[0]], [goal_m[1]], s=120, marker="x", label="Goal")

    ax.set_title(
        f"A* path, delta={planning.grid_resolution_m:.2f} m, r_eff={planning.inflation_radius_m:.2f} m"
    )
    ax.legend()
    plt.show()


def main() -> None:
    obstacles, start, goal, grid_resolution_m = load_layout()

    planning = PlanningConfig(grid_resolution_m=grid_resolution_m)
    plan = plan_path(start_m=start, goal_m=goal, obstacles=obstacles, planning=planning)
    print(f"{grid_resolution_m:.2f} m grid raw path length: {plan.raw_length_m:.2f} m")
    print(f"{grid_resolution_m:.2f} m grid smoothed path length: {plan.smoothed_length_m:.2f} m")
    plot_plan(plan, obstacles, planning, CourseConfig(), start, goal)


if __name__ == "__main__":
    main()
