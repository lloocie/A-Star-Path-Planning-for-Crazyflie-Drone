import csv
import math
import sys
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np

from planner import plan_path
from project_config import FlightConfig, PlanningConfig, load_layout


Point = Tuple[float, float]


def load_logged_points(path: Path) -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    with path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            points.append((float(row["x"]), float(row["y"]), float(row["z"])))
    return points


def drone_point_to_map_frame(point: Point, config: FlightConfig) -> Point:
    return (
        point[0] / config.compare_x_sign,
        point[1] / config.compare_y_sign,
    )


def filter_flight_window(points: List[Tuple[float, float, float]]) -> List[Point]:
    filtered = [(x, y) for x, y, z in points if z > 0.05]
    if len(filtered) < 2:
        filtered = [(x, y) for x, y, _ in points]
    return filtered


def cumulative_lengths(points: List[Point]) -> np.ndarray:
    values = [0.0]
    for index in range(1, len(points)):
        values.append(values[-1] + math.dist(points[index - 1], points[index]))
    return np.array(values, dtype=float)


def resample_polyline(points: List[Point], sample_count: int) -> List[Point]:
    if len(points) <= 1:
        return points

    lengths = cumulative_lengths(points)
    total = lengths[-1]
    if total <= 1e-9:
        return [points[0]] * sample_count

    targets = np.linspace(0.0, total, sample_count)
    xs = np.array([point[0] for point in points], dtype=float)
    ys = np.array([point[1] for point in points], dtype=float)
    resampled_x = np.interp(targets, lengths, xs)
    resampled_y = np.interp(targets, lengths, ys)
    return list(zip(resampled_x.tolist(), resampled_y.tolist()))


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python compare_flight_to_plan.py /path/to/log.csv")

    log_path = Path(sys.argv[1]).expanduser().resolve()
    if not log_path.exists():
        raise SystemExit(f"Log file not found: {log_path}")

    config = FlightConfig()
    obstacles, start, goal, grid_resolution_m = load_layout()
    plan = plan_path(
        start_m=start,
        goal_m=goal,
        obstacles=obstacles,
        planning=PlanningConfig(grid_resolution_m=grid_resolution_m),
    )

    planned_relative_map = [(point[0] - start[0], point[1] - start[1]) for point in plan.smoothed_points_m]

    executed_drone_with_z = load_logged_points(log_path)
    if not executed_drone_with_z:
        raise SystemExit("The log file is empty.")

    executed_drone = filter_flight_window(executed_drone_with_z)
    start_drone = executed_drone[0]
    executed_relative_drone = [(point[0] - start_drone[0], point[1] - start_drone[1]) for point in executed_drone]
    executed_relative_map = [drone_point_to_map_frame(point, config) for point in executed_relative_drone]

    planned_x = [point[0] for point in planned_relative_map]
    planned_y = [point[1] for point in planned_relative_map]
    exec_x = [point[0] for point in executed_relative_map]
    exec_y = [point[1] for point in executed_relative_map]

    sample_count = min(40, max(10, min(len(planned_relative_map), len(executed_relative_map))))
    planned_samples = resample_polyline(planned_relative_map, sample_count)
    executed_samples = resample_polyline(executed_relative_map, sample_count)

    plt.figure(figsize=(8, 8))
    plt.plot(planned_x, planned_y, "k--", linewidth=2, label="Planned")
    plt.plot(exec_x, exec_y, "b.-", linewidth=2, label="Executed")
    plt.scatter([planned_x[0]], [planned_y[0]], color="green", s=100, label="Start")
    plt.scatter([planned_x[-1]], [planned_y[-1]], color="red", marker="x", s=120, label="Goal")

    for (px, py), (ex, ey) in zip(planned_samples, executed_samples):
        plt.plot([px, ex], [py, ey], color="red", linewidth=0.8, alpha=0.7)

    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title("Planned vs. Executed Trajectory")
    plt.axis("equal")
    plt.grid(True, linewidth=0.3)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
