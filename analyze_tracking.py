import csv
import math
import statistics
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple


Point = Tuple[float, float]


THIRTY_CM_RUNS = [
    ("Try 1", Path("/Users/lloocie/ARCS-ai/logs/flight_20260723_184854.csv")),
    ("Try 2", Path("/Users/lloocie/ARCS-ai/logs/flight_20260723_190428.csv")),
    ("Try 3", Path("/Users/lloocie/ARCS-ai/logs/flight_20260723_190753.csv")),
    ("Try 4", Path("/Users/lloocie/ARCS-ai/logs/flight_20260723_190939.csv")),
    ("Try 5", Path("/Users/lloocie/ARCS-ai/logs/flight_20260723_191102.csv")),
]

SIXTY_CM_RUNS = [
    ("Try 1", Path("/Users/lloocie/ARCS-ai/logs1/1.csv")),
    ("Try 2", Path("/Users/lloocie/ARCS-ai/logs1/2.csv")),
    ("Try 3", Path("/Users/lloocie/ARCS-ai/logs1/3.csv")),
    ("Try 4", Path("/Users/lloocie/ARCS-ai/logs1/4.csv")),
    ("Try 5", Path("/Users/lloocie/ARCS-ai/logs1/5.csv")),
]


def load_xy_points(path: Path) -> List[Point]:
    points: List[Point] = []
    with path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            points.append((float(row["x"]) * 100.0, float(row["y"]) * 100.0))
    return points


def summarize_runs(label: str, runs: Sequence[Tuple[str, Path]], target_cm: float) -> Tuple[float, float]:
    print(f"\n{label}")
    print("-" * len(label))

    outbound_errors_cm: List[float] = []
    return_errors_cm: List[float] = []
    valid_returns: List[Point] = []

    for run_name, path in runs:
        points = load_xy_points(path)
        if len(points) < 20:
            print(f"{run_name}: skipped, incomplete log ({len(points)} samples)")
            continue

        outbound = max(points, key=lambda point: point[0])
        returned = points[-1]

        outbound_error = math.hypot(outbound[0] - target_cm, outbound[1])
        return_error = math.hypot(returned[0], returned[1])

        outbound_errors_cm.append(outbound_error)
        return_errors_cm.append(return_error)
        valid_returns.append(returned)

        print(
            f"{run_name}: outbound=({outbound[0]:.1f}, {outbound[1]:.1f}) cm, "
            f"return=({returned[0]:.1f}, {returned[1]:.1f}) cm, "
            f"out err={outbound_error:.1f} cm, return err={return_error:.1f} cm"
        )

    mean_outbound = statistics.mean(outbound_errors_cm)
    mean_return = statistics.mean(return_errors_cm)
    sigma_t = statistics.pstdev([point[1] for point in valid_returns]) if len(valid_returns) > 1 else 0.0

    print(f"mean outbound endpoint error: {mean_outbound:.2f} cm")
    print(f"mean return endpoint error: {mean_return:.2f} cm")
    print(f"estimated sigma(T) from return y spread: {sigma_t:.2f} cm")

    return mean(max(mean_outbound, mean_return), mean_return), sigma_t


def mean(a: float, b: float) -> float:
    return (a + b) / 2.0


def main() -> None:
    e_track_30_cm, sigma_30_cm = summarize_runs("30 cm test", THIRTY_CM_RUNS, 30.0)
    e_track_60_cm, sigma_60_cm = summarize_runs("60 cm test", SIXTY_CM_RUNS, 60.0)

    e_track_m = max(e_track_30_cm, e_track_60_cm) / 100.0
    sigma_t_m = max(sigma_30_cm, sigma_60_cm) / 100.0
    r_eff_m = 0.15 + e_track_m + sigma_t_m

    print("\nRecommended planning values")
    print("---------------------------")
    print(f"e_track = {e_track_m:.3f} m")
    print(f"sigma(T) = {sigma_t_m:.3f} m")
    print(f"r_eff = 0.15 + sigma(T) + e_track = {r_eff_m:.3f} m")


if __name__ == "__main__":
    main()
