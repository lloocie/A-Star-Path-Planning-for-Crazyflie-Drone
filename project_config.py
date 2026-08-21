import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple


Point = Tuple[float, float]
Rectangle = Tuple[float, float, float, float]
PROJECT_DIR = Path(__file__).resolve().parent
LAYOUT_FILE = PROJECT_DIR / "course_layout.json"
LOGS_DIR = PROJECT_DIR / "logs"
CACHE_DIR = PROJECT_DIR / "cache"


@dataclass(frozen=True)
class CourseConfig:
    width_m: float = 2.0
    height_m: float = 2.0


@dataclass(frozen=True)
class PlanningConfig:
    grid_resolution_m: float = 0.05
    drone_radius_m: float = 0.06
    sigma_t_m: float = 0.04
    e_track_m: float = 0.08
    smoothing_samples_per_segment: int = 8

    @property
    def inflation_radius_m(self) -> float:
        return self.drone_radius_m + self.sigma_t_m + self.e_track_m


@dataclass(frozen=True)
class FlightConfig:
    uri: str = "radio://0/80/2M/E7E7E7E709"
    flight_height_m: float = 0.40
    flight_speed_mps: float = 0.15
    setpoint_rate_hz: int = 20
    log_period_ms: int = 100
    yaw_degrees: float = 0.0
    command_x_sign: float = 1.0
    command_y_sign: float = 1.0
    compare_x_sign: float = 1.0
    compare_y_sign: float = 1.0


# Replace these with your real measured obstacle positions.
# Rectangle format: (x_min, y_min, x_max, y_max)
DEFAULT_OBSTACLES: List[Rectangle] = [
    (0.80, 0.80, 1.20, 1.20),
]


DEFAULT_START: Point = (0.20, 0.20)
DEFAULT_GOAL: Point = (1.80, 1.80)


def save_layout(
    obstacles: List[Rectangle],
    start: Point,
    goal: Point,
    grid_resolution_m: float,
    path: Path = LAYOUT_FILE,
) -> None:
    payload = {
        "obstacles": [list(rect) for rect in obstacles],
        "start": list(start),
        "goal": list(goal),
        "grid_resolution_m": grid_resolution_m,
    }
    path.write_text(json.dumps(payload, indent=2))


def load_layout(path: Path = LAYOUT_FILE) -> Tuple[List[Rectangle], Point, Point, float]:
    if not path.exists():
        return list(DEFAULT_OBSTACLES), DEFAULT_START, DEFAULT_GOAL, 0.05

    payload = json.loads(path.read_text())
    obstacles = [tuple(rect) for rect in payload.get("obstacles", DEFAULT_OBSTACLES)]
    start = tuple(payload.get("start", DEFAULT_START))
    goal = tuple(payload.get("goal", DEFAULT_GOAL))
    grid_resolution_m = float(payload.get("grid_resolution_m", 0.05))
    return obstacles, start, goal, grid_resolution_m
