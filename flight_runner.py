import csv
import os
import time
from datetime import datetime
from typing import Iterable, List, Tuple

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from planner import plan_path
from project_config import CACHE_DIR, FlightConfig, LOGS_DIR, PlanningConfig, load_layout


Point = Tuple[float, float]


writer = None
log_file = None


def position_callback(timestamp, data, logconf) -> None:
    global writer, log_file
    if writer is None or log_file is None:
        return

    writer.writerow(
        [
            timestamp,
            data["stateEstimate.x"],
            data["stateEstimate.y"],
            data["stateEstimate.z"],
        ]
    )
    log_file.flush()


def reset_estimator(scf: SyncCrazyflie) -> None:
    print("Resetting estimator...")
    scf.cf.param.set_value("kalman.resetEstimation", "1")
    time.sleep(0.1)
    scf.cf.param.set_value("kalman.resetEstimation", "0")
    time.sleep(2.0)
    print("Estimator reset complete.")


def send_setpoint_for_time(scf: SyncCrazyflie, config: FlightConfig, x: float, y: float, z: float, duration: float) -> None:
    period = 1.0 / config.setpoint_rate_hz
    end_time = time.monotonic() + duration
    while time.monotonic() < end_time:
        scf.cf.commander.send_position_setpoint(x, y, z, config.yaw_degrees)
        time.sleep(period)


def takeoff(scf: SyncCrazyflie, config: FlightConfig) -> None:
    duration = 3.0
    steps = int(duration * config.setpoint_rate_hz)
    for step in range(steps + 1):
        z = config.flight_height_m * (step / steps)
        scf.cf.commander.send_position_setpoint(0.0, 0.0, z, config.yaw_degrees)
        time.sleep(1.0 / config.setpoint_rate_hz)
    send_setpoint_for_time(scf, config, 0.0, 0.0, config.flight_height_m, 1.0)


def fly_segment(scf: SyncCrazyflie, config: FlightConfig, start: Point, goal: Point) -> None:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    distance = (dx * dx + dy * dy) ** 0.5
    if distance < 1e-6:
        return

    duration = distance / config.flight_speed_mps
    steps = max(1, int(duration * config.setpoint_rate_hz))
    print(f"Flying to ({goal[0]:.2f}, {goal[1]:.2f}) m")

    for step in range(steps + 1):
        progress = step / steps
        x = start[0] + progress * dx
        y = start[1] + progress * dy
        scf.cf.commander.send_position_setpoint(x, y, config.flight_height_m, config.yaw_degrees)
        time.sleep(1.0 / config.setpoint_rate_hz)

    send_setpoint_for_time(scf, config, goal[0], goal[1], config.flight_height_m, 0.8)


def land(scf: SyncCrazyflie, config: FlightConfig, final_xy: Point) -> None:
    duration = 3.0
    steps = int(duration * config.setpoint_rate_hz)
    for step in range(steps, -1, -1):
        z = config.flight_height_m * (step / steps)
        scf.cf.commander.send_position_setpoint(final_xy[0], final_xy[1], z, config.yaw_degrees)
        time.sleep(1.0 / config.setpoint_rate_hz)
    scf.cf.commander.send_stop_setpoint()
    time.sleep(0.1)
    scf.cf.commander.send_notify_setpoint_stop()


def fly_path(relative_points: List[Point], config: FlightConfig) -> None:
    global writer, log_file

    os.makedirs(LOGS_DIR, exist_ok=True)
    filename = os.path.join(
        str(LOGS_DIR),
        datetime.now().strftime("planned_path_%Y%m%d_%H%M%S.csv"),
    )
    log_file = open(filename, "w", newline="")
    writer = csv.writer(log_file)
    writer.writerow(["timestamp", "x", "y", "z"])

    cflib.crtp.init_drivers()
    os.makedirs(CACHE_DIR, exist_ok=True)
    with SyncCrazyflie(config.uri, cf=Crazyflie(rw_cache=str(CACHE_DIR))) as scf:
        print("Connected to Crazyflie.")
        reset_estimator(scf)

        lg_state = LogConfig(name="State", period_in_ms=config.log_period_ms)
        lg_state.add_variable("stateEstimate.x", "float")
        lg_state.add_variable("stateEstimate.y", "float")
        lg_state.add_variable("stateEstimate.z", "float")
        scf.cf.log.add_config(lg_state)
        lg_state.data_received_cb.add_callback(position_callback)
        lg_state.start()

        try:
            takeoff(scf, config)
            for index in range(len(relative_points) - 1):
                fly_segment(scf, config, relative_points[index], relative_points[index + 1])
            land(scf, config, relative_points[-1])
            print("Flight complete.")
            print(f"Log saved to: {filename}")
        finally:
            lg_state.stop()
            if log_file is not None:
                log_file.close()
            writer = None
            log_file = None


def map_point_to_drone_frame(point: Point, config: FlightConfig) -> Point:
    return (
        config.command_x_sign * point[0],
        config.command_y_sign * point[1],
    )


def main() -> None:
    config = FlightConfig()
    obstacles, start, goal, grid_resolution_m = load_layout()
    plan = plan_path(
        start_m=start,
        goal_m=goal,
        obstacles=obstacles,
        planning=PlanningConfig(grid_resolution_m=grid_resolution_m),
    )
    relative_map_points = [(point[0] - start[0], point[1] - start[1]) for point in plan.smoothed_points_m]
    relative_points = [map_point_to_drone_frame(point, config) for point in relative_map_points]

    print("Planned relative path in map frame:")
    for index, point in enumerate(relative_map_points):
        print(f"  {index}: x={point[0]:.2f} m, y={point[1]:.2f} m")

    print("\nCommanded relative path in drone frame:")
    for index, point in enumerate(relative_points):
        print(f"  {index}: x={point[0]:.2f} m, y={point[1]:.2f} m")

    print(
        f"\nCommand transform: command_x_sign={config.command_x_sign:.0f}, "
        f"command_y_sign={config.command_y_sign:.0f}"
    )

    confirmation = input(f"Type FLY to execute the {grid_resolution_m:.2f} m-grid smoothed path: ").strip().upper()
    if confirmation != "FLY":
        print("Flight cancelled.")
        return

    fly_path(relative_points, config)


if __name__ == "__main__":
    main()
