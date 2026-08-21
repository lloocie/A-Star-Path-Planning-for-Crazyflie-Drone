from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle as PatchRectangle
from matplotlib.widgets import RadioButtons

from project_config import CourseConfig, LAYOUT_FILE, PlanningConfig, Point, Rectangle, load_layout, save_layout


class CourseEditor:
    def __init__(self) -> None:
        self.course = CourseConfig()
        obstacles, start, goal, grid_resolution_m = load_layout()
        self.planning = PlanningConfig(grid_resolution_m=grid_resolution_m)
        self.obstacles: List[Rectangle] = list(obstacles)
        self.start: Optional[Point] = start
        self.goal: Optional[Point] = goal
        self.drag_start: Optional[Point] = None
        self.preview_patch: Optional[PatchRectangle] = None
        self.mode = "obstacle"

        self.fig, self.ax = plt.subplots(figsize=(10.5, 9))
        plt.subplots_adjust(left=0.08, right=0.80)
        button_ax = self.fig.add_axes([0.83, 0.70, 0.14, 0.14])
        self.delta_selector = RadioButtons(
            button_ax,
            ("0.05 m", "0.15 m"),
            active=0 if abs(self.planning.grid_resolution_m - 0.05) < 1e-9 else 1,
        )
        self.delta_selector.on_clicked(self.on_delta_change)
        self.fig.canvas.mpl_connect("button_press_event", self.on_press)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self.on_move)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    def snap(self, value: float) -> float:
        resolution = self.planning.grid_resolution_m
        snapped = round(value / resolution) * resolution
        return min(max(snapped, 0.0), self.course.width_m)

    def snap_point(self, x: float, y: float) -> Point:
        return self.snap(x), self.snap(y)

    def normalize_rect(self, p0: Point, p1: Point) -> Optional[Rectangle]:
        x0, y0 = p0
        x1, y1 = p1
        x_min, x_max = sorted((x0, x1))
        y_min, y_max = sorted((y0, y1))
        if x_max - x_min < self.planning.grid_resolution_m or y_max - y_min < self.planning.grid_resolution_m:
            return None
        return x_min, y_min, x_max, y_max

    def draw(self) -> None:
        self.ax.clear()
        self.ax.set_xlim(0, self.course.width_m)
        self.ax.set_ylim(0, self.course.height_m)
        self.ax.set_aspect("equal")
        ticks = [index * self.planning.grid_resolution_m for index in range(int(self.course.width_m / self.planning.grid_resolution_m) + 1)]
        self.ax.set_xticks(ticks)
        self.ax.set_yticks(ticks)
        self.ax.grid(True, linewidth=0.25)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_title(
            "2 x 2 m course editor\n"
            "Choose delta first | Drag obstacle | a: start | b: goal | d: delete last | enter: save"
        )

        self.fig.text(
            0.82,
            0.62,
            "Delta controls cell size.\n"
            "0.05 m = finer map.\n"
            "0.15 m = coarser map.",
            fontsize=10,
            va="top",
        )

        for index, rect in enumerate(self.obstacles, start=1):
            x_min, y_min, x_max, y_max = rect
            patch = PatchRectangle((x_min, y_min), x_max - x_min, y_max - y_min, alpha=0.5, color="tab:red")
            self.ax.add_patch(patch)
            self.ax.text(x_min, y_max, str(index), fontsize=8, va="bottom")

        if self.start is not None:
            self.ax.scatter([self.start[0]], [self.start[1]], marker="o", s=120, color="tab:green", label="Start")

        if self.goal is not None:
            self.ax.scatter([self.goal[0]], [self.goal[1]], marker="x", s=120, color="tab:blue", label="Goal")

        if self.start is not None or self.goal is not None:
            self.ax.legend(loc="upper right")

        if self.preview_patch is not None:
            self.ax.add_patch(self.preview_patch)

        self.fig.canvas.draw_idle()

    def event_point(self, event) -> Optional[Point]:
        if event.xdata is None or event.ydata is None:
            return None
        return self.snap_point(event.xdata, event.ydata)

    def on_press(self, event) -> None:
        point = self.event_point(event)
        if point is None:
            return

        if self.mode == "start":
            self.start = point
            self.mode = "obstacle"
            self.draw()
            return

        if self.mode == "goal":
            self.goal = point
            self.mode = "obstacle"
            self.draw()
            return

        self.drag_start = point

    def on_move(self, event) -> None:
        if self.drag_start is None or self.mode != "obstacle":
            return

        point = self.event_point(event)
        if point is None:
            return

        rect = self.normalize_rect(self.drag_start, point)
        self.preview_patch = None
        if rect is not None:
            x_min, y_min, x_max, y_max = rect
            self.preview_patch = PatchRectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                fill=False,
                linewidth=2,
                linestyle="--",
                color="tab:orange",
            )
        self.draw()

    def on_release(self, event) -> None:
        if self.drag_start is None or self.mode != "obstacle":
            self.drag_start = None
            self.preview_patch = None
            self.draw()
            return

        point = self.event_point(event)
        self.preview_patch = None
        if point is not None:
            rect = self.normalize_rect(self.drag_start, point)
            if rect is not None:
                self.obstacles.append(rect)
        self.drag_start = None
        self.draw()

    def on_key(self, event) -> None:
        if event.key == "a":
            self.mode = "start"
        elif event.key == "b":
            self.mode = "goal"
        elif event.key == "d":
            if self.obstacles:
                self.obstacles.pop()
        elif event.key == "enter":
            self.save()
        self.draw()

    def on_delta_change(self, label: str) -> None:
        grid_resolution_m = 0.05 if label == "0.05 m" else 0.15
        self.planning = PlanningConfig(grid_resolution_m=grid_resolution_m)
        self.drag_start = None
        self.preview_patch = None
        self.draw()

    def save(self) -> None:
        if self.start is None or self.goal is None:
            print("Start and goal must both be selected before saving.")
            return
        save_layout(
            self.obstacles,
            self.start,
            self.goal,
            self.planning.grid_resolution_m,
        )
        print(f"Saved layout to {LAYOUT_FILE}")

    def run(self) -> None:
        self.draw()
        plt.show()


def main() -> None:
    editor = CourseEditor()
    editor.run()


if __name__ == "__main__":
    main()
