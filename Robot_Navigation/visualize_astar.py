#!/usr/bin/env python3
"""Animate an occupancy-grid path using Matplotlib's FuncAnimation.

SETUP (Python 3.9+)
    python3 -m pip install numpy matplotlib

Put this script, occupancy_grid.csv, and path_to_goal.txt in the same folder:
    python3 visualize_astar.py
    python3 visualize_astar.py --fps 30 --paused
    python3 visualize_astar.py --grid /path/map.csv --path /path/route.txt

INPUTS
    CSV: rectangular matrix, 0 = free, 100 = obstacle; # comments are ignored.
    Path: the supplied C program writes "Move Number 0 is row,col".
          Plain "row,col", "row col", and "(row, col)" lines also work.
          For an actual x,y file instead, pass --path-order xy.

COORDINATES
    Default: x = column, y = row; row 0 is at the TOP and y increases downward.
    This matches astar.c, where moving up decrements the row. No transpose.
    For Cartesian display, pass --coordinates cartesian:
        x = column, y = number_of_rows - 1 - row; y increases upward.
    Both modes keep the first CSV row visually at the TOP, and place integer
    coordinates at cell CENTERS. No map scale or world origin is assumed.
    Start and goal come from the first/last PATH entries, not CSV metadata.

CONTROLS
    Space / Play-Pause button: pause or resume (at the end, play again).
    Left / Right: pause and move backward/forward by one path position.
    Slider: pause and jump to a path position.
    R / Restart button: return to the start and play.
    Escape: close the window. --fps sets the target playback frame rate.

NOTES
    One animation step = one path edge. This is a point-robot visualization,
    not a physical dynamics or collision-footprint simulation.
    --check-only validates files without opening a window.
    Run as a desktop Python script for working GUI controls, not a static
    notebook plot. If Ubuntu has no GUI backend: sudo apt install python3-tk
"""

from pathlib import Path
import argparse
import re
import sys

import numpy as np


# ---- Editable defaults; command-line arguments override these. ----
BASE_DIR = Path(__file__).resolve().parent
GRID_FILE = BASE_DIR / "occupancy_grid7.csv"
PATH_FILE = BASE_DIR / "path_to_goal.txt"
FPS = 50000.0
PATH_ORDER = "row-col"        # The uploaded astar.c writes row first.
COORDINATES = "image"         # "image" or "cartesian" (see above).
START_PAUSED = False
SHOW_PLANNED_PATH = False     # False shows only the trail already travelled.


VERBOSE_PAIR = re.compile(
    r"Move\s+Number\s+\d+\s+is\s+(-?\d+)\s*,\s*(-?\d+)", re.IGNORECASE
)
PLAIN_PAIR = re.compile(r"(-?\d+)(?:\s*,\s*|\s+)(-?\d+)")


def load_grid(filename):
    """Return a 2-D Boolean array indexed as obstacles[row, col]."""
    values = np.loadtxt(
        filename, delimiter=",", comments="#", dtype=float,
        ndmin=2, encoding="utf-8-sig",
    )
    if values.size == 0:
        raise ValueError("The occupancy CSV contains no grid cells.")
    if not np.isin(values, [0, 100]).all():
        bad = np.unique(values[~np.isin(values, [0, 100])])
        raise ValueError(
            f"Expected only 0 (free) and 100 (obstacle); found {bad[:8]}. "
            "Unknown cells need an explicit policy before visualization."
        )
    return values == 100


def load_path(filename, order=PATH_ORDER):
    """Read coordinates into canonical (row, col) order, as integer pairs."""
    if order not in ("row-col", "xy"):
        raise ValueError("Path order must be 'row-col' or 'xy'.")
    points = []
    with Path(filename).open(encoding="utf-8-sig") as stream:
        for line_number, raw in enumerate(stream, 1):
            line = raw.partition("#")[0].strip()
            if not line:
                continue
            match = VERBOSE_PAIR.fullmatch(line)
            if match is None:
                if line.startswith("(") and line.endswith(")"):
                    line = line[1:-1].strip()
                match = PLAIN_PAIR.fullmatch(line)
            if match is None:
                raise ValueError(
                    f"Path line {line_number}: expected two integer coordinates "
                    f"or 'Move Number k is row,col'; got {raw.strip()!r}."
                )
            first, second = map(int, match.groups())
            points.append((first, second) if order == "row-col" else (second, first))
    if not points:
        raise ValueError("The path file contains no coordinates.")
    return np.asarray(points, dtype=np.int64)


def validate_path(obstacles, path_rc):
    """Reject out-of-bounds cells, obstacles, and non-cardinal path jumps."""
    rows, cols = obstacles.shape
    rr, cc = path_rc.T
    outside = (rr < 0) | (rr >= rows) | (cc < 0) | (cc >= cols)
    if outside.any():
        index = int(np.flatnonzero(outside)[0])
        raise ValueError(
            f"Path position {index}: (row, col)={tuple(path_rc[index])} is "
            f"outside the {rows} x {cols} grid. Check --path-order."
        )
    blocked = obstacles[rr, cc]
    if blocked.any():
        index = int(np.flatnonzero(blocked)[0])
        raise ValueError(
            f"Path position {index}: (row, col)={tuple(path_rc[index])} is "
            "an obstacle. Check the map and --path-order."
        )
    lengths = np.abs(np.diff(path_rc, axis=0)).sum(axis=1)
    if np.any(lengths != 1):
        index = int(np.flatnonzero(lengths != 1)[0])
        raise ValueError(
            f"Path edge {index} -> {index + 1} is not a one-cell cardinal move: "
            f"{tuple(path_rc[index])} -> {tuple(path_rc[index + 1])}. "
            "This visualizer validates the four-direction paths used by astar.c."
        )


def plot_coordinates(path_rc, rows, mode):
    """Translate source matrix indices into the selected plotting coordinates."""
    x = path_rc[:, 1]
    y = path_rc[:, 0] if mode == "image" else rows - 1 - path_rc[:, 0]
    return np.column_stack((x, y))


class PathPlayer:
    """Own the artists, widgets, and a live reference to FuncAnimation."""

    def __init__(self, obstacles, path_rc, fps=FPS, coordinates=COORDINATES,
                 paused=START_PAUSED, show_plan=SHOW_PLANNED_PATH):
        # Delay GUI imports so --check-only also works on headless machines.
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.colors import ListedColormap
        from matplotlib.patches import Patch
        from matplotlib.ticker import MaxNLocator
        from matplotlib.widgets import Button, Slider

        self.plt = plt
        self.path_rc = path_rc
        rows, cols = obstacles.shape
        self.xy = plot_coordinates(path_rc, rows, coordinates)
        self.index = 0
        self.last = len(path_rc) - 1
        self.running = not paused and self.last > 0
        self.fps = fps

        self.fig, self.ax = plt.subplots(figsize=(14, 8))
        self.fig.subplots_adjust(left=0.06, right=0.98, bottom=0.23, top=0.83)
        self.fig.suptitle(
            f"A* path playback  |  {rows} rows x {cols} columns  |  {self.last} moves",
            fontsize=15, fontweight="bold", y=0.98,
        )
        # origin='upper' places the first matrix row at the top in BOTH modes.
        # The extent sets the axis direction; the same conversion is used for
        # every path artist. Half-cell borders keep integer points centered.
        extent = (-0.5, cols - 0.5,
                  rows - 0.5 if coordinates == "image" else -0.5,
                  -0.5 if coordinates == "image" else rows - 0.5)
        cmap = ListedColormap(["#fafaf7", "#20242c"])
        self.ax.imshow(
            obstacles.astype(np.uint8), cmap=cmap, vmin=0, vmax=1,
            origin="upper", extent=extent, interpolation="nearest", aspect="equal",
        )
        self.ax.set_xlim(extent[:2])
        self.ax.set_ylim(extent[2:])
        self.ax.set_xlabel("x = column (cell index)")
        self.ax.set_ylabel(
            "y = row (increases downward)" if coordinates == "image"
            else "y = M - 1 - row (increases upward)"
        )
        self.ax.xaxis.set_major_locator(MaxNLocator(nbins=12, integer=True))
        self.ax.yaxis.set_major_locator(MaxNLocator(nbins=8, integer=True))
        # Cell borders help on small maps; avoid thousands of grid lines on
        # large maps. Nearest-neighbor rendering always preserves crisp cells.
        if max(rows, cols) <= 120:
            self.ax.set_xticks(np.arange(cols + 1) - 0.5, minor=True)
            self.ax.set_yticks(np.arange(rows + 1) - 0.5, minor=True)
            self.ax.grid(which="minor", color="#9da4af", alpha=0.16, linewidth=0.3)
            self.ax.tick_params(which="minor", bottom=False, left=False)

        if show_plan:
            self.ax.plot(self.xy[:, 0], self.xy[:, 1], color="#4b91b5",
                         linestyle="--", linewidth=1, alpha=0.35, zorder=2)
        self.trail, = self.ax.plot([], [], color="#159ec4", linewidth=2,
                                   label="Travelled", zorder=3)
        start, = self.ax.plot(*self.xy[0], marker="o", linestyle="none",
                              color="#16a34a", markeredgecolor="white",
                              markersize=10, label="Start", zorder=4)
        goal, = self.ax.plot(*self.xy[-1], marker="*", linestyle="none",
                             color="#e43b3b", markeredgecolor="white",
                             markersize=14, label="Goal", zorder=4)
        self.robot, = self.ax.plot([], [], marker="o", linestyle="none",
                                   color="#2563eb", markeredgecolor="white",
                                   markersize=6, label="Robot", zorder=5)
        self.ax.legend(
            handles=[Patch(facecolor=cmap(0), edgecolor="#aaa", label="Free"),
                     Patch(facecolor=cmap(1), label="Obstacle"),
                     start, goal, self.robot, self.trail],
            loc="lower center", bbox_to_anchor=(0.5, 1.015), ncol=6,
            frameon=False, fontsize=10,
        )
        self.status = self.fig.text(0.06, 0.145, "", fontsize=11)
        self.fig.text(
            0.5, 0.017,
            "Space: play/pause    |    Left/Right: single step    |    R: restart    |    Esc: close",
            ha="center", fontsize=10, color="#424954",
        )
        self.play_button = Button(self.fig.add_axes([0.06, 0.065, 0.11, 0.045]), "Pause")
        self.restart_button = Button(self.fig.add_axes([0.185, 0.065, 0.11, 0.045]), "Restart")
        self.slider = Slider(
            self.fig.add_axes([0.37, 0.072, 0.56, 0.03]), "Move",
            0, max(1, self.last), valinit=0, valstep=1, valfmt="%0.0f",
        )
        if self.last == 0:
            self.slider.set_active(False)
        self.play_button.on_clicked(self.toggle)
        self.restart_button.on_clicked(self.restart)
        self.slider.on_changed(self.seek)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self._render()
        self.animation = FuncAnimation(
            self.fig, self._tick, init_func=self._render,
            interval=1000.0 / fps, blit=False, repeat=False,
            cache_frame_data=False,
        )

    def _render(self):
        x, y = self.xy[self.index]
        self.robot.set_data([x], [y])
        self.trail.set_data(self.xy[:self.index + 1, 0], self.xy[:self.index + 1, 1])
        row, col = self.path_rc[self.index]
        state = "Goal reached" if self.index == self.last else (
            "Playing" if self.running else "Paused"
        )
        self.status.set_text(
            f"{state}  |  Move {self.index}/{self.last}  |  "
            f"source (row, col) = ({row}, {col})  |  "
            f"display (x, y) = ({x}, {y})  |  target {self.fps:g} FPS"
        )
        self.play_button.label.set_text("Pause" if self.running else "Play")
        # Updating the slider from the animation must not trigger seek(),
        # which would pause playback or recursively update the slider.
        old_events, old_draw = self.slider.eventson, self.slider.drawon
        self.slider.eventson = self.slider.drawon = False
        try:
            self.slider.set_val(self.index)
        finally:
            self.slider.eventson, self.slider.drawon = old_events, old_draw
        return self.robot, self.trail, self.status

    def _tick(self, _frame):
        if self.running and self.index < self.last:
            self.index += 1
            if self.index == self.last:
                self.running = False
                self.animation.pause()
        return self._render()

    def _refresh(self):
        self._render()
        self.fig.canvas.draw_idle()

    def toggle(self, _event=None):
        if self.last == 0:
            return
        if self.running:
            self.running = False
            self.animation.pause()
        else:
            if self.index == self.last:
                self.index = 0
            self.running = True
            self.animation.resume()
        self._refresh()

    def seek(self, value):
        self.running = False
        self.animation.pause()
        self.index = int(np.clip(round(value), 0, self.last))
        self._refresh()

    def restart(self, _event=None):
        self.index = 0
        self.running = self.last > 0
        if self.running:
            self.animation.resume()
        self._refresh()

    def on_key(self, event):
        if event.key in (" ", "space"):
            self.toggle()
        elif event.key == "left":
            self.seek(self.index - 1)
        elif event.key == "right":
            self.seek(self.index + 1)
        elif event.key in ("r", "R"):
            self.restart()
        elif event.key == "escape":
            self.plt.close(self.fig)

    def show(self):
        # The first draw starts FuncAnimation's timer. Stop it AFTER that draw
        # when --paused was selected, so the initial pause is respected.
        self.fig.canvas.draw()
        if not self.running:
            self.animation.pause()
        self.plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Animate the supplied A* path on a 0/100 occupancy CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--grid", type=Path, default=GRID_FILE, help="Occupancy CSV")
    parser.add_argument("--path", type=Path, default=PATH_FILE, help="Path text file")
    parser.add_argument("--fps", type=float, default=FPS, help="Target moves per second")
    parser.add_argument("--path-order", choices=("row-col", "xy"), default=PATH_ORDER,
                        help="Coordinate order IN THE PATH FILE, independent of display mode")
    parser.add_argument("--coordinates", choices=("image", "cartesian"), default=COORDINATES,
                        help="Display coordinate convention (does not change file parsing)")
    parser.add_argument("--paused", action="store_true", default=START_PAUSED,
                        help="Start paused")
    parser.add_argument("--show-plan", action="store_true", default=SHOW_PLANNED_PATH,
                        help="Also draw the full planned route faintly behind the trail")
    parser.add_argument("--check-only", action="store_true",
                        help="Validate files and print a summary without opening a GUI")
    args = parser.parse_args(argv)
    if not np.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be a finite number greater than zero.")
    try:
        obstacles = load_grid(args.grid.expanduser())
        path_rc = load_path(args.path.expanduser(), args.path_order)
        validate_path(obstacles, path_rc)
    except (OSError, ValueError, OverflowError) as exc:
        parser.error(str(exc))
    rows, cols = obstacles.shape
    print(f"Map: {rows} rows x {cols} columns; {int(obstacles.sum())} obstacle cells.")
    print(f"Path: {len(path_rc)} positions, {len(path_rc) - 1} valid cardinal moves.")
    print(f"Start (row, col): {tuple(map(int, path_rc[0]))}; "
          f"goal: {tuple(map(int, path_rc[-1]))}.")
    print(f"Input order: {args.path_order}; display coordinates: {args.coordinates}.")
    if args.check_only:
        return 0
    player = PathPlayer(obstacles, path_rc, args.fps, args.coordinates,
                        args.paused, args.show_plan)
    player.show()  # Retain player AND its animation until the window closes.
    return 0


if __name__ == "__main__":
    sys.exit(main())
