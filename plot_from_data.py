"""
Step 2: Plot trajectory map from extracted JSON data.
Run with system Python (3.11+, has matplotlib).

Coordinate transform:
  CARLA uses Unreal's left-handed system: +X=forward, +Y=right, yaw CW→right turn.
  Matplotlib uses right-handed: +X=right, +Y=up.
  To fix handedness, we swap axes:
      plot_x = CARLA_Y   (CARLA "right" → plot "right")
      plot_y = CARLA_X   (CARLA "forward" → plot "up")
  This preserves turn direction (right turn → right turn on plot).
"""

import json
import math
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


def carla_to_plot(carla_x, carla_y):
    """Convert CARLA left-handed coords to right-handed plot coords.
    Swaps axes: CARLA Y (right) -> plot X, CARLA X (forward) -> plot Y.
    """
    return carla_y, carla_x


def plot_trajectory(data, output_path="trajectory_map.png"):
    route = data["route"]
    road_network = data["road_network"]
    map_name = data["map_name"]

    turn_events = route["turn_events"]
    close_to_index = route["close_to_index"]
    right_lane_prepare_index = route["right_lane_prepare_index"]
    route_length = route["route_length"]

    waypoints = route["waypoints"]

    # ---- Convert all coordinates: CARLA (X,Y) -> plot (CARLA_Y, CARLA_X) ----
    # Route waypoints: use transformed coordinates
    route_px = [wp["y"] for wp in waypoints]   # CARLA Y -> plot X (East -> right)
    route_py = [wp["x"] for wp in waypoints]   # CARLA X -> plot Y (North/forward -> up)

    # Road network: transform every segment
    road_net_plot = []
    for r in road_network:
        road_net_plot.append({
            "road_id": r["road_id"],
            "px": r["y"],    # CARLA Y -> plot X
            "py": r["x"],    # CARLA X -> plot Y
        })

    # ---- Canvas ----
    fig, ax = plt.subplots(figsize=(26, 20))

    # ---- 1. Background road network (light gray) ----
    for r in road_net_plot:
        ax.plot(r["px"], r["py"], color="#D0D0D0", linewidth=0.9, alpha=0.65, zorder=1)

    # ---- 2. Ego route (gradient viridis) ----
    n_pts = len(waypoints)
    cmap = plt.cm.viridis
    for i in range(n_pts - 1):
        t = i / (n_pts - 1)
        ax.plot(
            route_px[i : i + 2],
            route_py[i : i + 2],
            color=cmap(t * 0.92),
            linewidth=4.2,
            solid_capstyle="round",
            zorder=5,
        )

    # Direction arrows
    step_arrow = max(1, n_pts // 40)
    for i in range(0, n_pts - step_arrow, step_arrow):
        dx = route_px[i + step_arrow] - route_px[i]
        dy = route_py[i + step_arrow] - route_py[i]
        seg_len = math.sqrt(dx * dx + dy * dy)
        if seg_len > 0.1:
            dx, dy = dx / seg_len * 8.0, dy / seg_len * 8.0
            ax.arrow(
                route_px[i], route_py[i], dx, dy,
                head_width=5.5, head_length=7.0, fc="#1B5E20",
                ec="white", linewidth=0.6, alpha=0.55, zorder=6,
            )

    # ---- 3. Road ID labels along route ----
    last_road_id = None
    last_label_idx = -999
    seen_roads = set()

    for i, wp in enumerate(waypoints):
        rid = wp["road_id"]
        need_label = False
        if rid != last_road_id:
            if rid not in seen_roads:
                need_label = True
                seen_roads.add(rid)
            elif i - last_label_idx >= 25:
                need_label = True
        elif i - last_label_idx >= 25:
            need_label = True

        if need_label:
            px, py = carla_to_plot(wp["x"], wp["y"])
            ax.annotate(
                f"R{rid}",
                xy=(px, py),
                fontsize=7.5,
                fontweight="bold",
                color="#1A237E",
                ha="center",
                va="center",
                bbox=dict(
                    boxstyle="round,pad=0.35",
                    facecolor="white",
                    edgecolor="#1A237E",
                    alpha=0.88,
                    linewidth=0.8,
                ),
                zorder=11,
            )
            last_label_idx = i
            last_road_id = rid

    # ---- 4. Key points ----
    # Start point
    start_wp = waypoints[0]
    sx, sy = carla_to_plot(start_wp["x"], start_wp["y"])
    ax.scatter(
        sx, sy, c="#2E7D32", s=260, marker="o",
        edgecolors="white", linewidth=2.0, zorder=15,
    )
    ax.annotate(
        "START\nR{}".format(start_wp["road_id"]),
        xy=(sx, sy),
        xytext=(sx + 25, sy + 20),
        fontsize=9, fontweight="bold", color="#2E7D32",
        arrowprops=dict(arrowstyle="->", color="#2E7D32", lw=1.5),
        bbox=dict(boxstyle="round", facecolor="white", edgecolor="#2E7D32", alpha=0.9),
        zorder=16,
    )

    # Loop close point
    if close_to_index is not None and 0 <= close_to_index < len(waypoints):
        cwp = waypoints[close_to_index]
        cx, cy = carla_to_plot(cwp["x"], cwp["y"])
        ax.scatter(
            cx, cy, c="#C62828", s=220, marker="D",
            edgecolors="white", linewidth=1.5, zorder=15,
        )
        ax.annotate(
            "CLOSE (idx={})".format(close_to_index),
            xy=(cx, cy),
            xytext=(cx + 30, cy - 25),
            fontsize=8, color="#C62828", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#C62828", lw=1.2),
            bbox=dict(boxstyle="round", facecolor="white", edgecolor="#C62828", alpha=0.9),
            zorder=16,
        )

    # Right-lane prepare point
    if right_lane_prepare_index is not None and 0 <= right_lane_prepare_index < len(waypoints):
        rwp = waypoints[right_lane_prepare_index]
        rx, ry = carla_to_plot(rwp["x"], rwp["y"])
        ax.scatter(
            rx, ry, c="#E65100", s=180, marker="s",
            edgecolors="white", linewidth=1.5, zorder=15,
        )
        ax.annotate(
            "R-Lane Prep (idx={})".format(right_lane_prepare_index),
            xy=(rx, ry),
            xytext=(rx - 25, ry + 30),
            fontsize=8, color="#E65100", fontweight="bold",
            arrowprops=dict(arrowstyle="->", color="#E65100", lw=1.2),
            bbox=dict(boxstyle="round", facecolor="white", edgecolor="#E65100", alpha=0.9),
            zorder=16,
        )

    # ---- 5. Turn event labels ----
    turn_colors = {"left": "#1565C0", "right": "#C62828"}
    for idx, evt in enumerate(turn_events):
        si, ei = evt["start_index"], evt["end_index"]
        mid_i = (si + ei) // 2
        if not (0 <= mid_i < len(waypoints)):
            continue
        wp = waypoints[mid_i]
        tx, ty = carla_to_plot(wp["x"], wp["y"])
        offset_x = (idx % 3 - 1) * 18
        offset_y = (idx % 2) * 18 - 5
        color = turn_colors.get(evt["direction"], "#333")
        ax.annotate(
            "T{}: {}\n{:.0f} deg".format(idx + 1, evt["direction"], evt["degrees"]),
            xy=(tx, ty),
            xytext=(tx + offset_x, ty + offset_y),
            fontsize=7.5,
            color="white",
            fontweight="bold",
            ha="center",
            va="center",
            bbox=dict(boxstyle="round,pad=0.4", facecolor=color, edgecolor="none", alpha=0.82),
            zorder=14,
        )

    # ---- 6. Background road IDs (not on route) ----
    for r in road_net_plot:
        xs, ys = r["px"], r["py"]
        if len(xs) < 2:
            continue
        mid_i = len(xs) // 2
        mx, my = xs[mid_i], ys[mid_i]
        if r["road_id"] not in seen_roads and len(xs) > 6:
            ax.text(
                mx, my, "R{}".format(r["road_id"]),
                fontsize=4.5, color="#9E9E9E", alpha=0.55,
                ha="center", va="center", zorder=2,
                bbox=dict(boxstyle="round,pad=0.1", facecolor="white", edgecolor="none", alpha=0.4),
            )

    # ---- 7. Legend ----
    legend_elements = [
        mpatches.Patch(color="#2E7D32", label="Start Point"),
        mpatches.Patch(color="#C62828", label="Loop Close Point"),
        mpatches.Patch(color="#E65100", label="Right-Lane Prepare"),
        mpatches.Patch(color="#1565C0", label="Left Turn"),
        mpatches.Patch(color="#C62828", label="Right Turn"),
        mpatches.Patch(color="#D0D0D0", label="Road Network"),
    ]
    ax.legend(handles=legend_elements, loc="upper left", fontsize=9,
              framealpha=0.92, edgecolor="#666")

    # ---- 8. Title & formatting ----
    n_roads_route = len(set(wp["road_id"] for wp in waypoints))
    n_roads_network = len(road_network)
    title = (
        "{}  Trajectory Map (Ego Planned Route)".format(map_name) + "\n"
        "Route Length ~{:.0f} m  |  Waypoints: {}  |  Turns: {}  |  Roads on Route: {} / Network: {}".format(
            route_length, len(waypoints), len(turn_events), n_roads_route, n_roads_network
        ) + "\n"
        "Coordinate transform: plot_X = CARLA_Y (East), plot_Y = CARLA_X (North)  --  Right turn = Right on plot"
    )
    ax.set_title(title, fontsize=13, fontweight="bold", pad=16)

    ax.set_xlabel("CARLA Y / East (m)  --  plot X", fontsize=11)
    ax.set_ylabel("CARLA X / Forward (m)  --  plot Y", fontsize=11)

    # Auto-zoom
    x_min, x_max = min(route_px), max(route_px)
    y_min, y_max = min(route_py), max(route_py)
    x_margin = max(60, (x_max - x_min) * 0.15)
    y_margin = max(60, (y_max - y_min) * 0.15)
    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linestyle="--", linewidth=0.5)

    # Scale bar (100 m)
    ruler_len = 100.0
    ruler_y = y_min - y_margin * 0.5
    ruler_x0 = x_min - x_margin * 0.4
    ax.plot([ruler_x0, ruler_x0 + ruler_len], [ruler_y, ruler_y],
            "k-", linewidth=2, zorder=20)
    ax.text(ruler_x0 + ruler_len / 2, ruler_y - 12,
            "100 m", ha="center", fontsize=9, fontweight="bold", zorder=20)

    # ---- 9. Save ----
    fig.savefig(output_path, dpi=200, bbox_inches="tight", facecolor="white",
                edgecolor="none")
    plt.close(fig)
    print("Image saved: {}".format(output_path))


if __name__ == "__main__":
    if len(sys.argv) > 1:
        data_path = sys.argv[1]
    else:
        data_path = "trajectory_data.json"

    with open(data_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    print("Data loaded from: {}".format(data_path))

    output_path = "trajectory_map.png"
    if len(sys.argv) > 2:
        output_path = sys.argv[2]

    plot_trajectory(data, output_path)
