import argparse
import json
import os

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon
from matplotlib.widgets import Slider


def load_json(path):
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def as_xy(point):
    return float(point["x"]), float(point["y"])


def get_map_bounds(scenario):
    vertices = scenario.get("map_boundaries", {}).get("vertices", [])
    if not vertices:
        return None
    xs = [float(v["x"]) for v in vertices]
    ys = [float(v["y"]) for v in vertices]
    return min(xs), max(xs), min(ys), max(ys)


def parse_playback_series(playback_raw):
    required = {"time", "x", "y", "alt_layer", "energy"}

    if isinstance(playback_raw, list):
        playback = playback_raw
    elif isinstance(playback_raw, dict):
        participants = playback_raw.get("participants", [])
        if not participants:
            raise ValueError("Playback object has no participants")
        first = participants[0]
        playback = first.get("history", [])
    else:
        raise ValueError("Playback must be a list or an object export")

    if not isinstance(playback, list) or len(playback) == 0:
        raise ValueError("Playback must be a non-empty list of snapshots")

    for index, row in enumerate(playback):
        if not isinstance(row, dict) or not required.issubset(row.keys()):
            raise ValueError(f"Invalid playback row at index {index}")

    times = [int(row["time"]) for row in playback]
    xs = [float(row["x"]) for row in playback]
    ys = [float(row["y"]) for row in playback]
    alt_layers = [int(row["alt_layer"]) for row in playback]
    energies = [float(row["energy"]) for row in playback]
    return playback, times, xs, ys, alt_layers, energies


def region_to_patch(region, edgecolor, facecolor, linewidth=1.3, alpha=0.25, linestyle="-"):
    region_type = region.get("type")
    if region_type == "CircleRegion":
        center = region.get("center_pos") or region.get("center")
        if center is None:
            return None
        cx, cy = as_xy(center)
        radius = float(region.get("radius", 0))
        return Circle((cx, cy), radius=radius, edgecolor=edgecolor, facecolor=facecolor, linewidth=linewidth, alpha=alpha, linestyle=linestyle)

    vertices = region.get("vertices", [])
    if not vertices:
        return None
    points = [as_xy(v) for v in vertices]
    return Polygon(points, closed=True, edgecolor=edgecolor, facecolor=facecolor, linewidth=linewidth, alpha=alpha, linestyle=linestyle)


def get_notam_phase(notam, time_value):
    advisory = int(notam.get("advisory_start_time", 10**9))
    controlled = int(notam.get("controlled_start_time", 10**9))
    restricted = int(notam.get("restricted_start_time", 10**9))
    if time_value < advisory:
        return "inactive"
    if time_value < controlled:
        return "advisory"
    if time_value < restricted:
        return "controlled"
    return "restricted"


def get_traffic_state(segments, time_value):
    for seg in segments:
        start_time = int(seg.get("start_time", 0))
        end_time = int(seg.get("end_time", -1))
        if start_time <= time_value < end_time:
            sx, sy = as_xy(seg["start_pos"])
            vx, vy = as_xy(seg["velocity"])
            dt = time_value - start_time
            x = sx + vx * dt
            y = sy + vy * dt
            return x, y, int(seg.get("alt_layer", 0)), seg
    return None


def get_recent_traffic_trail(segment, time_value, trail_steps=25):
    start_time = int(segment.get("start_time", 0))
    sx, sy = as_xy(segment["start_pos"])
    vx, vy = as_xy(segment["velocity"])
    begin = max(start_time, time_value - trail_steps)
    xs = []
    ys = []
    for tick in range(begin, time_value + 1):
        dt = tick - start_time
        xs.append(sx + vx * dt)
        ys.append(sy + vy * dt)
    return xs, ys


def main():
    parser = argparse.ArgumentParser(description="Interactive playback viewer with slider")
    parser.add_argument("--playback", default="playback.json", help="Playback JSON path")
    parser.add_argument("--scenario", default="scenario.json", help="Scenario JSON path for map bounds")
    parser.add_argument("--hidden", default="aerohacks/dummy_hidden.json", help="Hidden events JSON path for NOTAMs/NPC traffic")
    args = parser.parse_args()

    if not os.path.exists(args.playback):
        raise FileNotFoundError(f"Playback file not found: {args.playback}")

    playback_raw = load_json(args.playback)
    playback, times, xs, ys, alt_layers, energies = parse_playback_series(playback_raw)

    scenario = None
    bounds = None
    if args.scenario and os.path.exists(args.scenario):
        scenario = load_json(args.scenario)
        bounds = get_map_bounds(scenario)

    hidden = None
    if args.hidden and os.path.exists(args.hidden):
        hidden = load_json(args.hidden)

    fig = plt.figure(figsize=(12, 8))
    ax_map = fig.add_axes([0.08, 0.28, 0.86, 0.66])
    ax_slider = fig.add_axes([0.10, 0.10, 0.80, 0.05])

    ax_map.set_title("AeroHacks Playback Viewer (Scenario + Dynamic Overlays)")
    ax_map.set_xlabel("X (m)")
    ax_map.set_ylabel("Y (m)")

    if bounds is not None:
        min_x, max_x, min_y, max_y = bounds
        ax_map.set_xlim(min_x, max_x)
        ax_map.set_ylim(min_y, max_y)
    else:
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        pad_x = max(1.0, (max_x - min_x) * 0.05)
        pad_y = max(1.0, (max_y - min_y) * 0.05)
        ax_map.set_xlim(min_x - pad_x, max_x + pad_x)
        ax_map.set_ylim(min_y - pad_y, max_y + pad_y)

    ax_map.set_aspect("equal", adjustable="box")
    ax_map.grid(alpha=0.3)

    if scenario is not None:
        for item in scenario.get("permanent_constraints", []):
            patch = region_to_patch(
                item.get("region", {}),
                edgecolor="#ff8c00",
                facecolor="#ffddaa",
                linewidth=1.5,
                alpha=0.22,
            )
            if patch is not None:
                ax_map.add_patch(patch)

        for region in scenario.get("static_obstacles", []):
            patch = region_to_patch(
                region,
                edgecolor="#8b5a2b",
                facecolor="#d2b48c",
                linewidth=1.2,
                alpha=0.30,
            )
            if patch is not None:
                ax_map.add_patch(patch)

        for site in scenario.get("emergency_landing_sites", []):
            patch = region_to_patch(
                site.get("region", {}),
                edgecolor="#16a34a",
                facecolor="#86efac",
                linewidth=1.4,
                alpha=0.30,
            )
            if patch is not None:
                ax_map.add_patch(patch)

        goal_region = scenario.get("mission_goal", {}).get("region")
        if goal_region:
            patch = region_to_patch(
                goal_region,
                edgecolor="#22c55e",
                facecolor="#bbf7d0",
                linewidth=2.0,
                alpha=0.38,
            )
            if patch is not None:
                ax_map.add_patch(patch)

        start_pos = scenario.get("start_state", {}).get("position")
        if start_pos is not None:
            sx, sy = as_xy(start_pos)
            ax_map.scatter([sx], [sy], marker="s", s=80, color="#111827", zorder=5)
            ax_map.text(sx, sy, " START", fontsize=8, color="#111827", va="bottom")

        goal_region = scenario.get("mission_goal", {}).get("region", {})
        goal_center = None
        if goal_region.get("type") == "CircleRegion" and goal_region.get("center_pos"):
            goal_center = as_xy(goal_region["center_pos"])
        else:
            goal_vertices = goal_region.get("vertices", [])
            if goal_vertices:
                gx = sum(v["x"] for v in goal_vertices) / len(goal_vertices)
                gy = sum(v["y"] for v in goal_vertices) / len(goal_vertices)
                goal_center = float(gx), float(gy)
        if goal_center is not None:
            ax_map.text(goal_center[0], goal_center[1], " GOAL", fontsize=8, color="#166534", va="center")

    notam_layers = []
    if hidden is not None:
        for notam in hidden.get("shrinking_notams", []):
            patch = region_to_patch(
                notam.get("region", {}),
                edgecolor="#fbbf24",
                facecolor="#fde68a",
                linewidth=1.8,
                alpha=0.18,
            )
            if patch is not None:
                patch.set_visible(False)
                ax_map.add_patch(patch)
                notam_layers.append((notam, patch))

    traffic_traces = []
    if hidden is not None:
        traffic_traces = hidden.get("traffic_traces", [])

    npc_scatter = ax_map.scatter([], [], s=34, color="#0ea5e9", marker="x", zorder=6)
    npc_trails = []
    for _ in traffic_traces:
        line, = ax_map.plot([], [], color="#38bdf8", linewidth=0.9, alpha=0.8)
        npc_trails.append(line)

    ax_map.plot(xs, ys, color="#b0b0b0", linewidth=1.5, alpha=0.8)
    trail_line, = ax_map.plot([xs[0]], [ys[0]], color="#1f77b4", linewidth=2.5)
    current_point = ax_map.scatter([xs[0]], [ys[0]], s=70, color="#d62728", zorder=3)

    legend_handles = [
        Line2D([0], [0], color="#1f77b4", lw=2.5, label="Ownship Trail"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#d62728", markersize=8, label="Ownship"),
        Line2D([0], [0], marker="x", color="#0ea5e9", markersize=8, lw=0, label="NPC Drones"),
        Line2D([0], [0], color="#ff8c00", lw=2, label="Permanent Constraints"),
        Line2D([0], [0], color="#16a34a", lw=2, label="Emergency Sites / Goal"),
        Line2D([0], [0], color="#fbbf24", lw=2, label="NOTAM Advisory"),
        Line2D([0], [0], color="#f97316", lw=2, label="NOTAM Controlled"),
        Line2D([0], [0], color="#dc2626", lw=2, label="NOTAM Restricted"),
    ]
    ax_map.legend(handles=legend_handles, loc="upper right", fontsize=8, framealpha=0.9)

    status_text = ax_map.text(
        0.02,
        0.98,
        "",
        transform=ax_map.transAxes,
        va="top",
        ha="left",
        bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "#cccccc"},
    )

    slider = Slider(
        ax=ax_slider,
        label="Frame",
        valmin=0,
        valmax=len(playback) - 1,
        valinit=0,
        valstep=1,
    )

    def render(frame_idx):
        idx = int(frame_idx)
        t = times[idx]
        ownship_alt = alt_layers[idx]

        trail_line.set_data(xs[: idx + 1], ys[: idx + 1])
        current_point.set_offsets([[xs[idx], ys[idx]]])

        advisory_count = 0
        controlled_count = 0
        restricted_count = 0
        for notam, patch in notam_layers:
            phase = get_notam_phase(notam, t)
            if phase == "inactive":
                patch.set_visible(False)
                continue
            patch.set_visible(True)

            applies_to_alt = ownship_alt in set(notam.get("alt_layers", []))
            emphasis_alpha = 0.30 if applies_to_alt else 0.14

            if phase == "advisory":
                advisory_count += 1
                patch.set_edgecolor("#fbbf24")
                patch.set_facecolor("#fde68a")
                patch.set_alpha(emphasis_alpha)
                patch.set_linestyle("-")
            elif phase == "controlled":
                controlled_count += 1
                patch.set_edgecolor("#f97316")
                patch.set_facecolor("#fed7aa")
                patch.set_alpha(emphasis_alpha + 0.05)
                patch.set_linestyle("-")
            else:
                restricted_count += 1
                patch.set_edgecolor("#dc2626")
                patch.set_facecolor("#fecaca")
                patch.set_alpha(emphasis_alpha + 0.07)
                patch.set_linestyle("-")

        npc_points = []
        active_npc_count = 0
        for trace_index, trace in enumerate(traffic_traces):
            state = get_traffic_state(trace.get("segments", []), t)
            if state is None:
                npc_trails[trace_index].set_data([], [])
                continue

            active_npc_count += 1
            npc_x, npc_y, npc_alt, segment = state
            npc_points.append((npc_x, npc_y))

            trail_x, trail_y = get_recent_traffic_trail(segment, t, trail_steps=25)
            npc_trails[trace_index].set_data(trail_x, trail_y)
            npc_trails[trace_index].set_alpha(0.85 if npc_alt == ownship_alt else 0.35)

        if npc_points:
            npc_scatter.set_offsets(npc_points)
        else:
            npc_scatter.set_offsets([])

        status_text.set_text(
            f"time={t}\n"
            f"x={xs[idx]:.1f}, y={ys[idx]:.1f}\n"
            f"alt_layer={alt_layers[idx]}\n"
            f"energy={energies[idx]:.1f}\n"
            f"NOTAM A/C/R={advisory_count}/{controlled_count}/{restricted_count}\n"
            f"active_npc={active_npc_count}"
        )
        fig.canvas.draw_idle()

    def on_slider_change(value):
        render(value)

    def on_key(event):
        idx = int(slider.val)
        if event.key == "left":
            slider.set_val(max(0, idx - 1))
        elif event.key == "right":
            slider.set_val(min(len(playback) - 1, idx + 1))
        elif event.key == "home":
            slider.set_val(0)
        elif event.key == "end":
            slider.set_val(len(playback) - 1)

    slider.on_changed(on_slider_change)
    fig.canvas.mpl_connect("key_press_event", on_key)

    render(0)
    plt.show()


if __name__ == "__main__":
    main()
