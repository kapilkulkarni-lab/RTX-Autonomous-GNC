#!/usr/bin/env python3
"""
generate_avc_world.py

Procedurally builds a Gazebo Sim (Harmonic or newer) SDF world of the 2026-27
Raytheon AVC "Mission Bird Dog" field, plus a JSON manifest of ground truth
(obstacles, targets, start zone, spawn pose) for planner testing and for
mocking the perception team's output.

Frames: world is ENU, origin at the field centre, +X = east, +Y = north, +Z = up.
        PX4's local frame is NED with origin at the spawn point (see manifest).
Units:  metres.

Example:
    python3 generate_avc_world.py --challenge 2 --seed 7 --start-corner sw --textured-floor
"""
from __future__ import annotations

import argparse
import json
import math
import random
import struct
import sys
import zlib
from pathlib import Path

FT = 0.3048
YD = 3 * FT

# --- Rules-derived constants (2627 AVC Rules v0.3) ---------------------------
DIE_SIZE = 6 * 0.0254          # 2.7: 6-inch foam dice
DIE_MASS = 0.08                # foam, rough guess
DIE_COLORS = {
    "blue":   (0.05, 0.20, 0.85),
    "red":    (0.85, 0.08, 0.08),
    "yellow": (0.95, 0.85, 0.10),
    "black":  (0.03, 0.03, 0.03),
}
ZONE_SIZE = 5 * FT             # 2.8: 5-foot square start zone

CARDBOARD = (0.70, 0.54, 0.35)
ORANGE = (1.00, 0.40, 0.00)
WHITE = (0.95, 0.95, 0.95)
TAPE_BLUE = (0.20, 0.45, 0.95)
FLOOR_GREY = (0.55, 0.55, 0.55)
WALL_GREY = (0.80, 0.80, 0.78)

# Rough real-world sizes. Replace with measurements of the actual items.
OBSTACLE_TYPES = {
    "box_small":  {"shape": "box", "size": (0.30, 0.25, 0.25), "weight": 2},
    "box_medium": {"shape": "box", "size": (0.45, 0.35, 0.35), "weight": 2},
    "box_large":  {"shape": "box", "size": (0.60, 0.45, 0.50), "weight": 2},
    "cone":       {"shape": "cone", "base": 0.28, "height": 0.46, "weight": 3},
    "bucket":     {"shape": "bucket", "radius": 0.15, "height": 0.37, "weight": 2},
}


# --- Geometry helpers ---------------------------------------------------------
def footprint_radius(spec: dict) -> float:
    """Bounding-circle radius of an obstacle's footprint (conservative)."""
    if spec["shape"] == "box":
        w, d, _ = spec["size"]
        return 0.5 * math.hypot(w, d)
    if spec["shape"] == "cone":
        return spec["base"] * math.sqrt(2) / 2
    return spec["radius"]


def height_of(spec: dict) -> float:
    return spec["size"][2] if spec["shape"] == "box" else spec["height"]


def circle_rect_gap(x, y, r, rect) -> float:
    """Edge gap between a circle and an axis-aligned rect (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = rect
    dx = max(x0 - x, 0.0, x - x1)
    dy = max(y0 - y, 0.0, y - y1)
    return math.hypot(dx, dy) - r


# --- SDF snippet helpers -------------------------------------------------------
def rgba(c, a=1.0) -> str:
    return f"{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a}"


def material(c) -> str:
    return (f"<material><ambient>{rgba(c)}</ambient><diffuse>{rgba(c)}</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0) -> str:
    return f"<pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>"


def box_geom(w, d, h) -> str:
    return f"<geometry><box><size>{w:.4f} {d:.4f} {h:.4f}</size></box></geometry>"


def cyl_geom(r, h) -> str:
    return f"<geometry><cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder></geometry>"


def obstacle_model(name, kind, x, y, yaw) -> str:
    spec = OBSTACLE_TYPES[kind]
    parts = []
    if spec["shape"] == "box":
        w, d, h = spec["size"]
        parts.append(f"<collision name='collision'>{pose(0, 0, h/2)}{box_geom(w, d, h)}</collision>")
        parts.append(f"<visual name='visual'>{pose(0, 0, h/2)}{box_geom(w, d, h)}{material(CARDBOARD)}</visual>")
    elif spec["shape"] == "cone":
        b, h = spec["base"], spec["height"]
        base_h = 0.03
        # Conservative collision: cylinder of the base half-width over full height.
        parts.append(f"<collision name='collision'>{pose(0, 0, h/2)}{cyl_geom(b/2, h)}</collision>")
        parts.append(f"<visual name='base'>{pose(0, 0, base_h/2)}{box_geom(b, b, base_h)}{material(ORANGE)}</visual>")
        n_seg = 4
        seg_h = (h - base_h) / n_seg
        for i in range(n_seg):
            r = 0.12 - i * (0.12 - 0.03) / (n_seg - 1)
            z = base_h + seg_h * (i + 0.5)
            col = WHITE if i == 2 else ORANGE
            parts.append(f"<visual name='seg{i}'>{pose(0, 0, z)}{cyl_geom(r, seg_h)}{material(col)}</visual>")
    else:  # bucket
        r, h = spec["radius"], spec["height"]
        parts.append(f"<collision name='collision'>{pose(0, 0, h/2)}{cyl_geom(r, h)}</collision>")
        parts.append(f"<visual name='body'>{pose(0, 0, h/2)}{cyl_geom(r, h)}{material(ORANGE)}</visual>")
        parts.append(f"<visual name='rim'>{pose(0, 0, h - 0.01)}{cyl_geom(r + 0.01, 0.02)}{material(WHITE)}</visual>")
    return (f"<model name='{name}'><static>true</static>{pose(x, y, 0, yaw)}"
            f"<link name='link'>{''.join(parts)}</link></model>")


def die_model(color, x, y, yaw) -> str:
    s, m = DIE_SIZE, DIE_MASS
    inertia = m * s * s / 6
    return (
        f"<model name='target_{color}'>{pose(x, y, s/2 + 0.001, yaw)}"
        f"<link name='link'><inertial><mass>{m}</mass><inertia>"
        f"<ixx>{inertia:.6f}</ixx><iyy>{inertia:.6f}</iyy><izz>{inertia:.6f}</izz>"
        f"<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f"<collision name='collision'>{box_geom(s, s, s)}</collision>"
        f"<visual name='visual'>{box_geom(s, s, s)}{material(DIE_COLORS[color])}</visual>"
        f"</link></model>"
    )


def tape_rect(name, rect, width, color) -> str:
    """Painter's-tape outline (visual only, no collision) centred on the rect edges."""
    x0, y0, x1, y1 = rect
    z, t = 0.002, 0.001
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    lx, ly = x1 - x0, y1 - y0
    segs = [
        (cx, y0, lx + width, width), (cx, y1, lx + width, width),
        (x0, cy, width, ly + width), (x1, cy, width, ly + width),
    ]
    vis = "".join(
        f"<visual name='s{i}'>{pose(x, y, z)}{box_geom(w, d, t)}{material(color)}</visual>"
        for i, (x, y, w, d) in enumerate(segs)
    )
    return f"<model name='{name}'><static>true</static><link name='link'>{vis}</link></model>"


def walls_model(half_w, half_l, margin, height=2.5, thick=0.2) -> str:
    xw, yw = half_w + margin, half_l + margin
    segs = [
        (0, yw + thick/2, 2*xw + 2*thick, thick), (0, -yw - thick/2, 2*xw + 2*thick, thick),
        (xw + thick/2, 0, thick, 2*yw), (-xw - thick/2, 0, thick, 2*yw),
    ]
    parts = []
    for i, (x, y, w, d) in enumerate(segs):
        g = box_geom(w, d, height)
        parts.append(f"<collision name='c{i}'>{pose(x, y, height/2)}{g}</collision>"
                     f"<visual name='v{i}'>{pose(x, y, height/2)}{g}{material(WALL_GREY)}</visual>")
    return f"<model name='venue_walls'><static>true</static><link name='link'>{''.join(parts)}</link></model>"


def ground_model(field_w, field_l, texture_path: Path | None) -> str:
    field_vis = ""
    if texture_path is not None:
        field_vis = (
            f"<visual name='field_texture'>{pose(0, 0, 0.0005)}"
            f"<geometry><plane><normal>0 0 1</normal><size>{field_w:.4f} {field_l:.4f}</size></plane></geometry>"
            f"<material><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>"
            f"<pbr><metal><albedo_map>{texture_path.resolve()}</albedo_map>"
            f"<roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material></visual>"
        )
    return (
        "<model name='ground_plane'><static>true</static><link name='link'>"
        "<collision name='collision'><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>"
        "<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>"
        "<visual name='visual'><geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>"
        f"{material(FLOOR_GREY)}</visual>{field_vis}</link></model>"
    )


WORLD_SYSTEMS = """
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-air-speed-system" name="gz::sim::systems::AirSpeed"/>
    <plugin filename="gz-sim-apply-link-wrench-system" name="gz::sim::systems::ApplyLinkWrench"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>"""


# --- Texture (stdlib-only PNG writer) ------------------------------------------
def write_noise_png(path: Path, seed: int, size: int = 1024, block: int = 2) -> None:
    """Grey speckle texture so optical flow / VIO have features to track."""
    rng = random.Random(seed)
    n = size // block
    vals = [bytes(rng.randint(70, 190) for _ in range(n)) for _ in range(n)]
    raw = bytearray()
    for y in range(size):
        raw.append(0)  # filter type: none
        row = vals[y // block]
        raw.extend(row[x // block] for x in range(size))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 0, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 9))
           + chunk(b"IEND", b""))
    path.write_bytes(png)


# --- Placement ------------------------------------------------------------------
def place_items(rng, count, radius_fn, field_rect, keepouts, placed, min_gap, edge_margin, max_tries=20000):
    """Rejection-sample circles inside field_rect.

    placed: list of (x, y, r) already on the field; new items respect min_gap to them.
    keepouts: list of (rect, clearance) that items must stay clear of.
    radius_fn: callable returning (label, r) for the next item.
    Returns list of (label, x, y, r).
    """
    x0, y0, x1, y1 = field_rect
    out = []
    tries = 0
    while len(out) < count and tries < max_tries:
        tries += 1
        label, r = radius_fn(len(out))
        x = rng.uniform(x0 + r + edge_margin, x1 - r - edge_margin)
        y = rng.uniform(y0 + r + edge_margin, y1 - r - edge_margin)
        if any(circle_rect_gap(x, y, r, rect) < clr for rect, clr in keepouts):
            continue
        if any(math.hypot(x - px, y - py) - r - pr < min_gap for px, py, pr in placed):
            continue
        out.append((label, x, y, r))
        placed.append((x, y, r))
    return out


# --- Main -----------------------------------------------------------------------
def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out-dir", default="worlds", help="output directory")
    p.add_argument("--world-name", default="avc_field", help="SDF world name (must match PX4_GZ_WORLD)")
    p.add_argument("--seed", type=int, default=None, help="RNG seed (random if omitted)")
    p.add_argument("--challenge", type=int, choices=(1, 2, 3), default=2)
    p.add_argument("--num-obstacles", type=int, default=None, help="default: 0 for C1, 12 for C2/C3")
    p.add_argument("--min-gap-ft", type=float, default=5.0, help="edge-to-edge obstacle spacing (rules: 5 ft)")
    p.add_argument("--field-width-yd", type=float, default=15.0, help="east-west size")
    p.add_argument("--field-length-yd", type=float, default=15.0, help="north-south size")
    p.add_argument("--start-corner", choices=("sw", "se", "nw", "ne"), default=None, help="random if omitted")
    p.add_argument("--target-clearance", type=float, default=1.0, help="min gap (m) from dice to obstacles/each other")
    p.add_argument("--zone-clearance", type=float, default=1.0, help="min gap (m) from anything to the start zone")
    p.add_argument("--no-walls", action="store_true", help="omit venue walls")
    p.add_argument("--wall-margin", type=float, default=3.0, help="distance (m) from field edge to venue walls")
    p.add_argument("--textured-floor", action="store_true", help="speckle floor texture for optical flow / VIO")
    p.add_argument("--lat", type=float, default=32.2226)
    p.add_argument("--lon", type=float, default=-110.9747)
    p.add_argument("--alt", type=float, default=730.0)
    args = p.parse_args()

    seed = args.seed if args.seed is not None else random.randrange(1_000_000)
    rng = random.Random(seed)

    field_w, field_l = args.field_width_yd * YD, args.field_length_yd * YD
    hw, hl = field_w / 2, field_l / 2
    field_rect = (-hw, -hl, hw, hl)

    corner = args.start_corner or rng.choice(["sw", "se", "nw", "ne"])
    zx0 = -hw if "w" in corner else hw - ZONE_SIZE
    zy0 = -hl if "s" in corner else hl - ZONE_SIZE
    zone = (zx0, zy0, zx0 + ZONE_SIZE, zy0 + ZONE_SIZE)
    zcx, zcy = zx0 + ZONE_SIZE / 2, zy0 + ZONE_SIZE / 2
    spawn_yaw = math.atan2(-zcy, -zcx)  # face the field centre

    n_obs = args.num_obstacles if args.num_obstacles is not None else (0 if args.challenge == 1 else 12)
    kinds = list(OBSTACLE_TYPES)
    weights = [OBSTACLE_TYPES[k]["weight"] for k in kinds]
    obs_kinds = rng.choices(kinds, weights=weights, k=n_obs)

    placed: list[tuple[float, float, float]] = []
    obstacles = place_items(
        rng, n_obs,
        lambda i: (obs_kinds[i], footprint_radius(OBSTACLE_TYPES[obs_kinds[i]])),
        field_rect, [(zone, args.zone_clearance)], placed,
        min_gap=args.min_gap_ft * FT, edge_margin=0.3,
    )
    if len(obstacles) < n_obs:
        print(f"warning: only placed {len(obstacles)}/{n_obs} obstacles; field too crowded for "
              f"{args.min_gap_ft} ft spacing", file=sys.stderr)

    colors = list(DIE_COLORS)
    rng.shuffle(colors)
    die_r = DIE_SIZE * math.sqrt(2) / 2
    dice = place_items(
        rng, len(colors), lambda i: (colors[i], die_r),
        field_rect, [(zone, args.zone_clearance + 0.5)], placed,
        min_gap=args.target_clearance, edge_margin=0.3,
    )
    if len(dice) < len(colors):
        print("error: could not place all target dice; loosen clearances", file=sys.stderr)
        return 1

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tex = None
    if args.textured_floor:
        tex = out / f"{args.world_name}_floor.png"
        write_noise_png(tex, seed)

    models = [ground_model(field_w, field_l, tex),
              tape_rect("field_boundary", field_rect, 0.05, WHITE),
              tape_rect("start_zone", zone, 0.05, TAPE_BLUE)]
    if not args.no_walls:
        models.append(walls_model(hw, hl, args.wall_margin))

    obs_manifest = []
    for i, (kind, x, y, r) in enumerate(obstacles):
        yaw = rng.uniform(-math.pi, math.pi)
        name = f"obs_{i:02d}_{kind}"
        models.append(obstacle_model(name, kind, x, y, yaw))
        obs_manifest.append({"name": name, "kind": kind, "x": round(x, 4), "y": round(y, 4),
                             "yaw": round(yaw, 4), "footprint_radius": round(r, 4),
                             "height": height_of(OBSTACLE_TYPES[kind])})

    dice_manifest = []
    for color, x, y, _ in dice:
        yaw = rng.uniform(-math.pi, math.pi)
        models.append(die_model(color, x, y, yaw))
        dice_manifest.append({"name": f"target_{color}", "color": color,
                              "x": round(x, 4), "y": round(y, 4), "yaw": round(yaw, 4)})

    scenario = {"initial_target": rng.choice(colors)}
    if args.challenge == 3:
        scenario["changed_target"] = rng.choice([c for c in colors if c != scenario["initial_target"]])
        scenario["change_after_s"] = round(rng.uniform(30, 150), 1)

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="{args.world_name}">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>{WORLD_SYSTEMS}
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.85 0.85 0.85 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{args.lat}</latitude_deg>
      <longitude_deg>{args.lon}</longitude_deg>
      <elevation>{args.alt}</elevation>
    </spherical_coordinates>
    <light type="directional" name="ceiling_key">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    {chr(10).join('    ' + m for m in models)}
  </world>
</sdf>
"""
    sdf_path = out / f"{args.world_name}.sdf"
    sdf_path.write_text(sdf)

    manifest = {
        "world_name": args.world_name,
        "seed": seed,
        "challenge": args.challenge,
        "frame": "ENU, origin at field centre, +x east, +y north, metres",
        "field": {"width_m": round(field_w, 4), "length_m": round(field_l, 4),
                  "bounds": [round(v, 4) for v in field_rect]},
        "start_zone": {"corner": corner, "bounds": [round(v, 4) for v in zone],
                       "center": [round(zcx, 4), round(zcy, 4)]},
        "px4_spawn_pose": f"{zcx:.3f},{zcy:.3f},0.2,0,0,{spawn_yaw:.3f}",
        "px4_frame_note": ("PX4 local NED origin = spawn point. world(x_e, y_n) -> "
                           "local NED: north = y_n - spawn_y, east = x_e - spawn_x, down = -z"),
        "min_obstacle_gap_m": round(args.min_gap_ft * FT, 4),
        "obstacles": obs_manifest,
        "targets": dice_manifest,
        "scenario": scenario,
    }
    man_path = out / f"{args.world_name}_manifest.json"
    man_path.write_text(json.dumps(manifest, indent=2))

    print(f"seed={seed} corner={corner} obstacles={len(obstacles)} "
          f"scenario={scenario}")
    print(f"wrote {sdf_path}")
    print(f"wrote {man_path}")
    if tex:
        print(f"wrote {tex}")
    print(f'PX4_GZ_MODEL_POSE="{manifest["px4_spawn_pose"]}"')
    return 0


if __name__ == "__main__":
    sys.exit(main())
