from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass
class Rect:
    x: float
    y: float
    w: float
    h: float

    def contains(self, px, py, margin=0.0):
        return (self.x - margin <= px <= self.x + self.w + margin) and (self.y - margin <= py <= self.y + self.h + margin)


@dataclass
class Layout:
    width: float
    height: float
    walls: list = field(default_factory=list)
    furniture: list = field(default_factory=list)
    name: str = "office"

    @property
    def obstacles(self):
        return self.walls + self.furniture


def office_layout():
    W, H, t = 16.0, 12.0, 0.15
    walls = [
        Rect(0, 0, W, t), Rect(0, H - t, W, t), Rect(0, 0, t, H), Rect(W - t, 0, t, H),
        Rect(5.0, 0, t, 3.5), Rect(5.0, 4.6, t, 2.4),
        Rect(0, 7.0, 2.2, t), Rect(3.3, 7.0, 4.0, t),
        Rect(7.3, 7.0, t, 2.0), Rect(7.3, 10.1, t, 1.9),
        Rect(10.5, 0, t, 2.0), Rect(10.5, 3.1, t, 4.6),
        Rect(10.5, 7.6, 2.0, t), Rect(13.6, 7.6, 2.4, t),
    ]
    furniture = [
        Rect(1.2, 1.5, 1.6, 0.8), Rect(1.2, 4.2, 0.6, 0.6), Rect(3.2, 5.4, 0.8, 0.8),
        Rect(7.0, 2.0, 1.2, 1.2), Rect(8.8, 4.8, 0.4, 0.4), Rect(6.4, 5.4, 0.4, 0.4),
        Rect(12.0, 1.0, 2.5, 0.8), Rect(12.3, 4.6, 0.8, 1.6),
        Rect(1.0, 9.2, 2.0, 0.9), Rect(4.8, 9.6, 0.5, 0.5),
        Rect(9.0, 10.4, 1.4, 0.7), Rect(13.0, 9.6, 1.2, 1.2),
    ]
    return Layout(W, H, walls, furniture)


class GridMap:
    def __init__(self, occupied, resolution, origin=(0.0, 0.0)):
        self.occupied = np.asarray(occupied, dtype=bool)
        self.resolution = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.ny, self.nx = self.occupied.shape
        self._distance = None

    @classmethod
    def from_layout(cls, layout, resolution=0.05, extra=()):
        nx = int(round(layout.width / resolution))
        ny = int(round(layout.height / resolution))
        occ = np.zeros((ny, nx), dtype=bool)
        for r in list(layout.obstacles) + list(extra):
            x0 = int(np.floor(r.x / resolution))
            x1 = int(np.ceil((r.x + r.w) / resolution))
            y0 = int(np.floor(r.y / resolution))
            y1 = int(np.ceil((r.y + r.h) / resolution))
            occ[max(y0, 0):min(y1, ny), max(x0, 0):min(x1, nx)] = True
        return cls(occ, resolution)

    def to_cell(self, x, y):
        ix = np.floor((np.asarray(x) - self.origin[0]) / self.resolution).astype(int)
        iy = np.floor((np.asarray(y) - self.origin[1]) / self.resolution).astype(int)
        return ix, iy

    def to_world(self, ix, iy):
        return self.origin[0] + (np.asarray(ix) + 0.5) * self.resolution, self.origin[1] + (np.asarray(iy) + 0.5) * self.resolution

    def inside(self, ix, iy):
        return (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)

    def is_occupied(self, x, y):
        ix, iy = self.to_cell(x, y)
        ok = self.inside(ix, iy)
        out = np.ones(np.shape(ix), dtype=bool)
        out[ok] = self.occupied[iy[ok], ix[ok]]
        return out

    def distance_field(self):
        if self._distance is None:
            from scipy import ndimage

            self._distance = ndimage.distance_transform_edt(~self.occupied) * self.resolution
        return self._distance

    def collides(self, x, y, radius):
        ix, iy = self.to_cell(x, y)
        if not self.inside(ix, iy):
            return True
        return bool(self.distance_field()[iy, ix] < radius)

    def save_ros_map(self, stem):
        from PIL import Image

        stem = Path(stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        image = np.where(self.occupied, 0, 254).astype(np.uint8)[::-1]
        Image.fromarray(image).save(stem.with_suffix(".pgm"))
        stem.with_suffix(".yaml").write_text(
            f"image: {stem.with_suffix('.pgm').name}\nmode: trinary\nresolution: {self.resolution}\n"
            f"origin: [{self.origin[0]}, {self.origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
            encoding="utf-8",
        )


def free_points(grid, rng, n, clearance=0.5, region=None):
    dist = grid.distance_field()
    iy, ix = np.nonzero(dist >= clearance)
    xs, ys = grid.to_world(ix, iy)
    if region is not None:
        keep = np.array([any(r.contains(x, y) for r in region) for x, y in zip(xs, ys)])
        xs, ys = xs[keep], ys[keep]
    idx = rng.choice(len(xs), size=n, replace=len(xs) < n)
    return np.stack([xs[idx], ys[idx]], axis=1)


def gazebo_world_sdf(layout, wall_height=1.0, furniture_height=0.75, name="p3at_office"):
    def box(i, r, h, colour):
        cx, cy = r.x + r.w / 2, r.y + r.h / 2
        return f"""    <model name="obstacle_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {h / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{r.w:.3f} {r.h:.3f} {h:.3f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{r.w:.3f} {r.h:.3f} {h:.3f}</size></box></geometry>
          <material><ambient>{colour}</ambient><diffuse>{colour}</diffuse></material></visual>
      </link>
    </model>
"""

    models = "".join(box(i, r, wall_height, "0.8 0.8 0.78 1") for i, r in enumerate(layout.walls))
    models += "".join(box(len(layout.walls) + i, r, furniture_height, "0.55 0.42 0.3 1") for i, r in enumerate(layout.furniture))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link>
    </model>
{models}  </world>
</sdf>
"""
