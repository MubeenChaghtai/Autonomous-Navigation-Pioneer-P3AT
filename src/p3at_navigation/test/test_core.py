import numpy as np
import pytest

from p3at_navigation.amcl import ParticleFilter
from p3at_navigation.planning import PurePursuit, costmap, plan
from p3at_navigation.sensors import LidarSpec, OdometryNoise, apply_motion, corrupt_odometry, raycast, relative_motion
from p3at_navigation.simulation import NOISE_PROFILES, PLANNING_RADIUS, ate, build_maps, drive_tour, map_quality, run_navigation
from p3at_navigation.slam import OccupancyMapper, ScanMatcher
from p3at_navigation.world import GridMap, Layout, Rect, office_layout


@pytest.fixture(scope="module")
def grid():
    return GridMap.from_layout(office_layout())


def box_room():
    t = 0.1
    return Layout(6.0, 4.0, [Rect(0, 0, 6, t), Rect(0, 4 - t, 6, t), Rect(0, 0, t, 4), Rect(6 - t, 0, t, 4)], [Rect(2.0, 1.5, 0.5, 1.0)])


def test_raycast_measures_distance_to_wall():
    g = GridMap.from_layout(box_room())
    spec = LidarSpec(beams=3, fov=np.pi / 2, mount_x=0.0)
    r = raycast(g, (4.0, 2.0, 0.0), spec)
    assert r[1] == pytest.approx(5.9 - 4.0, abs=0.06)


def test_ros_map_export(tmp_path, grid):
    grid.save_ros_map(tmp_path / "office")
    from PIL import Image

    image = np.asarray(Image.open(tmp_path / "office.pgm"))
    assert image.shape == grid.occupied.shape
    assert np.array_equal(image[::-1] == 0, grid.occupied)
    assert "resolution: 0.05" in (tmp_path / "office.yaml").read_text()


def test_motion_decomposition_round_trip():
    p0 = np.array([1.0, 2.0, 0.3])
    p1 = np.array([1.8, 2.5, -0.4])
    assert np.allclose(apply_motion(p0, *relative_motion(p0, p1)), p1)


def test_odometry_without_noise_is_exact():
    rng = np.random.default_rng(0)
    p0, p1 = np.array([0.0, 0.0, 0.0]), np.array([0.5, 0.1, 0.2])
    assert np.allclose(corrupt_odometry(p0, p0, p1, OdometryNoise(0, 0, 0, 0, 0), rng), p1)


def test_mapper_marks_walls_and_free_space():
    g = GridMap.from_layout(box_room())
    spec = LidarSpec(sigma=0.0)
    m = OccupancyMapper(8, 6, 0.05, (-1, -1))
    for _ in range(3):
        m.integrate((4.0, 2.0, np.pi), raycast(g, (4.0, 2.0, np.pi), spec), spec)
    ix, iy, _ = m.cells(np.array([2.47]), np.array([2.0]))
    assert m.occupied()[iy[0], ix[0] - 1:ix[0] + 2].any()
    jx, jy, _ = m.cells(np.array([3.3]), np.array([2.0]))
    assert m.logodds[jy[0], jx[0]] < 0


def test_scan_matcher_recovers_offset(grid):
    spec = LidarSpec(sigma=0.0)
    true_pose = np.array([3.0, 3.0, 0.4])
    m = OccupancyMapper(18, 14, 0.05, (-1, -1))
    for _ in range(3):
        m.integrate(true_pose, raycast(grid, true_pose, spec), spec)
    guess = true_pose + np.array([0.12, -0.08, np.radians(3.0)])
    matched, score = ScanMatcher().match(m, guess, raycast(grid, true_pose, spec), spec)
    assert np.hypot(*(matched[:2] - true_pose[:2])) < 0.03
    assert abs(matched[2] - true_pose[2]) < np.radians(0.6)
    assert score > 0.5


def test_particle_filter_tracks_tour(grid):
    rng = np.random.default_rng(1)
    truth, odoms, scans, spec = drive_tour(grid, [(2.5, 3.0, 0.0), (8.5, 1.0), (9.5, 4.0)], NOISE_PROFILES["moderate"], rng, scan_every=4)
    pf = ParticleFilter(grid, spec, 300, rng)
    pf.init_pose(truth[0], 0.15, 0.08)
    pf.last_odom = odoms[0].copy()
    errors = [np.hypot(*(pf.update(o, z)[0][:2] - t[:2])) for t, o, z in zip(truth, odoms, scans)]
    assert np.mean(errors) < 0.15 and errors[-1] < 0.25


def test_slam_beats_odometry_under_noise(grid):
    rng = np.random.default_rng(2)
    truth, odoms, scans, spec = drive_tour(grid, [(2.5, 3.0, 0.0), (8.5, 1.0), (9.5, 4.0), (14.0, 3.2)], NOISE_PROFILES["high"], rng, scan_every=4)
    reference, dead, slam_map, est, _ = build_maps(grid, truth, odoms, scans, spec)
    assert ate(est, truth) < ate(odoms, truth)
    assert map_quality(slam_map, reference, grid)["f1"] > map_quality(dead, reference, grid)["f1"]


def test_planner_path_is_collision_free(grid):
    cost = costmap(grid.occupied, grid.resolution, PLANNING_RADIUS)
    path = plan(grid, cost, (2.5, 3.0), (11.5, 10.5))
    assert path is not None
    assert all(not grid.collides(x, y, 0.3) for x, y in path)


def test_pure_pursuit_stops_at_goal():
    controller = PurePursuit()
    path = np.stack([np.linspace(0, 3, 61), np.zeros(61)], axis=1)
    assert controller.command((2.9, 0.0, 0.0), path, 5.0)[2]
    v, w, done = controller.command((0.0, 0.0, 0.0), path, 5.0)
    assert v > 0 and not done
    assert controller.command((0.0, 0.0, 0.0), path, 0.3)[0] == 0.0


def test_navigation_reaches_goal_with_unmapped_obstacle(grid):
    truth = GridMap.from_layout(office_layout(), extra=[Rect(6.0, 3.6, 0.5, 0.5)])
    result = run_navigation(grid, truth, (2.5, 3.0, 0.0), (8.5, 1.0), NOISE_PROFILES["moderate"], np.random.default_rng(3))
    assert result["success"], result
