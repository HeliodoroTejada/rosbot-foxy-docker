import math

import numpy as np
import pytest

from rosbot_exploration.frontier_search import Frontier, Grid, clearance_mask, search
from rosbot_exploration.goal_selection import GoalMemory
from rosbot_exploration.viewpoints import footprint_is_free, turn_cost, utility_cost, visible_unknown_area


def target(x=2.0, y=2.0):
    return Frontier(np.asarray([[20, 20]]), x, y, 0.0, 1.0, 1.0, 0.0,
                    region_x=2.5, region_y=2.0)


def test_mapped_wall_occludes_unknown_information():
    data = np.zeros((80, 80), dtype=np.int16)
    data[:, 35:] = -1
    data[:, 30] = 100
    grid = Grid(data, 0.1)
    assert visible_unknown_area(grid, 2.0, 4.0) == 0.0
    data[:, 30] = 0
    assert visible_unknown_area(grid, 2.0, 4.0) > 1.0


def test_information_area_uses_metric_units_at_different_resolutions():
    estimates = []
    for resolution in (0.05, 0.1):
        cells = int(round(8 / resolution))
        data = np.zeros((cells, cells), dtype=np.int16)
        data[:, int(round(3 / resolution)):] = -1
        estimates.append(visible_unknown_area(Grid(data, resolution), 2.0, 4.0, radius=2.0))
    assert estimates[0] > 1.0
    assert abs(estimates[0] - estimates[1]) / max(estimates) < 0.1


def test_gain_rewards_productive_viewpoint_and_turn_cost_prefers_forward_motion():
    forward = turn_cost((0, 0), 0.0, 1.0, 0.0, 0.0)
    behind = turn_cost((0, 0), 0.0, -1.0, 0.0, math.pi)
    assert utility_cost(1.0, 5.0, forward) < utility_cost(1.0, 5.0, behind)
    assert utility_cost(2.0, 20.0) < utility_cost(1.0, 1.0)
    assert utility_cost(1.0, 5.0, revisit=1.0) > utility_cost(1.0, 5.0, revisit=0.0)


def test_oriented_rectangle_fits_narrow_corridor_where_circle_does_not():
    data = np.full((44, 36), 100, dtype=np.int16)
    data[2:36, 12:21] = 0  # 0.45 m corridor
    data[36:42, 12:21] = -1
    grid = Grid(data, 0.05)
    assert footprint_is_free(grid, 0.825, 0.725, math.pi / 2)
    assert not clearance_mask(data == 0, 0.05, 0.25)[grid.cell(0.825, 0.725)]
    with pytest.raises(ValueError):
        search(grid, (0.825, 0.725))
    result = search(grid, (0.825, 0.725), use_rectangular_footprint=True)
    assert result.frontiers
    assert all(footprint_is_free(grid, f.x, f.y, f.yaw) for f in result.frontiers)


def test_rectangle_rotation_and_interior_obstacles_are_checked():
    data = np.full((40, 40), 100, dtype=np.int16)
    data[2:38, 16:24] = 0
    grid = Grid(data, 0.05)
    assert footprint_is_free(grid, 1.0, 1.0, math.pi / 2, length=0.6, width=0.24, padding=0.01)
    assert not footprint_is_free(grid, 1.0, 1.0, 0.0, length=0.6, width=0.24, padding=0.01)
    data[20, 20] = 100
    assert not footprint_is_free(grid, 1.0, 1.0, math.pi / 2, length=0.6, width=0.24, padding=0.01)


def test_rectangle_geometry_respects_rotated_map_origin():
    data = np.full((40, 40), 100, dtype=np.int16)
    data[2:38, 16:24] = 0
    grid = Grid(data, 0.05, -3.0, 1.0, 0.7)
    x, y = grid.world(20, 20)
    assert footprint_is_free(grid, x, y, 0.7 + math.pi / 2, length=0.6, width=0.24, padding=0.01)
    assert not footprint_is_free(grid, x, y, 0.7, length=0.6, width=0.24, padding=0.01)


def test_failed_frontier_is_reactivated_when_obstruction_is_removed():
    data = np.zeros((50, 50), dtype=np.int16)
    data[10:31, 25] = 100
    grid = Grid(data, 0.1)
    memory = GoalMemory(max_attempts=1)
    viewpoint = target()
    memory.record(viewpoint, 0.0, grid=grid)
    assert not memory.allowed(viewpoint, 1.0)
    assert memory.update_scene(grid, None, 2.0) == 0
    cleared = Grid(data.copy(), 0.1)
    cleared.data[10:31, 25] = 0
    assert memory.update_scene(cleared, None, 3.0) == 1
    assert memory.allowed(viewpoint, 3.0)


def test_map_resize_without_scene_change_does_not_clear_exclusion():
    data = np.zeros((50, 50), dtype=np.int16)
    data[10:31, 25] = 100
    grid = Grid(data, 0.1)
    memory = GoalMemory(max_attempts=1)
    viewpoint = target()
    memory.record(viewpoint, 0.0, grid=grid)
    expanded = Grid(np.pad(data, 5, constant_values=-1), 0.1, -0.5, -0.5)
    assert memory.update_scene(expanded, None, 1.0) == 0
    assert not memory.allowed(viewpoint, 1.0)


def test_small_wall_shift_does_not_reactivate_failed_frontier():
    data = np.zeros((50, 50), dtype=np.int16)
    data[10:31, 25] = 100
    memory = GoalMemory(max_attempts=1)
    viewpoint = target()
    memory.record(viewpoint, 0.0, grid=Grid(data, 0.1))
    shifted = np.zeros_like(data)
    shifted[10:31, 26] = 100
    assert memory.update_scene(Grid(shifted, 0.1), None, 1.0) == 0
    assert not memory.allowed(viewpoint, 1.0)


def test_failure_excludes_other_viewpoints_of_same_frontier_region():
    memory = GoalMemory(max_attempts=1)
    first = target()
    memory.record(first, 0.0)
    alternative = target(1.8, 1.2)
    alternative.region_y = 2.4
    assert math.hypot(first.x - alternative.x, first.y - alternative.y) > memory.radius
    assert not memory.allowed(alternative, 1.0)
    assert memory.frontier_exclusions(1.0) == [(2.5, 2.0, 0.75)]


def test_exclusions_expire_and_recent_trail_decays():
    memory = GoalMemory(max_attempts=1, blocked_retry_delay=20.0, visit_cooldown=15.0)
    viewpoint = target()
    memory.record(viewpoint, 0.0)
    assert not memory.allowed(viewpoint, 19.0)
    assert memory.allowed(viewpoint, 20.0)
    memory.record(viewpoint, 21.0, success=True)
    assert not memory.allowed(viewpoint, 30.0)
    assert memory.allowed(viewpoint, 37.0)
    memory.observe_pose(1.0, 1.0, 0.0)
    memory.observe_pose(1.01, 1.0, 1.0)
    assert len(memory.trail) == 1
    assert memory.recent_positions(0.0)[0][2] > memory.recent_positions(90.0)[0][2]
    assert memory.recent_positions(300.0) == []
