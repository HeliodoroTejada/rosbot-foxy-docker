import math

import numpy as np

from rosbot_exploration.frontier_search import Grid, clearance_mask, path_is_safe, search
from rosbot_exploration.goal_selection import CompletionGate, GoalMemory


def open_map(resolution=0.1):
    cells = int(round(5 / resolution))
    data = np.full((cells, cells), -1, dtype=np.int16)
    pad = int(round(0.5 / resolution))
    data[pad:-pad, pad:-pad] = 0
    return Grid(data, resolution)


def test_frontiers_choose_known_free_viewpoints_with_footprint_clearance():
    grid = open_map()
    result = search(grid, (2.5, 2.5))
    assert result.count > 0 and result.frontiers
    for target in result.frontiers:
        row, col = grid.cell(target.x, target.y)
        assert result.safe[row, col]
        assert grid.data[row, col] == 0
        assert math.isfinite(target.yaw)
        assert math.hypot(target.x - 2.5, target.y - 2.5) >= 0.5


def test_excluding_a_viewpoint_selects_an_alternative_on_the_same_frontier():
    grid = open_map()
    first = search(grid, (2.5, 2.5)).frontiers[0]
    result = search(grid, (2.5, 2.5), excluded_positions=[(first.x, first.y, 0.5)])
    assert result.frontiers
    second = result.frontiers[0]
    assert math.hypot(first.x - second.x, first.y - second.y) > 0.5


def test_nearby_frontier_does_not_produce_an_already_reached_goal():
    grid = open_map()
    robot = (2.5, 0.95)
    result = search(grid, robot)
    assert result.frontiers
    assert math.hypot(result.frontiers[0].x - robot[0],
                      result.frontiers[0].y - robot[1]) >= 0.5


def test_costmap_obstacle_selects_another_viewpoint_instead_of_blocking_the_frontier():
    grid = open_map()
    first = search(grid, (2.5, 2.5)).frontiers[0]
    costs = grid.data.copy()
    row, col = grid.cell(first.x, first.y)
    costs[max(0, row - 3):row + 4, max(0, col - 3):col + 4] = 100
    result = search(grid, (2.5, 2.5), costmap=Grid(costs, 0.1))
    assert result.frontiers
    second = result.frontiers[0]
    assert costs[grid.cell(second.x, second.y)] == 0
    assert (first.x, first.y) != (second.x, second.y)


def test_disconnected_frontiers_behind_wall_are_excluded():
    data = np.full((40, 60), 100, dtype=np.int16)
    data[5:35, 5:25] = 0  # enclosed room containing robot
    data[5:35, 35:55] = 0
    data[5:35, 55:] = -1  # frontier in disconnected room only
    result = search(Grid(data, 0.1), (1.5, 2.0))
    assert result.count == 0
    assert result.frontiers == []


def test_no_corner_cutting_through_occupied_cells():
    free = np.ones((20, 20), dtype=bool)
    free[10, 10] = False
    safe = clearance_mask(free, 0.1, 0.25)
    assert not safe[8, 8]
    assert safe[6, 6]
    assert not path_is_safe([(0.65, 0.65), (1.45, 1.45)], Grid(free.astype(int), 0.1), safe)


def test_sparse_path_crossing_unknown_is_rejected():
    data = np.zeros((20, 30), dtype=np.int16)
    data[:, 14:16] = -1
    grid = Grid(data, 0.1)
    safe = clearance_mask(data == 0, 0.1, 0.2)
    assert not path_is_safe([(0.65, 1.05), (2.35, 1.05)], grid, safe)
    assert path_is_safe([(0.65, 0.65), (0.65, 1.35)], grid, safe)


def test_path_touching_an_occupied_corner_is_rejected():
    safe = np.ones((10, 10), dtype=bool)
    safe[4, 5] = False
    grid = Grid(np.zeros((10, 10)), 1.0)
    assert not path_is_safe([(4.5, 4.5), (5.5, 5.5)], grid, safe)
    assert path_is_safe([(4.5, 4.5), (3.5, 3.5)], grid, safe)


def test_world_coordinates_follow_translated_rotated_origin():
    grid = open_map()
    grid.origin_x, grid.origin_y, grid.origin_yaw = -12.3, 4.6, 0.7
    for cell in ((10, 11), (0, 0), (49, 49)):
        assert grid.cell(*grid.world(*cell)) == cell
    robot = grid.world(25, 25)
    result = search(grid, robot)
    assert result.frontiers
    target = result.frontiers[0]
    assert result.safe[grid.cell(target.x, target.y)]


def test_frontier_threshold_is_in_metres_at_both_workspace_resolutions():
    for resolution in (0.05, 0.06):
        result = search(open_map(resolution), (2.5, 2.5), min_frontier_size=0.5)
        assert result.frontiers
        assert result.frontiers[0].size > 10


def test_map_growth_changes_indices_without_changing_world_goal_memory():
    target = search(open_map(), (2.5, 2.5)).frontiers[0]
    memory = GoalMemory(max_attempts=2, retry_delay=2.0)
    memory.record(target, 10.0)
    assert not memory.allowed(target, 11.0)
    assert memory.allowed(target, 12.0)
    memory.record(target, 13.0)
    assert not memory.allowed(target, 14.0)
    assert memory.allowed(target, 1000.0)


def test_completion_requires_distinct_maps_and_elapsed_confirmation_time():
    gate = CompletionGate(confirmations=3, delay=5.0)
    assert not gate.update(True, 1, 0.0)
    assert not gate.update(True, 1, 10.0)
    assert not gate.update(True, 2, 11.0)
    assert gate.update(True, 3, 12.0)
    assert not gate.update(False, 4, 13.0)
    assert not gate.update(True, 5, 20.0)
