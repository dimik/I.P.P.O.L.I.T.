"""Unit tests for q6a_objmap's merge_object -- pure logic, no live rclpy Node needed."""
from ippolit_perception.q6a_objmap import merge_object


def test_first_observation_creates_new_entry():
    objects = []
    entry = merge_object(objects, 'chair', 1.0, 2.0, 0.6, merge_dist=0.5)
    assert objects == [{'cls': 'chair', 'x': 1.0, 'y': 2.0, 'n': 1, 'conf': 0.6}]
    assert entry is objects[0]


def test_close_same_class_merges_and_averages_position():
    objects = [{'cls': 'chair', 'x': 0.0, 'y': 0.0, 'n': 1, 'conf': 0.5}]
    merge_object(objects, 'chair', 1.0, 0.0, 0.6, merge_dist=2.0)
    assert len(objects) == 1
    assert objects[0]['n'] == 2
    assert objects[0]['x'] == 0.5           # running average of 0.0 and 1.0
    assert objects[0]['y'] == 0.0


def test_merge_keeps_max_confidence_not_latest():
    objects = [{'cls': 'chair', 'x': 0.0, 'y': 0.0, 'n': 1, 'conf': 0.9}]
    merge_object(objects, 'chair', 0.1, 0.0, 0.3, merge_dist=2.0)
    assert objects[0]['conf'] == 0.9         # lower incoming conf must not overwrite the max
    merge_object(objects, 'chair', 0.1, 0.0, 0.95, merge_dist=2.0)
    assert objects[0]['conf'] == 0.95         # higher incoming conf does win


def test_far_same_class_creates_separate_entry():
    objects = [{'cls': 'chair', 'x': 0.0, 'y': 0.0, 'n': 1, 'conf': 0.5}]
    merge_object(objects, 'chair', 10.0, 10.0, 0.6, merge_dist=0.5)
    assert len(objects) == 2                 # too far apart -- a second chair, not the same one


def test_different_class_never_merges_even_at_same_position():
    objects = [{'cls': 'chair', 'x': 0.0, 'y': 0.0, 'n': 1, 'conf': 0.5}]
    merge_object(objects, 'couch', 0.0, 0.0, 0.6, merge_dist=0.5)
    assert len(objects) == 2
    assert {o['cls'] for o in objects} == {'chair', 'couch'}


def test_distance_exactly_at_merge_dist_boundary_does_not_merge():
    # merge_object uses a strict "<" comparison -- exactly-equal distance must NOT merge.
    objects = [{'cls': 'chair', 'x': 0.0, 'y': 0.0, 'n': 1, 'conf': 0.5}]
    merge_object(objects, 'chair', 0.5, 0.0, 0.6, merge_dist=0.5)
    assert len(objects) == 2


def test_running_average_over_three_observations():
    objects = []
    merge_object(objects, 'chair', 0.0, 0.0, 0.5, merge_dist=2.0)
    merge_object(objects, 'chair', 1.0, 0.0, 0.5, merge_dist=2.0)
    merge_object(objects, 'chair', 2.0, 0.0, 0.5, merge_dist=2.0)
    assert objects[0]['n'] == 3
    assert objects[0]['x'] == 1.0             # mean of 0, 1, 2
