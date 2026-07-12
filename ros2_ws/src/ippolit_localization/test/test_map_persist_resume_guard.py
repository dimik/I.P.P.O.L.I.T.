"""
Unit tests for q6a_map_persist's resume_decision -- the min_resume_bytes SAFETY guard.

See the module's CRASH FOUND note: deserializing a saved pose graph with zero real scan nodes
reliably segfaults slam_toolbox, so a too-small file must never reach deserialize_map.
"""
from ippolit_localization.q6a_map_persist import resume_decision


def test_zero_bytes_is_empty_not_refuse():
    assert resume_decision(0, min_resume_bytes=51200) == 'empty'


def test_below_threshold_is_refused():
    assert resume_decision(1, min_resume_bytes=51200) == 'refuse'
    assert resume_decision(51199, min_resume_bytes=51200) == 'refuse'


def test_at_threshold_is_allowed_to_resume():
    # the guard is a floor, not an exclusive bound -- exactly-equal must be safe to resume
    assert resume_decision(51200, min_resume_bytes=51200) == 'resume'


def test_well_above_threshold_resumes():
    assert resume_decision(500_000, min_resume_bytes=51200) == 'resume'


def test_never_confuses_empty_with_a_tiny_nonzero_file():
    # a 1-byte corrupt/truncated file must be refused, not silently treated as "nothing to
    # resume" -- those are different states (log messages differ, and 'empty' skips retry-timer
    # setup entirely while 'refuse' still logs a warning pointing at a possibly-stale file).
    assert resume_decision(1, min_resume_bytes=1024) == 'refuse'
