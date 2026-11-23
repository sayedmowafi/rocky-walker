import numpy as np
import pytest

from rocky.config import ConfigError
from rocky.gait.patterns import make_pattern

PHASES = np.arange(10_000) / 10_000


def test_star_offsets():
    assert make_pattern("star", 0.6).offsets == pytest.approx((0.0, 0.4, 0.8, 0.2, 0.6))


def test_swing_start_order():
    p = make_pattern("star", 0.6)
    assert sorted(range(5), key=lambda i: p.offsets[i]) == [0, 3, 1, 4, 2]


def test_wave_one_leg_at_a_time():
    p = make_pattern("wave", 0.8)
    for phase in PHASES:
        assert sum(p.in_swing(i, phase) for i in range(5)) == 1


def test_star_invariants():
    p = make_pattern("star", 0.6)
    for phase in PHASES:
        swing = [p.in_swing(i, phase) for i in range(5)]
        assert sum(swing) <= 2
        assert 5 - sum(swing) >= 3
        for i in range(5):
            assert not (swing[i] and swing[(i + 1) % 5])


@pytest.mark.parametrize(("name", "duty"), [("star", 0.5), ("wave", 0.7), ("trot", 0.8)])
def test_rejects_unsafe_duty(name, duty):
    with pytest.raises(ConfigError):
        make_pattern(name, duty)
