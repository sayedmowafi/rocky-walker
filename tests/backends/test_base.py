import numpy as np

from rocky.backends.base import DryRunBackend


def test_dry_run_records():
    backend = DryRunBackend()
    a = np.zeros((5, 3))
    backend.write(a)
    a[0, 0] = 1.0
    backend.write(a)
    assert len(backend.history) == 2
    assert backend.history[0][0, 0] == 0.0
    assert backend.history[1][0, 0] == 1.0
