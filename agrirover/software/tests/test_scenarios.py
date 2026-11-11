"""System-level simulation scenarios (see agrirover/sim/scenarios.py for the acceptance criteria)."""
import pytest

from agrirover.sim import scenarios


@pytest.mark.parametrize("fn", scenarios.SCENARIOS, ids=lambda f: f.__name__)
def test_scenario(fn):
    res = fn()
    failed = [f"{c}: {m}" for c, ok, m in res.criteria if not ok]
    assert res.passed, f"{res.name}: " + "; ".join(failed)
