"""The absence release: unit cases of the deployed rule, and the 4 August replay."""
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "controller"))
import release as rl  # noqa: E402


def test_needs_three_live_channels():
    assert rl.absence_evidence_nats(0.1, 0.1, None, None, saw_density_dip=True)[0] == 0.0
    assert rl.absence_evidence_nats(0.1, 0.1, 0.1, 1, saw_density_dip=True)[1] == "too_few_live_channels(2)"


def test_quiet_bin_needs_a_traffic_dropout_or_a_probe():
    assert rl.absence_evidence_nats(0.1, 0.1, 0.1, 3, saw_density_dip=False) == (0.0, "no_traffic_dropout")
    assert rl.absence_evidence_nats(0.1, 0.1, 0.1, 3, saw_density_dip=True) == (0.05, "quiet_3ch")
    nats, why = rl.absence_evidence_nats(0.1, 0.1, 0.1, 3, illuminated=True, saw_density_dip=False)
    assert abs(nats - 0.15) < 1e-12 and why == "quiet_3ch_probed"


def test_movement_subtracts_a_signed_penalty():
    led = rl.AbsenceLedger()
    led.enter()
    led.s_absence = 3.0
    led.step(rl.Bin(0.0, 3.0, 0.0, 0.0, 3, 5.0))
    assert abs(led.s_absence - (3.0 - rl.ABSENCE_ACTIVE_PENALTY_MODERATE_NATS)) < 1e-12
    led.s_absence = 3.0
    led.step(rl.Bin(10.0, -3.0, 0.0, 0.0, 3, 5.0))          # negative z holds
    assert led.s_absence == 3.0


def test_release_at_threshold():
    led = rl.AbsenceLedger()
    led.enter()
    led.s_absence = rl.ABSENCE_RELEASE_THRESHOLD_NATS - 0.05
    event, _ = led.step(rl.Bin(0.0, 0.0, 0.0, 0.0, 3, 5.0))
    assert event == "absence_confirmed" and not led.present


def test_replay_reproduces_the_logged_release():
    r = subprocess.run([sys.executable, str(ROOT / "controller" / "replay_episode.py")],
                       capture_output=True, text=True, cwd=ROOT / "controller")
    assert r.returncode == 0, r.stdout + r.stderr
    assert '"release_replayed": "21:50:30"' in r.stdout and '"mismatches": 0' in r.stdout
