from types import SimpleNamespace

import numpy as np

from opendbc.can import CANPacker
from opendbc.can.parser import CANDefine
from openpilot.tools.lateral_maneuvers.ford_pscm_report import (
  DBC_NAME, Frames, LMC2, STATUS, _decoder, delivery, health, response, steady_holds,
)
from opendbc.can.dbc import DBC

DT = 0.05


def synthetic(ratio=0.8, lag_s=0.4, v=5.36, peak=0.035, mode=1, ste=2):
  """One maneuver: 2 s straight, 0.5 s ramp, 4 s hold, 0.5 s ramp out, 2 s straight. The truck
  follows the request at `ratio` behind a pure delay of `lag_s`."""
  t = np.arange(0, 9.0, DT)
  plan = np.interp(t, [0, 2.0, 2.5, 6.5, 7.0, 9.0], [0, 0, peak, peak, 0, 0])
  delayed = np.interp(t - lag_s, t, plan, left=0.0)
  kappa = ratio * delayed
  n = len(t)
  cols = {
    "t": t, "mode": np.full(n, float(mode)), "path_angle": -plan * v * 1.3, "v": np.full(n, v),
    "yaw": -kappa * v, "wheel": -kappa * 3700, "wheel_rate": np.gradient(-kappa * 3700, DT),
    "pressed": np.zeros(n), "torque": np.zeros(n), "fault": np.zeros(n), "lat_active": np.ones(n),
    "request": plan, "maneuver": (t >= 2.0).astype(float) * (t < 8.5), "plan": plan,
    "ste": np.full(n, float(ste)), "lim": np.zeros(n), "cap": np.full(n, 2.0), "deny": np.zeros(n),
    "current": np.zeros(n), "kappa": kappa,
  }
  f = Frames(cols=cols, tuning=SimpleNamespace(lowSpeedFactor=1.0, highSpeedFactor=1.3, primaryControl=2),
             labels=["hold +1.0m/s^2 12mph" if m else "" for m in cols["maneuver"]])
  f.yaw_bias = 0.0
  return f


class TestFordPSCMReport:
  def test_decodes_what_the_packer_encodes(self):
    """Every field comes from the DBC; round-trip it through the packer that builds real frames."""
    packer = CANPacker(DBC_NAME)
    dbc = DBC(DBC_NAME)
    _, dec = _decoder(dbc, LMC2)
    _, dat, _ = packer.make_can_msg(LMC2, 0, {"LatCtl_D2_Rq": 2, "LatCtlPath_An_Actl": -0.1235, "LatCtlPathOffst_L_Actl": 0.0})
    out = dec(bytes(dat))
    assert out["LatCtl_D2_Rq"] == 2
    assert abs(out["LatCtlPath_An_Actl"] + 0.1235) < 0.0006
    _, dec = _decoder(dbc, STATUS)
    _, dat, _ = packer.make_can_msg(STATUS, 0, {"LatCtlSte_D_Stat": 4, "LatCtlLim_D_Stat": 1, "LatCtlCpblty_D_Stat": 2})
    out = dec(bytes(dat))
    assert (out["LatCtlSte_D_Stat"], out["LatCtlLim_D_Stat"], out["LatCtlCpblty_D_Stat"]) == (4, 1, 2)

  def test_steady_delivery_and_the_factor_that_cancels_it(self):
    f = synthetic(ratio=0.8)
    holds = steady_holds(f)
    assert len(holds) == 1
    assert abs(holds[0]["ratio"] - 0.8) < 0.01
    f.cols = {k: np.concatenate([v, v]) for k, v in f.cols.items()}   # two runs
    f.labels = f.labels * 2
    f.cols["t"] = np.arange(len(f.cols["t"])) * DT
    text = "\n".join(delivery(f))
    assert "0.80" in text and "-> 1.25" in text

  def test_response_lag(self):
    lines = response(synthetic(ratio=1.0, lag_s=0.4))
    lag50 = float(lines[1].split()[-4])   # the label has spaces; count from the right
    assert abs(lag50 - 0.4) < DT + 1e-9

  def test_denied_is_an_abort_condition(self):
    define = CANDefine(DBC_NAME)
    ok = "\n".join(health(synthetic(ste=2), define))
    bad = "\n".join(health(synthetic(mode=2, ste=4), define))
    assert "no abort condition hit" in ok
    assert "ABORT CONDITION" in bad and "Extended" in bad
