#!/usr/bin/env python3
"""
Ford PSCM closed-course characterization report. See FORD_PSCM.md in this directory for the
protocol this analyzes.

Reads rlogs (local paths, or anything LogReader accepts, such as a route ID) and reports, from what
openpilot sent and what the PSCM said back:

  health       what the module reported per requested mode (Limited / Extended), steering faults,
               panda blocks, hand-backs; flags the protocol's abort conditions
  limit onset  LatCtlLim_D_Stat against commanded path angle, per requested mode
  delivery     steady-state delivered vs requested curvature in the Ford maneuver holds, per
               speed and level, and the Low Speed Factor that would cancel the shortfall
  response     request-to-truck lag and overshoot for every maneuver run

Every CAN signal is decoded from the DBC, not from hand-written bit positions.

  python openpilot/tools/lateral_maneuvers/ford_pscm_report.py <rlog or route> [...] [--csv out.csv]
"""
import argparse
import csv
import math
from collections import Counter, defaultdict
from dataclasses import dataclass, field

import numpy as np

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value, CANDefine
from openpilot.tools.lib.logreader import LogReader

DBC_NAME = "ford_lincoln_base_pt"
LMC2 = "LateralMotionControl2"
STATUS = "Lane_Assist_Data3_FD1"
EPAS = "EPAS_INFO"
PT_BUS = 0

# the angle controller's gain schedule applies the Low Speed Factor at and below this speed
LOW_SPEED_FACTOR_MAX_V = 13.5            # m/s
STEADY_MIN_S = 2.5                       # a hold must be flat this long to count
STEADY_TAIL_S = 2.0                      # and only its last part is measured
STRAIGHT_KAPPA = 2e-4                    # 1/m, what counts as straight for the yaw-bias estimate
ANGLE_BUCKETS_DEG = list(range(11)) + [15, 20, 30]
# Limit onset is speed dependent: on logged drives the PSCM reported LimitClose from 3-4 deg of
# path angle at 20-24 m/s and never up to 11 deg below 10 m/s. One onset number across speeds is
# a blend of whatever speeds the data happened to cover, so the table is split.
SPEED_BANDS = ((0.0, 9.0), (9.0, 18.0), (18.0, 99.0))   # m/s


def _decoder(dbc: DBC, msg_name: str):
  msg = dbc.name_to_msg[msg_name]

  def decode(dat: bytes) -> dict[str, float]:
    out = {}
    for name, sig in msg.sigs.items():
      raw = get_raw_value(dat, sig)
      if sig.is_signed and raw >= (1 << (sig.size - 1)):
        raw -= 1 << sig.size
      out[name] = raw * sig.factor + sig.offset
    return out
  return msg.address, decode


def _names(define: CANDefine, msg: str, sig: str) -> dict[int, str]:
  return {int(k): str(v) for k, v in define.dv.get(msg, {}).get(sig, {}).items()}


@dataclass
class Frames:
  cols: dict[str, np.ndarray] = field(default_factory=dict)
  tuning: object = None
  labels: list[str] = field(default_factory=list)
  blocked_delta: int = 0
  files: int = 0

  def __len__(self):
    return len(self.cols.get("t", []))


def load(sources: list[str]) -> Frames:
  """One row per LateralMotionControl2 frame openpilot sent, with the latest of everything else."""
  dbc = DBC(DBC_NAME)
  lmc2_addr, dec_lmc2 = _decoder(dbc, LMC2)
  status_addr, dec_status = _decoder(dbc, STATUS)
  epas_addr, dec_epas = _decoder(dbc, EPAS)

  rows = defaultdict(list)
  f = Frames()
  latest = {"cs": None, "cc": None, "plan_valid": False, "plan_kappa": 0.0, "label": "",
            "status": None, "current": math.nan}
  blocked = []
  for src in sources:
    f.files += 1
    t0 = None
    for m in LogReader(src):
      if t0 is None:
        t0 = m.logMonoTime
      w = m.which()
      if w == "carState":
        latest["cs"] = m.carState
      elif w == "carControl":
        latest["cc"] = m.carControl
      elif w == "lateralManeuverPlan":
        latest["plan_valid"] = m.valid
        latest["plan_kappa"] = m.lateralManeuverPlan.desiredCurvature
      elif w == "alertDebug":
        latest["label"] = m.alertDebug.alertText2
      elif w == "carParamsSP" and f.tuning is None:
        f.tuning = m.carParamsSP.fordLateralTuning
      elif w == "pandaStates" and len(m.pandaStates):
        blocked.append(m.pandaStates[0].safetyTxBlocked)
      elif w == "can":
        for c in m.can:
          if c.src != PT_BUS:
            continue
          if c.address == status_addr:
            latest["status"] = dec_status(bytes(c.dat))
          elif c.address == epas_addr:
            latest["current"] = dec_epas(bytes(c.dat)).get("SteMdule_I_Est", math.nan)
      elif w == "sendcan":
        for c in m.sendcan:
          if c.address != lmc2_addr or latest["cs"] is None or latest["cc"] is None:
            continue
          cmd = dec_lmc2(bytes(c.dat))
          cs, cc, st = latest["cs"], latest["cc"], latest["status"] or {}
          rows["t"].append((m.logMonoTime - t0) * 1e-9 + 1e4 * (f.files - 1))
          rows["mode"].append(cmd["LatCtl_D2_Rq"])
          rows["path_angle"].append(cmd["LatCtlPath_An_Actl"])
          rows["v"].append(cs.vEgoRaw)
          rows["yaw"].append(cs.yawRate)
          rows["wheel"].append(cs.steeringAngleDeg)
          rows["wheel_rate"].append(cs.steeringRateDeg)
          rows["pressed"].append(float(cs.steeringPressed))
          rows["torque"].append(cs.steeringTorque)
          rows["fault"].append(float(cs.steerFaultTemporary or cs.steerFaultPermanent))
          rows["lat_active"].append(float(cc.latActive))
          rows["request"].append(cc.actuators.curvature)
          rows["maneuver"].append(float(latest["plan_valid"]))
          rows["plan"].append(latest["plan_kappa"])
          rows["ste"].append(st.get("LatCtlSte_D_Stat", math.nan))
          rows["lim"].append(st.get("LatCtlLim_D_Stat", math.nan))
          rows["cap"].append(st.get("LatCtlCpblty_D_Stat", math.nan))
          rows["deny"].append(st.get("LaActDeny_B_Actl", math.nan))
          rows["current"].append(latest["current"])
          f.labels.append(latest["label"] if latest["plan_valid"] else "")
  f.cols = {k: np.asarray(v, dtype=float) for k, v in rows.items()}
  if blocked:
    f.blocked_delta = int(max(blocked) - blocked[0])
  if len(f):
    f.cols["kappa"] = measured_curvature(f)
  return f


def measured_curvature(f: Frames) -> np.ndarray:
  """Truck curvature from yaw rate in openpilot's sign convention, with the yaw sensor's offset
  removed (every logged drive on this truck shows about -0.0047 rad/s on straight road)."""
  c = f.cols
  straight = (c["lat_active"] > 0) & (np.abs(c["request"]) < STRAIGHT_KAPPA) & (c["v"] > 5) & (c["pressed"] == 0)
  f.yaw_bias = float(np.median(c["yaw"][straight])) if straight.sum() > 100 else 0.0
  return -(c["yaw"] - f.yaw_bias) / np.maximum(c["v"], 0.5)


def health(f: Frames, define: CANDefine) -> list[str]:
  c = f.cols
  ste_n = _names(define, STATUS, "LatCtlSte_D_Stat")
  cap_n = _names(define, STATUS, "LatCtlCpblty_D_Stat")
  out = []
  abort = []
  for mode in sorted(set(c["mode"][c["mode"] > 0].astype(int))):
    m = c["mode"] == mode
    ste = Counter(int(x) for x in c["ste"][m] if not math.isnan(x))
    cap = Counter(int(x) for x in c["cap"][m] if not math.isnan(x))
    deny = int(np.nansum(c["deny"][m]))
    out.append(f"requested {mode} ({'Extended' if mode == 2 else 'Limited'}): {m.sum()} frames")
    out.append("  LatCtlSte:    " + ", ".join(f"{ste_n.get(k, k)} {v}" for k, v in ste.most_common()))
    out.append("  LatCtlCpblty: " + ", ".join(f"{cap_n.get(k, k)} {v}" for k, v in cap.most_common()))
    out.append(f"  LaActDeny frames: {deny}")
    for k, name in ste_n.items():
      if ste.get(k) and any(word in name.lower() for word in ("fault", "deni", "unavail")):
        abort.append(f"PSCM reported {name} while {'Extended' if mode == 2 else 'Limited'} was requested ({ste[k]} frames)")
    if deny:
      abort.append(f"LaActDeny set on {deny} frames with mode {mode} requested")
  handed_back = int(((c["lat_active"] > 0) & (c["mode"] == 0)).sum())
  out.append(f"steering faults: {int(c['fault'].sum())} frames; panda blocks this log: {f.blocked_delta}; " +
             f"hand-backs (mode 0 while engaged): {handed_back} frames")
  if c["fault"].sum():
    abort.append("carState steering fault")
  if f.blocked_delta:
    abort.append(f"panda blocked {f.blocked_delta} messages; replay the route before trusting anything else")
  out += [f"ABORT CONDITION: {a}" for a in abort] or ["no abort condition hit"]
  return out


def _onset_table(c: dict, m: np.ndarray, lim_n: dict[int, str], title: str) -> list[str]:
  deg = np.degrees(np.abs(c["path_angle"][m]))
  lim = c["lim"][m].astype(int)
  wheel = np.abs(c["wheel"][m])
  cur = c["current"][m]
  keys = sorted(lim_n)
  out = [f"{title}: peak |path angle| {deg.max():.1f} deg",
         f"  {'|path angle| deg':>16} {'n':>6} " + " ".join(f"{lim_n[k]:>13}" for k in keys) +
         f" {'|wheel| deg':>12} {'motor A':>8}"]
  onset = None
  edges = ANGLE_BUCKETS_DEG + [1e9]
  for lo, hi in zip(edges[:-1], edges[1:], strict=True):
    b = (deg >= lo) & (deg < hi)
    if not b.any():
      continue
    cnt = Counter(lim[b])
    pct = [100.0 * cnt.get(k, 0) / b.sum() for k in keys]
    label = f"{lo}-{hi}" if hi < 1e9 else f"{lo}+"
    out.append(f"  {label:>16} {b.sum():>6} " + " ".join(f"{p:>12.1f}%" for p in pct) +
               f" {np.median(wheel[b]):>12.0f} {np.nanmedian(cur[b]):>8.1f}")
    if onset is None and any(cnt.get(k, 0) for k in keys if k != 0):
      onset = label
  out.append(f"  first non-NotReached report in bucket: {onset or 'never'}")
  return out


def limit_onset(f: Frames, define: CANDefine) -> list[str]:
  """LatCtlLim_D_Stat against commanded path angle, hands off, per requested mode and speed band."""
  c = f.cols
  lim_n = _names(define, STATUS, "LatCtlLim_D_Stat")
  out = []
  for mode in sorted(set(c["mode"][c["mode"] > 0].astype(int))):
    for v_lo, v_hi in SPEED_BANDS:
      m = ((c["mode"] == mode) & (c["pressed"] == 0) & ~np.isnan(c["lim"]) &
           (c["v"] >= v_lo) & (c["v"] < v_hi))
      if m.sum() >= 20:
        name = "Extended" if mode == 2 else "Limited"
        out += _onset_table(c, m, lim_n, f"requested {name}, hands off, {v_lo:.0f}-{v_hi:.0f} m/s")
  return out


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
  out, i = [], 0
  while i < len(mask):
    if mask[i]:
      j = i
      while j < len(mask) and mask[j]:
        j += 1
      out.append((i, j))
      i = j
    else:
      i += 1
  return out


def steady_holds(f: Frames) -> list[dict]:
  """The flat part of every maneuver hold: request steady within 3% for STEADY_MIN_S, hands off,
  measured over its last STEADY_TAIL_S."""
  c = f.cols
  out = []
  for a, b in _runs((c["maneuver"] > 0) & (c["lat_active"] > 0)):
    plan = c["plan"][a:b]
    peak = np.max(np.abs(plan))
    if peak < 1e-3:
      continue
    flat = np.abs(np.abs(plan) - peak) < 0.03 * peak
    for fa, fb in _runs(flat):
      fa += a
      fb += a
      if c["t"][fb - 1] - c["t"][fa] < STEADY_MIN_S or c["pressed"][fa:fb].any():
        continue
      tail = fa + int(np.searchsorted(c["t"][fa:fb], c["t"][fb - 1] - STEADY_TAIL_S))
      sl = slice(tail, fb)
      req = float(np.median(c["plan"][sl]))
      got = float(np.median(c["kappa"][sl]))
      v = float(np.median(c["v"][sl]))
      pa = float(np.median(np.abs(c["path_angle"][sl])))
      out.append({"label": f.labels[fa], "mode": int(np.median(c["mode"][sl])), "v": v, "request": req,
                  "delivered": got, "ratio": got / req if req else math.nan, "path_angle": pa,
                  "wheel": float(np.median(np.abs(c["wheel"][sl]))),
                  "plant": abs(got) / (pa / max(v, 0.5)) if pa > 1e-3 else math.nan,
                  "lim": Counter(int(x) for x in c["lim"][sl] if not math.isnan(x)).most_common(1)})
  return out


def _mode_name(mode: float, short: bool = False) -> str:
  if short:
    return "Ext" if mode == 2 else "Lim"
  return "Extended" if mode == 2 else "Limited"


def delivery(f: Frames) -> list[str]:
  holds = steady_holds(f)
  if not holds:
    return ["no steady maneuver holds in these logs (run the Ford suite in Lateral Maneuver Mode)"]
  lsf = getattr(f.tuning, "lowSpeedFactor", 0.0) or 1.0
  out = [f"{'maneuver':>28} {'mode':>5} {'v':>5} {'request':>8} {'truck':>8} {'ratio':>6} {'path ang':>9} " +
         f"{'wheel':>6} {'plant k':>8}"]
  for h in holds:
    out.append(f"{h['label']:>28} {_mode_name(h['mode'], True):>5} {h['v']:5.1f} {h['request'] * 1e3:7.1f}m " +
               f"{h['delivered'] * 1e3:7.1f}m {h['ratio']:6.2f} {h['path_angle']:9.3f} {h['wheel']:6.0f} {h['plant']:8.2f}")
  for mode in sorted({h["mode"] for h in holds}):
    low = [h["ratio"] for h in holds if h["mode"] == mode and h["v"] <= LOW_SPEED_FACTOR_MAX_V and h["ratio"] > 0]
    if len(low) < 2:
      continue
    r = float(np.median(low))
    spread = float(np.percentile(low, 75) - np.percentile(low, 25))
    factor = lsf / r
    clamped = min(max(factor, 0.5), 1.5)
    note = "" if clamped == factor else " (clamped to the dial's 0.5-1.5)"
    out.append(f"{_mode_name(mode)}: median delivery at or below {LOW_SPEED_FACTOR_MAX_V} m/s {r:.2f} " +
               f"(IQR {spread:.2f}, {len(low)} holds). Low Speed Factor {lsf:.2f} -> {clamped:.2f} would cancel it{note}")
    if spread > 0.15:
      out.append("  spread is wide: the shortfall is not a single gain, so a factor alone will not fix it")
  return out


def _crossing(t: np.ndarray, sig: np.ndarray, sign: float, level: float) -> float:
  """First time sig reaches level in the direction of sign."""
  idx = np.nonzero(sig * sign >= level)[0]
  return float(t[idx[0]]) if len(idx) else math.nan


def response(f: Frames) -> list[str]:
  """Request-to-truck lag at 50% and 90% of each run's peak, and overshoot."""
  c = f.cols
  out = [f"{'maneuver':>28} {'mode':>5} {'v':>5} {'lag50':>6} {'lag90':>6} {'overshoot':>9} {'peak wheel rate':>16}"]
  for a, b in _runs((c["maneuver"] > 0) & (c["lat_active"] > 0)):
    plan = c["plan"][a:b]
    kap = c["kappa"][a:b]
    t = c["t"][a:b]
    peak = float(plan[int(np.argmax(np.abs(plan)))])
    if abs(peak) < 1e-3 or c["pressed"][a:b].any():
      continue
    s = float(np.sign(peak))
    lag50 = _crossing(t, kap, s, 0.5 * abs(peak)) - _crossing(t, plan, s, 0.5 * abs(peak))
    lag90 = _crossing(t, kap, s, 0.9 * abs(peak)) - _crossing(t, plan, s, 0.9 * abs(peak))
    over = (np.max(kap * s) / abs(peak) - 1) * 100
    out.append(f"{f.labels[a]:>28} {_mode_name(np.median(c['mode'][a:b]), True):>5} {np.median(c['v'][a:b]):5.1f} " +
               f"{lag50:6.2f} {lag90:6.2f} {over:8.0f}% {np.max(np.abs(c['wheel_rate'][a:b])):15.0f}/s")
  return out if len(out) > 1 else ["no complete hands-off maneuver runs"]


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("sources", nargs="+", help="rlog paths or a route ID")
  parser.add_argument("--csv", help="write every steering frame here")
  args = parser.parse_args()

  f = load(args.sources)
  if not len(f):
    raise SystemExit("no LateralMotionControl2 frames found: is this a CAN FD Ford in angle or curvature mode?")
  define = CANDefine(DBC_NAME)
  t = f.tuning
  print(f"{f.files} log(s), {len(f)} steering frames, yaw bias {f.yaw_bias:+.4f} rad/s")
  if t is not None:
    print(f"primary control {t.primaryControl}, low/high speed factor {t.lowSpeedFactor:.2f}/{t.highSpeedFactor:.2f}, " +
          f"extended mode test {getattr(t, 'extendedModeTest', False)}")
  for title, lines in (("HEALTH", health(f, define)), ("LIMIT ONSET", limit_onset(f, define)),
                       ("STEADY-STATE DELIVERY", delivery(f)), ("RESPONSE", response(f))):
    print(f"\n== {title}")
    print("\n".join(lines))

  if args.csv:
    with open(args.csv, "w", newline="") as fh:
      w = csv.writer(fh)
      keys = list(f.cols)
      w.writerow(keys + ["label"])
      for i in range(len(f)):
        w.writerow([f"{f.cols[k][i]:.6g}" for k in keys] + [f.labels[i]])
    print(f"\nwrote {args.csv}")


if __name__ == "__main__":
  main()
