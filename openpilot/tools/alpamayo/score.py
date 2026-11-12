#!/usr/bin/env python3
"""Score an Alpamayo trajectory against the path the car actually drove next.

Takes the rlog a sample was packed from, the packed sample (replay.py, optionally through
rescale.py) and the report of one run (benchmark.py or reference.py). Not part of comma's
prototype; see FORK_NOTES.md.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from openpilot.common.transformations.orientation import quat2rot, rot2quat, rot_from_euler
from openpilot.tools.lib.logreader import LogReader

HORIZON = np.arange(1, 65) * .1         # the 64 predicted poses, seconds after the sample
HISTORY = np.arange(-19, 1) * .1        # the 20 ego poses replay.py packs
CHECKPOINTS = (1., 2., 3., 4., 5., 6.4)
REFERENCE_SECONDS = 12.                 # driven path kept beyond the horizon, for a prediction that runs ahead
# openpilot: forward/right/down; Alpamayo: forward/left/up. Same as replay.py.
AXES = np.diag([1., -1., -1.])


def read_log(events):
  motions, calibrations, plans, steering, speeds = [], [], [], [], []
  for event in events:
    kind = event.which()
    if kind == 'deviceMotion' and event.valid:
      motion = event.deviceMotion
      if motion.orientationNED.valid and motion.velocityDevice.valid:
        motions.append((motion.timestamp / 1e9, [getattr(motion.orientationNED, axis) for axis in 'xyz'],
                        [getattr(motion.velocityDevice, axis) for axis in 'xyz']))
    elif kind == 'extrinsicsCalibration' and event.valid and event.extrinsicsCalibration.calStatus == 'calibrated':
      calibrations.append((event.logMonoTime / 1e9, list(event.extrinsicsCalibration.rpyCalib)))
    elif kind == 'modelV2':
      position = event.modelV2.position
      plans.append((event.modelV2.timestampEof / 1e9, np.array([list(position.t), list(position.x), list(position.y), list(position.z)])))
    elif kind == 'carControl':
      steering.append((event.logMonoTime / 1e9, bool(event.carControl.latActive)))
    elif kind == 'carState':
      speeds.append((event.logMonoTime / 1e9, event.carState.vEgo))
  assert motions and calibrations, 'Need valid deviceMotion and a calibrated extrinsicsCalibration in the rlog'
  motions.sort(key=lambda x: x[0])
  motions = [m for i, m in enumerate(motions) if i == 0 or m[0] > motions[i - 1][0]]
  return {'times': np.array([m[0] for m in motions]), 'orientation': rot_from_euler(np.array([m[1] for m in motions])),
          'velocity': np.array([m[2] for m in motions]), 'calibrations': calibrations, 'plans': plans, 'steering': steering,
          'speeds': np.array(sorted(speeds)).reshape(-1, 2)}


def speed_scale(log):
  """Per-sample factor that gives the localizer's velocity the size of the car's own speed.

  deviceMotion observes speed only through the cameras. On a 2023 F-150 Raptor R it read 18% below
  both GPS and the car's speed sensor in the one log checked, and replay.py packs it as it is."""
  assert len(log['speeds']), 'Need carState in the rlog to use the car\'s own speed'
  measured = np.interp(log['times'], log['speeds'][:, 0], log['speeds'][:, 1])
  estimated = np.linalg.norm(log['velocity'], axis=1)
  return np.where(measured < .1, 0., np.where(estimated > .5, measured / np.maximum(estimated, .5), 1.))


def rotation_at(times, rotations, t):
  i = int(np.clip(np.searchsorted(times, t) - 1, 0, len(times) - 2))
  a, b = rot2quat(rotations[i]), rot2quat(rotations[i + 1])
  b = -b if a @ b < 0 else b
  angle = np.arccos(np.clip(a @ b, -1., 1.))
  u = (t - times[i]) / (times[i + 1] - times[i])
  q = (1 - u) * a + u * b if angle < 1e-6 else (np.sin((1 - u) * angle) * a + np.sin(u * angle) * b) / np.sin(angle)
  return quat2rot(q / np.linalg.norm(q))


def driven_path(log, query, offsets, wheel_speed=False):
  """Where the device was at query + offsets, in the forward/left/up frame the model was given.

  Same construction as replay.py's ego history, so the two are directly comparable. With
  wheel_speed, distances come from the car's speed sensor and only directions from the localizer."""
  times = log['times']
  targets = query + np.asarray(offsets)
  assert times[0] <= targets.min() and times[-1] >= targets.max(), \
    f'rlog covers {times[0] - query:+.1f} to {times[-1] - query:+.1f} s around the sample, need {targets.min() - query:+.1f} to {targets.max() - query:+.1f} s'
  window = (times >= targets.min() - .1) & (times <= targets.max() + .1)
  assert np.diff(times[window]).max() < .15, 'Gap in deviceMotion over the scored window'
  velocity = log['velocity'] * speed_scale(log)[:, None] if wheel_speed else log['velocity']
  velocity_world = np.einsum('nij,nj->ni', log['orientation'], velocity)
  position_world = np.concatenate((np.zeros((1, 3)), np.cumsum(np.diff(times)[:, None] * (velocity_world[1:] + velocity_world[:-1]) / 2, axis=0)))

  def at(t):
    return np.stack([np.interp(t, times, position_world[:, axis]) for axis in range(3)], axis=-1)

  calibration = [rpy for timestamp, rpy in log['calibrations'] if timestamp <= query]
  assert calibration, 'No calibration before the sample'
  world_from_vehicle = rotation_at(times, log['orientation'], query) @ rot_from_euler(calibration[-1])
  return (at(targets) - at(query)) @ (AXES @ world_from_vehicle.T).T


def hold_turn(history):
  """Carry on at the speed and turn rate of the last second of ego history."""
  a, b = history[9, :2], history[14, :2]      # 1.0 and 0.5 s before the sample, which is the origin
  chord = np.linalg.norm(b)
  if chord < .05:
    return np.zeros((len(HORIZON), 2))
  ab, bc = b - a, -b
  curvature = 2 * (ab[0] * bc[1] - ab[1] * bc[0]) / max(np.linalg.norm(ab) * chord * np.linalg.norm(a), 1e-9)
  turned = 2 * np.arcsin(np.clip(curvature * chord / 2, -1., 1.))
  heading = np.arctan2(bc[1], bc[0]) + turned / 2
  speed = (turned / curvature if abs(curvature) > 1e-6 else chord) / .5
  if abs(curvature) < 1e-6:
    return np.outer(speed * HORIZON, [np.cos(heading), np.sin(heading)])
  angle = heading + curvature * speed * HORIZON
  return np.stack([np.sin(angle) - np.sin(heading), np.cos(heading) - np.cos(angle)], axis=-1) / curvature


def openpilot_plan(log, query):
  """What openpilot's own model planned from the same frame, or None if that frame has no modelV2."""
  if not log['plans']:
    return None
  timestamp, plan = min(log['plans'], key=lambda p: abs(p[0] - query))
  if abs(timestamp - query) > .06 or plan.shape[1] < 3:
    return None
  # modelV2 positions are forward/right/down in the calibrated frame. Its 33 points are up to 0.6 s apart, and
  # straight lines between them would cut a curve short by centimetres, so interpolate through three at a time.
  i = np.clip(np.searchsorted(plan[0], HORIZON), 1, plan.shape[1] - 2)
  t0, t1, t2 = plan[0][i - 1], plan[0][i], plan[0][i + 1]
  weights = np.stack([(HORIZON - t1) * (HORIZON - t2) / ((t0 - t1) * (t0 - t2)), (HORIZON - t0) * (HORIZON - t2) / ((t1 - t0) * (t1 - t2)),
                      (HORIZON - t0) * (HORIZON - t1) / ((t2 - t0) * (t2 - t1))])
  x, y = ((weights * np.stack([values[i - 1], values[i], values[i + 1]])).sum(axis=0) for values in (plan[1], plan[2]))
  return np.stack([x, -y], axis=-1)


def project(path, points):
  """Distance along a driven path and signed distance off it (+ is left) for each point.

  A point that lies beyond either end of the path gets nan: it has no place on the path to be compared with."""
  segment = np.diff(path, axis=0)
  length = np.linalg.norm(segment, axis=1)
  start, segment, length = path[:-1][length > 1e-6], segment[length > 1e-6], length[length > 1e-6]
  out = np.full((len(points), 2), np.nan)
  if not len(start):
    return out
  along = np.concatenate(([0.], np.cumsum(length)[:-1]))
  for n, point in enumerate(points):
    raw = np.einsum('ij,ij->i', point - start, segment) / length**2
    u = np.clip(raw, 0., 1.)
    distance = np.linalg.norm(point - (start + u[:, None] * segment), axis=1)
    i = int(np.argmin(distance))
    if (i == 0 and raw[i] < 0) or (i == len(start) - 1 and raw[i] > 1):
      continue
    side = segment[i, 0] * (point[1] - start[i, 1]) - segment[i, 1] * (point[0] - start[i, 0])
    out[n] = along[i] + u[i] * length[i], np.sign(side) * distance[i]
  return out


def camera_speed_ratio(log, query):
  """Distance the camera-based ego motion covers around the sample, relative to the car's speed sensor."""
  if not len(log['speeds']):
    return None
  window = np.concatenate((HISTORY, HORIZON))
  camera, sensor = (np.linalg.norm(np.diff(driven_path(log, query, window, wheel_speed), axis=0), axis=1).sum() for wheel_speed in (False, True))
  return float(camera / sensor) if sensor > 1. else None


def _clean(values):
  return [None if np.isnan(v) else float(v) for v in values]


def score(log, sample, prediction):
  query = float(np.asarray(sample['absolute_timestamps']).reshape(-1)[-1]) / 1e6
  packed = np.asarray(sample['ego_history_xyz']).reshape(20, 3)
  # the truth has to be built the way the sample's ego history was: as replay.py packs it, or rescaled by rescale.py
  mismatch = {}
  for wheel_speed in (False, True) if len(log['speeds']) else (False,):
    history = driven_path(log, query, HISTORY, wheel_speed)
    mismatch[wheel_speed] = np.abs(history - packed).max()
    if mismatch[wheel_speed] < .02:
      break
  else:
    off = min(mismatch.values())
    raise AssertionError(f'Ego history rebuilt from this rlog is {off:.2f} m off the packed sample: wrong rlog, or the frame convention changed')
  available = min(REFERENCE_SECONDS, np.floor((log['times'][-1] - query - 1e-3) * 10) / 10)
  assert available >= HORIZON[-1] - 1e-6, \
    f'rlog ends {log["times"][-1] - query:.1f} s after the sample, need {HORIZON[-1]:.1f} s: add the next segment with another --rlog'
  reference = np.concatenate((np.zeros((1, 2)), driven_path(log, query, np.arange(1, round(available * 10) + 1) * .1, wheel_speed)[:, :2]))
  truth = reference[1:len(HORIZON) + 1]
  truth_along = np.concatenate(([0.], np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))))[1:len(HORIZON) + 1]
  rows = {'alpamayo': np.asarray(prediction, dtype=np.float64).reshape(-1, len(HORIZON), 3)[0, :, :2], 'hold_turn': hold_turn(history)}
  if (plan := openpilot_plan(log, query)) is not None:
    rows['openpilot_plan'] = plan
  index = [int(round(t * 10)) - 1 for t in CHECKPOINTS]
  result = {'query_s': query, 'speed_mps': float(np.linalg.norm(history[-1, :2] - history[-2, :2]) / .1), 'checkpoints_s': list(CHECKPOINTS),
            'driven_xy': truth[index].tolist(), 'rows': {}, 'ego_motion': 'speed sensor' if wheel_speed else 'camera',
            'camera_speed_ratio': camera_speed_ratio(log, query)}
  steering = [active for timestamp, active in log['steering'] if query <= timestamp <= query + HORIZON[-1]]
  result['openpilot_steering_fraction'] = float(np.mean(steering)) if steering else None
  for name, xy in rows.items():
    projected = project(reference, xy)
    result['rows'][name] = {'off_path_m': _clean(projected[index, 1]), 'ahead_m': _clean(projected[index, 0] - truth_along[index]),
                            'mean_distance_5s_m': float(np.linalg.norm(xy[:50] - truth[:50], axis=1).mean()), 'xy': xy[index].tolist()}
  return result


def render(result, run=None):
  labels = {'alpamayo': 'Alpamayo', 'hold_turn': 'hold speed and turn', 'openpilot_plan': 'openpilot plan'}

  def cell(v):
    return f'{"n/a":>9}' if v is None else f'{v:+9.2f}'

  lines = [f'Sample at {result["query_s"]:.3f} s, {result["speed_mps"]:.1f} m/s']
  if run is not None:
    latency = run.get('inference_ms', run.get('timings_ms', {}).get('total'))
    took = 'time not recorded' if latency is None else f'{latency:.0f} ms'
    lines.append(f'Alpamayo: {took}, {run.get("generated_tokens")} reasoning tokens: {run.get("reasoning", "").strip()!r}')
  ratio, rescaled = result['camera_speed_ratio'], result['ego_motion'] == 'speed sensor'
  if ratio is None:
    lines.append('Ego motion given to the model: ' + ('rescaled to the speed sensor' if rescaled else 'camera-based, not checked against the speed sensor'))
  elif rescaled:
    lines.append(f'Ego motion given to the model: rescaled to the speed sensor (camera-based reads {abs(1 - ratio):.0%} {"low" if ratio < 1 else "high"} here)')
  elif abs(1 - ratio) <= .03:
    lines.append(f'Ego motion given to the model: camera-based, within {abs(1 - ratio):.0%} of the speed sensor')
  else:
    lines.append(f'Ego motion given to the model: camera-based, {abs(1 - ratio):.0%} {"below" if ratio < 1 else "above"} the speed sensor.')
    lines.append('  The model was told the wrong speed, and distances below are in that wrong scale. Run rescale.py on the sample and benchmark it again.')
  fraction = result['openpilot_steering_fraction']
  lines.append('openpilot steering during the scored 6.4 s: ' + ('unknown' if fraction is None else f'{fraction:.0%}'))
  if fraction:
    lines.append('  The driven path then follows openpilot\'s own plan, which flatters the "openpilot plan" row.')
  header = f'{"":22}' + ''.join(f'{t:>7g} s' for t in result['checkpoints_s'])
  lines += ['', header, f'{"truck drove: forward":22}' + ''.join(f'{x:9.1f}' for x, _ in result['driven_xy']),
            f'{"             left":22}' + ''.join(f'{y:+9.2f}' for _, y in result['driven_xy'])]
  for key, title in (('off_path_m', 'Off the driven path, m (+ is left of it)'), ('ahead_m', 'Ahead of the truck along the path, m (- is behind)')):
    lines += ['', title] + [f'  {labels[name]:20}' + ''.join(cell(v) for v in row[key]) for name, row in result['rows'].items()]
  lines += ['', 'Mean distance from the truck\'s position over the first 5 s, m']
  lines += [f'  {labels[name]:20}{row["mean_distance_5s_m"]:9.2f}' for name, row in result['rows'].items()]
  return '\n'.join(lines)


def main():
  parser = argparse.ArgumentParser(description='Score one Alpamayo trajectory against the path the car drove next.')
  parser.add_argument('--rlog', required=True, action='append',
                      help='rlog the sample was packed from; repeat with the next segment if the sample is near its end')
  parser.add_argument('--input', required=True, type=Path, help='sample packed by replay.py')
  parser.add_argument('--report', required=True, type=Path, help='report written by benchmark.py or reference.py')
  parser.add_argument('--run', type=int, default=-1, help='which measured run of the report to score; default is the last')
  parser.add_argument('--output', type=Path, help='also write the scores as JSON')
  args = parser.parse_args()
  log = read_log(event for path in args.rlog for event in LogReader(path))
  with np.load(args.input, allow_pickle=False) as data:
    sample = {key: data[key] for key in ('absolute_timestamps', 'ego_history_xyz')}
  report = json.loads(args.report.read_text())
  run = report['runs'][args.run]
  result = score(log, sample, run['xyz'])
  print(render(result, run))
  benchmarked = report.get('settings', {}).get('input')
  if benchmarked and Path(benchmarked).name != args.input.name:
    # scoring against a different ego history than the model was given compares two different scales
    print(f'\nNote: this report was made from {Path(benchmarked).name}, not {args.input.name}.')
  if args.output:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(result, reasoning=run.get('reasoning'), generated_tokens=run.get('generated_tokens')), indent=2) + '\n')


if __name__ == '__main__':
  main()
