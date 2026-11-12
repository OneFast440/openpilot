import json
import sys

import numpy as np
import pytest

from openpilot.cereal import log as capnp_log
from openpilot.common.transformations.orientation import rot2euler, rot_from_euler
from openpilot.tools.alpamayo import rescale as rescale_tool, score as score_tool
from openpilot.tools.alpamayo.rescale import rescale
from openpilot.tools.alpamayo.score import HISTORY, HORIZON, driven_path, hold_turn, project, read_log, render, score

DT = .05
T0 = 100.
SAMPLE = T0 + 3.                    # sample time of every synthetic drive
CALIB = [0., .09, -.025]            # device pitched and yawed on the windshield, as on the truck
PLAN_T = 10. * (np.arange(33) / 32) ** 2


def arc(speed, yaw_rate, t):
  """Forward/left displacement after t seconds at constant speed and yaw rate (+ turns left)."""
  t = np.asarray(t, dtype=np.float64)
  if abs(yaw_rate) < 1e-9:
    return np.stack([speed * t, np.zeros_like(t)], axis=-1)
  return np.stack([np.sin(yaw_rate * t), 1 - np.cos(yaw_rate * t)], axis=-1) * speed / yaw_rate


def drive(speed=10., yaw_rate=0., seconds=12., camera_scale=1., steering_until=None, plan_scale=1.):
  """rlog events of a drive at constant speed and yaw rate. The localizer's speed is camera_scale of the truth."""
  events = []

  def add(t, **fields):
    events.append(capnp_log.Event.new_message(logMonoTime=int(round(t * 1e9)), valid=True, **fields).as_reader())

  add(T0, extrinsicsCalibration={'calStatus': 'calibrated', 'rpyCalib': CALIB})
  for t in T0 + np.arange(0, seconds + DT / 2, DT):
    heading = -yaw_rate * (t - T0)    # NED yaw grows clockwise, so a left turn shrinks it
    world_from_device = rot_from_euler([0., 0., heading]) @ rot_from_euler(CALIB).T
    velocity = world_from_device.T @ (speed * np.array([np.cos(heading), np.sin(heading), 0.])) * camera_scale
    roll, pitch, yaw = rot2euler(world_from_device)
    add(t, deviceMotion={'timestamp': int(round(t * 1e9)),
                         'orientationNED': {'x': float(roll), 'y': float(pitch), 'z': float(yaw), 'valid': True},
                         'velocityDevice': {'x': float(velocity[0]), 'y': float(velocity[1]), 'z': float(velocity[2]), 'valid': True}})
    add(t, carState={'vEgo': speed})
    add(t, carControl={'latActive': bool(steering_until is not None and t < steering_until)})
    plan = arc(speed, yaw_rate, PLAN_T) * plan_scale
    add(t, modelV2={'timestampEof': int(round(t * 1e9)),     # forward/right/down
                    'position': {'t': PLAN_T.tolist(), 'x': plan[:, 0].tolist(), 'y': (-plan[:, 1]).tolist(), 'z': [0.] * len(PLAN_T)}})
  return events


def packed(log, wheel_speed=False):
  """A sample as replay.py packs it: the camera frame time and the 20 ego poses before it."""
  history = driven_path(log, SAMPLE, HISTORY, wheel_speed)
  return {'absolute_timestamps': np.array([round(SAMPLE * 1e6)], dtype=np.int64), 'ego_history_xyz': history[None].astype(np.float32)}


def prediction(xy):
  return np.concatenate((xy, np.zeros((len(xy), 1))), axis=-1)[None]


@pytest.mark.parametrize("yaw_rate", [.2, -.2, 0.])
def test_driven_path_is_forward_left_up(yaw_rate):
  log = read_log(drive(yaw_rate=yaw_rate))
  for offsets in (HORIZON, HISTORY):
    path = driven_path(log, SAMPLE, offsets)
    np.testing.assert_allclose(path[:, :2], arc(10., yaw_rate, offsets), atol=2e-3)
    np.testing.assert_allclose(path[:, 2], 0., atol=2e-3)
  if yaw_rate:
    assert np.sign(driven_path(log, SAMPLE, HORIZON)[-1, 1]) == np.sign(yaw_rate)   # a left turn ends to the left


@pytest.mark.parametrize("speed,yaw_rate", [(10., .2), (25., -.05), (8., 0.), (0., 0.)])
def test_hold_turn_continues_the_history(speed, yaw_rate):
  history = np.concatenate((arc(speed, yaw_rate, HISTORY), np.zeros((20, 1))), axis=-1)
  np.testing.assert_allclose(hold_turn(history), arc(speed, yaw_rate, HORIZON), atol=1e-6)


def test_project_measures_along_and_across():
  path = np.stack([np.arange(0., 21.), np.zeros(21)], axis=-1)
  points = np.array([[5., .3], [7.5, -.2], [25., 0.], [-1., 0.]])
  out = project(path, points)
  np.testing.assert_allclose(out[:2], [[5., .3], [7.5, -.2]], atol=1e-9)   # + is left of the direction of travel
  assert np.isnan(out[2:]).all()                                           # beyond either end there is nothing to compare with
  assert np.isnan(project(np.zeros((5, 2)), points)).all()                 # a parked car has no path


def test_perfect_prediction_scores_zero_everywhere():
  log = read_log(drive(yaw_rate=.15))
  result = score(log, packed(log), prediction(arc(10., .15, HORIZON)))
  assert set(result['rows']) == {'alpamayo', 'hold_turn', 'openpilot_plan'}
  for row in result['rows'].values():
    np.testing.assert_allclose(row['off_path_m'], 0., atol=5e-3)
    np.testing.assert_allclose(row['ahead_m'], 0., atol=5e-3)
    assert row['mean_distance_5s_m'] < 5e-3
  assert result['ego_motion'] == 'camera' and result['camera_speed_ratio'] == pytest.approx(1., abs=1e-3)
  assert result['speed_mps'] == pytest.approx(10., abs=.01)
  np.testing.assert_allclose(result['driven_xy'], arc(10., .15, result['checkpoints_s']), atol=2e-3)


def test_offsets_show_up_with_their_sign():
  log = read_log(drive(yaw_rate=.1))
  slow = arc(9.5, .095, HORIZON)                              # the same 100 m circle, driven 5% slower
  heading = .095 * HORIZON
  left = slow + .3 * np.stack([-np.sin(heading), np.cos(heading)], axis=-1)
  row = score(log, packed(log), prediction(left))['rows']['alpamayo']
  np.testing.assert_allclose(row['off_path_m'], .3, atol=5e-3)
  np.testing.assert_allclose(row['ahead_m'], -.5 * np.array([1., 2., 3., 4., 5., 6.4]), atol=.02)
  straight = score(log, packed(log), prediction(arc(10., 0., HORIZON)))['rows']['alpamayo']
  assert all(v < 0 for v in straight['off_path_m'])           # going straight on while the road bends left: right of the path


def test_wrong_rlog_is_refused():
  sample = packed(read_log(drive(yaw_rate=.2)))
  with pytest.raises(AssertionError, match='off the packed sample'):
    score(read_log(drive(yaw_rate=-.2)), sample, prediction(arc(10., .2, HORIZON)))


def test_needs_the_full_horizon_and_accepts_a_second_segment():
  events = drive(seconds=12.)
  first = [e for e in events if e.logMonoTime < (SAMPLE + 5.) * 1e9]
  sample = packed(read_log(events))
  with pytest.raises(AssertionError, match='add the next segment'):
    score(read_log(first), sample, prediction(arc(10., 0., HORIZON)))
  second = [e for e in events if e.logMonoTime >= (SAMPLE + 5.) * 1e9]
  row = score(read_log(first + second), sample, prediction(arc(10., 0., HORIZON)))['rows']['alpamayo']
  np.testing.assert_allclose(row['ahead_m'], 0., atol=5e-3)


def test_camera_speed_off_is_reported_and_rescaled():
  log = read_log(drive(yaw_rate=.1, camera_scale=.82, plan_scale=.92))
  sample = packed(log)
  np.testing.assert_allclose(sample['ego_history_xyz'][0, :, :2], .82 * arc(10., .1, HISTORY), atol=3e-3)
  result = score(log, sample, prediction(.82 * arc(10., .1, HORIZON)))
  assert result['ego_motion'] == 'camera' and result['camera_speed_ratio'] == pytest.approx(.82, abs=2e-3)
  assert '18% below the speed sensor' in render(result) and 'rescale.py' in render(result)

  scaled, camera_m, sensor_m = rescale(log, sample)
  assert camera_m / sensor_m == pytest.approx(.82, abs=2e-3)
  assert scaled['ego_history_xyz'].shape == sample['ego_history_xyz'].shape and scaled['ego_history_xyz'].dtype == np.float32
  np.testing.assert_allclose(scaled['ego_history_xyz'][0, :, :2], arc(10., .1, HISTORY), atol=3e-3)
  np.testing.assert_array_equal(scaled['absolute_timestamps'], sample['absolute_timestamps'])
  with pytest.raises(AssertionError, match='already rescaled'):
    rescale(log, scaled)

  result = score(log, scaled, prediction(arc(10., .1, HORIZON)))        # truth is now in the car's own scale
  assert result['ego_motion'] == 'speed sensor' and 'rescaled to the speed sensor' in render(result)
  np.testing.assert_allclose(result['driven_xy'], arc(10., .1, result['checkpoints_s']), atol=3e-3)
  np.testing.assert_allclose(result['rows']['alpamayo']['off_path_m'], 0., atol=5e-3)
  np.testing.assert_allclose(result['rows']['alpamayo']['ahead_m'], 0., atol=5e-3)
  # openpilot's plan keeps its own camera scale, so against the true path it comes up short: 8% of 10 m/s
  behind = result['rows']['openpilot_plan']['ahead_m']
  np.testing.assert_allclose(behind[:3], [-.8, -1.6, -2.4], atol=.03)
  assert all(later < earlier for earlier, later in zip(behind, behind[1:], strict=False))


def test_unusable_messages_are_ignored():
  events = drive()
  broken = []
  for event in events:
    builder = event.as_builder()
    if event.which() == 'deviceMotion':
      builder.deviceMotion.velocityDevice.valid = False
    elif event.which() == 'extrinsicsCalibration':
      builder.extrinsicsCalibration.calStatus = 'uncalibrated'
    broken.append(builder.as_reader())
  with pytest.raises(AssertionError, match='Need valid deviceMotion'):
    read_log(broken)
  # a calibration that only arrives after the sample cannot have been used to pack it
  late = [e for e in events if e.which() != 'extrinsicsCalibration'] + [e for e in drive() if e.which() == 'extrinsicsCalibration']
  late[-1] = late[-1].as_builder()
  late[-1].logMonoTime = int((SAMPLE + 1.) * 1e9)
  with pytest.raises(AssertionError, match='No calibration before the sample'):
    driven_path(read_log(late), SAMPLE, HISTORY)


def test_steering_fraction_and_report_text():
  log = read_log(drive(steering_until=SAMPLE + 3.2))
  result = score(log, packed(log), prediction(arc(10., 0., HORIZON) + [0., 30.]))
  assert result['openpilot_steering_fraction'] == pytest.approx(.5, abs=.02)
  text = render(result, {'timings_ms': {'total': 1336.07}, 'generated_tokens': 6, 'reasoning': ' Keep lane. '})
  assert "1336 ms, 6 reasoning tokens: 'Keep lane.'" in text and 'openpilot steering during the scored 6.4 s: 50%' in text
  assert 'flatters' in text and 'within 0% of the speed sensor' in text
  assert all(label in text for label in ('Alpamayo', 'hold speed and turn', 'openpilot plan'))
  assert render(score(log, packed(log), prediction(arc(40., 0., HORIZON)))).count('n/a') >= 2   # a prediction that outruns the driven path
  assert 'unknown' in render(dict(result, openpilot_steering_fraction=None))


def test_command_line_round_trip(tmp_path, capsys, monkeypatch):
  """rlog file, sample and report on disk, as after replay.py and benchmark.py on a truck whose camera speed reads 18% low."""
  events = drive(yaw_rate=.1, camera_scale=.82)
  rlog, sample_path, scaled_path = tmp_path / 'rlog', tmp_path / 'sample.npz', tmp_path / 'scaled.npz'
  rlog.write_bytes(b''.join(event.as_builder().to_bytes() for event in events))
  history = driven_path(read_log(events), SAMPLE, HISTORY)
  frames = np.arange(10 * 3 * 4 * 6, dtype=np.uint8).reshape(10, 1, 3, 4, 6)
  np.savez_compressed(sample_path, image_frames=frames, camera_indices=np.array([1] * 5 + [6] * 5, dtype=np.int64),
                      absolute_timestamps=np.array([round((SAMPLE + o) * 1e6) for o in (-2., -1., -.5, -.1, 0.)] * 2, dtype=np.int64),
                      ego_history_xyz=history[None].astype(np.float32), ego_history_rot=np.eye(3, dtype=np.float32)[None, None].repeat(20, axis=1),
                      nav_text=np.array('Continue straight'))

  def run(tool, *args):
    monkeypatch.setattr(sys, 'argv', [tool.__name__, *map(str, args)])
    tool.main()
    return capsys.readouterr().out

  # benchmark.py's report shape, made from the sample as packed
  native = tmp_path / 'native.json'
  native.write_text(json.dumps({'settings': {'input': str(sample_path)}, 'runs': [
    {'timings_ms': {'total': 1336.07}, 'generated_tokens': 6, 'reasoning': 'Keep lane.', 'xyz': prediction(.82 * arc(10., .1, HORIZON)).tolist()}]}))
  out = run(score_tool, '--rlog', rlog, '--input', sample_path, '--report', native)
  assert '18% below the speed sensor' in out and 'Note:' not in out

  out = run(rescale_tool, '--rlog', rlog, '--input', sample_path, '--output', scaled_path)
  assert 'Ego history rescaled' in out
  with np.load(sample_path, allow_pickle=False) as before, np.load(scaled_path, allow_pickle=False) as after:
    assert set(after.files) == set(before.files) and str(after['nav_text'].item()) == 'Continue straight'
    for key in ('image_frames', 'camera_indices', 'absolute_timestamps', 'ego_history_rot'):
      np.testing.assert_array_equal(after[key], before[key])
    np.testing.assert_allclose(after['ego_history_xyz'][0, :, :2], arc(10., .1, HISTORY), atol=3e-3)

  # reference.py's report shape, made from the rescaled sample
  reference = tmp_path / 'reference.json'
  reference.write_text(json.dumps({'settings': {'input': str(scaled_path)}, 'runs': [
    {'inference_ms': 96.51, 'generated_tokens': 6, 'reasoning': 'Keep lane.', 'xyz': prediction(arc(10., .1, HORIZON))[None, None].tolist()}]}))
  scores = tmp_path / 'out' / 'scores.json'
  out = run(score_tool, '--rlog', rlog, '--input', scaled_path, '--report', reference, '--output', scores)
  assert 'rescaled to the speed sensor' in out and '97 ms' in out

  def no_nan(name):
    raise ValueError(name)
  saved = json.loads(scores.read_text(), parse_constant=no_nan)
  assert saved['ego_motion'] == 'speed sensor' and saved['reasoning'] == 'Keep lane.'
  np.testing.assert_allclose(saved['rows']['alpamayo']['off_path_m'], 0., atol=5e-3)

  # the report of one sample scored against the other is called out
  assert 'Note: this report was made from scaled.npz, not sample.npz.' in run(score_tool, '--rlog', rlog, '--input', sample_path, '--report', reference)
