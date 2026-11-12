#!/usr/bin/env python3
"""Rewrite a packed sample's ego history in the scale of the car's own speed sensor.

replay.py builds the ego history from deviceMotion, whose speed is estimated from the cameras.
Where that disagrees with the car (score.py reports by how much), the model is told the wrong
speed. This keeps the directions and takes the distances from carState. Not part of comma's
prototype; see FORK_NOTES.md.
"""
import argparse
from pathlib import Path

import numpy as np

from openpilot.tools.alpamayo.score import HISTORY, driven_path, read_log
from openpilot.tools.lib.logreader import LogReader


def rescale(log, sample):
  query = float(np.asarray(sample['absolute_timestamps']).reshape(-1)[-1]) / 1e6
  camera = driven_path(log, query, HISTORY)
  packed = np.asarray(sample['ego_history_xyz'])
  assert np.abs(camera - packed.reshape(20, 3)).max() < .02, 'Sample was not packed by replay.py from this rlog, or is already rescaled'
  sensor = driven_path(log, query, HISTORY, wheel_speed=True)
  camera_m, sensor_m = (float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()) for path in (camera, sensor))
  return dict(sample, ego_history_xyz=sensor.reshape(packed.shape).astype(packed.dtype)), camera_m, sensor_m


def main():
  parser = argparse.ArgumentParser(description='Give a packed sample an ego history in the scale of the car\'s own speed sensor.')
  parser.add_argument('--rlog', required=True, help='rlog the sample was packed from')
  parser.add_argument('--input', required=True, type=Path, help='sample packed by replay.py')
  parser.add_argument('--output', required=True, type=Path)
  args = parser.parse_args()
  with np.load(args.input, allow_pickle=False) as data:
    sample = {key: data[key] for key in data.files}
  scaled, camera_m, sensor_m = rescale(read_log(LogReader(args.rlog)), sample)
  args.output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(args.output, **scaled)
  print(f'Last 1.9 s: camera-based ego motion covers {camera_m:.2f} m, the speed sensor {sensor_m:.2f} m. Ego history rescaled.')
  print(args.output.resolve())


if __name__ == '__main__':
  main()
