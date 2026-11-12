# Alpamayo 2 Edge on this fork

> [!WARNING]
> Offline benchmark only. Nothing here drives the truck. On this branch modeld, controls and the
> panda are the same as on `dev`, and no openpilot process imports the files in this directory.

This directory is comma's offline prototype for running
[NVIDIA Alpamayo 2 Edge](https://huggingface.co/nvidia/Alpamayo2-Edge) on a Chestnut. It is
imported unchanged from `commaai/openpilot`, branch `alpamayo2-edge-egpu`, at `fe68e1d`.
`README.md` is comma's and is the procedure they tested. This note covers what is different on
this fork: where tinygrad comes from, and three files added here (`score.py`, `rescale.py` and
their tests in `test_score.py`).

## What it does and does not do

It takes one logged segment and gives the model two cameras (wide as its "front" camera, road as
its "front telephoto"), two seconds of ego motion and a navigation sentence. It reports the
predicted 6.4 s trajectory, the model's reasoning text and the time each stage took.

comma's own result on a comma four with a Chestnut (8 GB, 100 W limit), one sample, FP8 weights:
1.34 s per inference, which is 0.75 per second. Their stated target is 5 per second, and
openpilot's driving model runs 20 times per second. NVIDIA specifies six cameras for this model.
The two-camera input is outside that specification, and comma's README says a successful
inference does not establish driving quality.

## The one difference: tinygrad

comma's branch moves the `tinygrad_repo` submodule to `commaai/tinygrad` `89846dcf`, which is
tinygrad master of 2026-11-02 plus a two-line USB fix the benchmark needs. This fork does not
follow it. The driving models in this tree are built for the tinygrad pinned here, and in October
the big model stopped loading because the two did not match.

So the tool gets comma's tinygrad from a separate checkout that comes first on `PYTHONPATH`. In
the Setup section of `README.md`, use this in place of the `export PYTHONPATH=...` line:

```bash
git clone https://github.com/commaai/tinygrad /data/alpamayo/tinygrad
git -C /data/alpamayo/tinygrad checkout 89846dcfb383a98ee01cfec3bba440ae7dcf9b1e
export PYTHONPATH="/data/alpamayo/tinygrad:$PWD:$PWD/opendbc_repo:$PWD/msgq_repo"
```

## On the truck

- The Chestnut serves one process at a time, and modeld takes it whenever the truck is onroad, so
  the benchmark has to run while modeld is not running. comma's README does not say how they
  arranged that. `sudo systemctl stop comma` is how this repository's `tools/op.sh` stops
  openpilot, and `sudo systemctl restart comma` starts it again.
- Put the Python environment, the 5.3 GB checkpoint and the compiler cache under `/data`. comma's
  README uses `/tmp` paths and notes that the comma four's small `/home` overlay filled up during
  compilation.
- Your own drives are already on the device. Each `/data/media/0/realdata/<route>--<segment>/`
  holds the `rlog.zst`, `fcamera.hevc` and `ecamera.hevc` that `replay.py` takes.

## Scoring a run

comma's tool reports how fast the model runs and prints a trajectory. `score.py`, added on this
fork, says how that trajectory compares with what the truck did in the next 6.4 s of the same log.
It needs only numpy and openpilot's log reader, so it runs in either Python environment.

```bash
python openpilot/tools/alpamayo/score.py --rlog /absolute/path/rlog.zst \
  --input /tmp/alpamayo-openpilot.npz --report /tmp/alpamayo-chestnut.json
```

If the sample is within 6.4 s of the end of its segment, add the next segment's rlog with a second
`--rlog`. `--output scores.json` also saves the numbers.

| Line in the output | Meaning |
| --- | --- |
| Off the driven path | Sideways distance from the line the truck drove, + is left. This is the steering result. |
| Ahead of the truck | How much farther along that line the prediction is than the truck was. This is speed, not steering. |
| Alpamayo | The model's trajectory. |
| hold speed and turn | The truck carrying on exactly as in its last second. A model has to beat this to be worth anything. |
| openpilot plan | What openpilot's own model planned from the same camera frame. |

Three things to keep in mind when reading it:

- If openpilot was steering during those 6.4 s, the truck followed openpilot's plan, so that row
  looks better than it deserves. The output prints how much of the window openpilot was steering.
  Use stretches you drove by hand to compare the two models fairly.
- One sample is an anecdote. Score a few dozen, curves included, before concluding anything. Each
  one needs its own `replay.py` and `benchmark.py` run, and the benchmark reloads the model each
  time.
- The path is the device's, as in comma's packer, not the rear axle's the model was trained on.
  In a tight turn the two differ a little.

## The truck's speed

comma's packer takes the ego motion from `deviceMotion`, and the localizer behind that message
observes speed only through the cameras. On this truck that speed reads low. In the one log
checked (build `9386beb`, the first 19 s of a drive) the camera-based path covered 51.6 m in 6.4 s
where the truck's speed sensor gives 62.8 m and GPS 64.4 m: 18% short. A model told the truck is
doing 8.4 m/s when it is doing 10.2 m/s is being tested on a wrong input.

`score.py` checks this on every sample and says so. When it does, fix the sample and benchmark the
fixed one:

```bash
python openpilot/tools/alpamayo/rescale.py --rlog /absolute/path/rlog.zst \
  --input /tmp/alpamayo-openpilot.npz --output /tmp/alpamayo-openpilot-scaled.npz
```

`rescale.py` keeps the directions and takes the distances from `carState`. `score.py` recognises a
rescaled sample and scores it in the truck's own scale. Whether the 18% holds later in a drive, or
on other trucks, is not known: one short log.

## Status on this fork

comma's tool is imported and read, not run. The fork's lint passes on it, the files are
byte-identical to comma's, and everything `replay.py` uses from openpilot exists in this tree under
the same names (`FrameReader`, `LogReader`, `rot_from_euler`, and the `narrowRoadEncodeIdx`,
`wideRoadEncodeIdx`, `deviceMotion` and `extrinsicsCalibration` messages). Nobody has executed it
from this fork, on a Chestnut or anywhere else.

`score.py` and `rescale.py` have unit tests (`test_score.py`) and were run on a real log from this
truck for the driven path, the speed check and openpilot's plan. They have never seen a real
Alpamayo trajectory, only stand-ins. They rebuild the ego history the way `replay.py` does and
refuse to score if it does not match the packed sample, so a change in comma's packer shows up as
an error, not as a wrong number.
