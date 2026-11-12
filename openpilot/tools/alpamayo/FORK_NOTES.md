# Alpamayo 2 Edge on this fork

> [!WARNING]
> Offline benchmark only. Nothing here drives the truck. On this branch modeld, controls and the
> panda are the same as on `dev`, and no openpilot process imports the files in this directory.

This directory is comma's offline prototype for running
[NVIDIA Alpamayo 2 Edge](https://huggingface.co/nvidia/Alpamayo2-Edge) on a Chestnut. It is
imported unchanged from `commaai/openpilot`, branch `alpamayo2-edge-egpu`, at `fe68e1d`.
`README.md` is comma's and is the procedure they tested. This note covers only what is different
on this fork.

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

## Status on this fork

Imported and read, not run. The fork's lint passes on it, the files are byte-identical to comma's,
and everything `replay.py` uses from openpilot exists in this tree under the same names
(`FrameReader`, `LogReader`, `rot_from_euler`, and the `narrowRoadEncodeIdx`, `wideRoadEncodeIdx`,
`deviceMotion` and `extrinsicsCalibration` messages). Nobody has executed it from this fork, on a
Chestnut or anywhere else.
