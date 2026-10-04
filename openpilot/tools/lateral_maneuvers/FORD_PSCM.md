# Ford PSCM closed-course characterization

> [!WARNING]
> Closed course only. In Lateral Maneuver Mode openpilot turns the truck on its own, up to 2.5 m/s²
> of lateral acceleration and up to about 170° of heading change per maneuver, with nothing in the
> lot telling it where to go. Nobody on foot inside the lot, a spotter outside it, and your hands a
> few centimetres from the wheel the whole time.

This measures what the power steering module (PSCM) actually does with the `LateralMotionControl2`
path it is sent, so the tuning can be set from numbers instead of feel. It answers three questions:

| Run | Question | Changes on the truck |
| --- | --- | --- |
| A | At what commanded path angle, at each speed, does the PSCM start reporting its limit? | none |
| B | How much of the requested low-speed curvature does the truck deliver, and is the shortfall one gain? | none |
| C | Does requesting Extended path following (`LatCtl_D2_Rq = 2`) change either answer? | one-drive toggle |

A and B come from the same drive. C is a second drive with one toggle on. Nothing here changes
firmware or PSCM calibration.

## What you need

- A dry, flat, paved, empty lot or skid pad with a clear **110 m straight** and **80 m of width**
  beside it, plus run-off. Maneuvers do not start above 6.8° of roll, and a crowned or sloped lot
  biases the result.
- Cruise that holds **20 and 16 mph** hands off and feet off. Ford ACC will not set below 15 mph, so
  there is nothing slower in the suite.
- Uploads working, or a way to pull the rlogs off the device.

### How much room each maneuver takes

Traced from the start of the 2 s countdown, with the truck at speed and following the request
exactly. The real truck turns in late, so add about 15 m to the forward distance. Positive is a left
turn.

| Maneuver | Radius | Turns through | Forward | To the side | Time |
| --- | --- | --- | --- | --- | --- |
| hold ±1.0 m/s², 20 mph | 80 m | 37° | 81 m | 26 m | 8 s |
| hold ±2.0 m/s², 20 mph | 40 m | 103° | 60 m | 65 m | 10 s |
| hold ±2.5 m/s², 20 mph | 32 m | 146° | 52 m | 68 m | 11 s |
| hold ±1.0 m/s², 16 mph | 51 m | 56° | 66 m | 34 m | 9 s |
| hold ±1.5 m/s², 16 mph | 34 m | 106° | 50 m | 56 m | 11 s |
| hold ±2.0 m/s², 16 mph | 26 m | 168° | 42 m | 53 m | 13 s |
| sweep 0 → ±2.5 m/s², 16 mph | down to 20 m | 70° | 59 m | 31 m | 9 s |

The space is not used up by the maneuvers. It goes on the run-up: getting from a stop to 20 mph
takes about 55 m of this truck's distance before the countdown can even start. **Never stop between
maneuvers.** Drive a loop around the lot at the maneuver speed, so you arrive at the start of the
straight already at speed and pointing straight.

For a left maneuver (+), enter the straight along its right edge; for a right one (−), along its
left edge. Then the turn goes into the open width instead of toward the edge you are driving
beside. While it waits for speed, the screen names the next maneuver ("next: hold +2.0m/s^2 20mph").

## Settings before each drive

Set these while the truck is off (all under the Ford settings unless noted):

| Setting | Value | Why |
| --- | --- | --- |
| Correct Steering Shortfall | **off** | it scales the request, which is the thing being measured |
| Detect Steering Saturation | **off** | same |
| Low Speed Factor / High Speed Factor | leave as they are, **write them down** | the report reads them from the log and recommends from them |
| Experimental Mode, Dynamic Experimental Control | **off** | the model must not slow the truck mid-turn |
| Smart Cruise Control (Vision and Map), Speed Limit Assist | **off** | same; any slowdown over 1.6 mph aborts the maneuver |
| Developer > Lateral Maneuver Mode | **on** | clears itself at the end of the drive |
| Extended Mode Test (One Drive) | **off** for Runs A/B, **on** for Run C | clears itself at the end of the drive |

Both one-drive toggles are cleared when the truck goes offroad and when openpilot restarts, so they
cannot follow you onto a public road. Check the Ford settings page after each drive anyway.

## Holding speed

Cruise has to hold the speed. The pedals cannot:

- **Gas aborts the maneuver.** Pressing it to get from 15 to 20 mph also does nothing for the cruise
  set speed. When you let go, the truck slows back to whatever cruise is set to.
- **Brake cancels cruise.** Lateral stays engaged, so the countdown can still start, but the truck
  is now coasting. It drops out of the 1.6 mph band within a second or two, and the maneuver
  aborts.

So: set cruise at 15 mph or above, then **tap +** until the set speed matches the screen. After any
brake press, press **RES** before the next maneuver. The 16 mph maneuvers come after all the 20 mph
ones; lower the set speed to 16 mph when the screen asks for it.

## How a maneuver runs

1. The screen says "Set speed to 20 mph" until cruise holds that speed with lateral engaged.
2. After **2 s** at speed within 1.6 mph, pointing straight and hands off, it counts down
   ("Starting: 2") and runs the maneuver.
   - Holds: a 0.5 s ramp into a constant turn, a hold sized so the wheel finishes turning in with
     3 s to spare (5 to 10 s), a 0.5 s ramp out, then 1.5 s commanded straight.
   - Sweeps: a 6 s ramp up to the peak, then 1 s back to straight.
3. "Complete" shows. Take the wheel, loop around at speed, line up, let go. The next maneuver arms
   itself.

Touching the wheel, touching the gas or leaving the speed band aborts the maneuver in progress. It
is repeated, not skipped, so abort whenever you want to; the only cost is time.

The suite, in order: 14 maneuvers, about 2½ minutes of maneuver time plus the loops. Every
maneuver runs in both directions, because the PSCM is not guaranteed to be symmetric. Every level
stays inside 90% of what the path-angle signal can carry at its speed, so the report measures the
module and not a clipped command.

## Run A + B: Limited (normal) mode

1. Extended Mode Test off, everything else as in the table above.
2. Run the whole suite.
3. Turn the truck off. Upload or copy the rlogs.

The report removes the yaw sensor's offset (about −0.0047 rad/s on every log from this truck so far)
using straight, engaged, hands-off driving. The countdowns supply that. If the report says "yaw
bias not estimated", it had less than 5 s of it.

## Run C: Extended mode A/B

Only after Run A/B came back with **no abort condition** in its report.

The PSCM on this truck reports `LatCtlCpblty = Extended available`. What Extended changes on this
module is not documented. It could be a wider authority envelope, a different rate limit, or a
different hand-off policy, and it may also be nothing at all. The panda applies exactly the same
angle, curvature and rate checks to mode 2 as to mode 1. That is covered by a test in
`opendbc/safety/tests/test_ford.py`.

1. Turn Extended Mode Test (One Drive) on. Nothing else changes.
2. Engage and drive straight at 20 mph for at least 30 s **before** starting the suite. Watch for:
   - any steering fault alert,
   - openpilot refusing to engage or dropping out,
   - the wheel moving when nothing asked it to.

   Any of those: disengage, stop, turn the truck off. The toggle is gone on the next drive. Send the
   logs, since that alone is the answer to Run C.
3. Run the whole suite again, same lot, same direction of travel if you can.
4. Turn the truck off.

Stop Run C on the spot for any of:

- a steering fault or "take control" alert,
- the wheel moving without being asked, or pulling toward one side on the straight line-ups,
- the truck not giving the wheel back when you take it,
- the truck noticeably fighting you when you take over. Taking over is how every maneuver ends, so
  this matters more than any number the report gives.

## Reading the results

On a PC with this checkout, for each run:

```sh
python openpilot/tools/lateral_maneuvers/ford_pscm_report.py <route ID or rlog paths> --csv run_a.csv
```

It prints four sections:

**HEALTH.** What the PSCM reported for each requested mode: `LatCtlSte`, `LatCtlCpblty`,
`LaActDeny`, steering faults, panda blocks, and frames where openpilot was engaged but had handed
the wheel back. Panda blocks are split in two. A few right at engage or disengage are the usual race
between the panda seeing the pedal and openpilot's next frame. Blocks before the panda switches to
Ford mode at startup are not counted. Any other block is an abort condition. Any `ABORT CONDITION`
line means the rest of that run is not trustworthy. For
Run C, `Denied`, `Faulty`, `Unavailable` or `LaActDeny` while Extended was requested means the
PSCM does not accept Extended from openpilot. That is the answer; stop there.

**LIMIT ONSET.** For each speed band (under 9 m/s, 9–18 m/s, over 18 m/s) and requested mode: how
often `LatCtlLim` said NotReached / Close / Reached against commanded path angle in 1° buckets. The
logs so far say the answer depends on speed. Below 9 m/s the module never flagged anything up to
10.8°. At 18 m/s and above it flagged Close from 3–4°. A single onset angle blended across speeds
is not a real number, which is why the table is split.

**STEADY-STATE DELIVERY.** One line per hold: requested curvature, curvature the truck actually
drove (from yaw rate), their ratio, path angle, wheel angle, and whether it **settled**: whether the
truck had stopped turning in by the end of the hold. A hold that did not settle measures lag, not
delivery. It is listed but left out of the recommendation. Then, for the settled low-speed holds (at
or below 13.5 m/s, where the Low Speed Factor applies, so both 16 and 20 mph), the median ratio, its
spread, and the Low Speed Factor that would cancel it. What to do with it:

| What the report shows | What it means | Action |
| --- | --- | --- |
| Ratio about the same at every level (IQR ≤ 0.15) | the shortfall is a gain | set Low Speed Factor to the recommendation, then repeat Run A/B once to confirm the ratio is now about 1.0 |
| Ratio falls as the level rises, or "spread is wide" | the module stops following as the demand grows | no factor fixes this. A bigger factor only reaches the limit sooner. Leave the factor, keep the data |
| Most holds at one level say NO under settled | the truck turns in slower than the 30°/s the holds are sized for | send the log; the holds get longer, and so does the room they need |
| Left and right holds differ by more than 0.1 | asymmetric module, or a sloped lot | repeat on a flat lot before changing anything |
| Extended ratio clearly higher than Limited, with a clean HEALTH | Extended changes authority | a separate decision about making it a real option. Do not leave the test toggle on |

**RESPONSE.** Per maneuver: lag from request to truck at 50% and 90% of the step, overshoot, and
peak wheel rate. Compare Limited against Extended on the same maneuver, not across maneuvers.

`--csv` writes every steering frame (request, path angle, measured curvature, wheel angle, PSCM
status, maneuver label) if you want to plot something the report does not.

## What this deliberately does not do

- It does not touch PSCM firmware or calibration. Raising the module's authority in its calibration
  is a different project with a different risk class.
- It does not add a "d_ref" or any other model of the PSCM into the controller. The angle-mode
  command here is `path_angle = κ·v·gain`. Any correction comes from the measured delivery ratio,
  applied through the existing speed factors, and only after this test says a single gain explains
  the shortfall.
- It does not make Extended mode a normal setting. That waits for a clean Run C.
