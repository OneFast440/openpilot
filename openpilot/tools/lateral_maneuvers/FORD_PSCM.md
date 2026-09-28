# Ford PSCM closed-course characterization

> [!WARNING]
> Closed course only. In Lateral Maneuver Mode openpilot turns the truck on its own, up to 2.5 m/s²
> of lateral acceleration and about 70° of heading change per maneuver, with nothing in the lot
> telling it where to go. Nobody on foot inside the lot, a spotter outside it, and your hands a few
> centimetres from the wheel the whole time.

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

- A dry, paved, empty lot or skid pad, at least about **150 m × 100 m** with run-off past that. The
  tightest maneuver is a 19 m radius at 12 mph; the widest is an 80 m radius at 20 mph that travels
  about 70 m forward and 15 m sideways. You also need room to loop back and line up for the next one.
- Flat. Maneuvers do not start above 6.8° of roll, and a crowned or sloped lot biases the result.
- Cruise has to hold **20, 16 and 12 mph** hands off and feet off: the gas pedal aborts a maneuver
  and the brake disengages. On this truck the set speed comes from the truck's own cruise, so check
  in the lot, before anything else, that you can set 12 mph and that it holds within 1.6 mph. If it
  cannot, the 12 mph maneuvers never start. They run last so that nothing else is lost.
- Uploads working, or a way to pull the rlogs off the device.

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

## How a maneuver runs

1. Engage, set cruise to the speed on screen ("Set speed to 20 mph") and let the truck roll straight.
2. After **2 s** engaged, at speed within 1.6 mph, pointing straight and hands off, it counts down
   ("Starting: 2") and runs the maneuver: a 0.5 s ramp into a constant turn, a 4 s hold, a 0.5 s ramp
   out, then 1.5 s commanded straight.
3. "Complete" shows. Take the wheel, loop around, line up, let go. The next maneuver arms itself.

Touching the wheel, touching the gas or leaving the speed band aborts the maneuver in progress. It
is repeated, not skipped, so abort whenever you want to; the only cost is time. Braking disengages.

The suite, in order (16 maneuvers, about 4 minutes of maneuver time plus the loops):

| Speed | Maneuvers | Radius |
| --- | --- | --- |
| 20 mph | hold ±1.0, ±2.0, ±2.5 m/s² | 80, 40, 32 m |
| 16 mph | hold ±1.0, ±2.0 m/s²; sweep 0 → ±2.5 m/s² over 6 s | 51, 26 m, down to 20 m |
| 12 mph | hold ±1.0, ±1.5 m/s² | 29, 19 m |

Every maneuver runs in both directions, because the PSCM is not guaranteed to be symmetric. Every
level stays inside 90% of what the path-angle signal can carry at its speed, so the report
measures the module and not a clipped command.

## Run A + B: Limited (normal) mode

1. Extended Mode Test off, everything else as in the table above.
2. Drive two minutes of plain straight engaged driving at 20 mph before the first maneuver. The
   report uses straight, engaged, hands-off frames to remove the yaw sensor's offset (about
   −0.0047 rad/s on every log from this truck so far). The line-ups between maneuvers add more.
3. Run the whole suite.
4. Turn the truck off. Upload or copy the rlogs.

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
the wheel back. Any `ABORT CONDITION` line means the rest of that run is not trustworthy. For
Run C, `Denied`, `Faulty`, `Unavailable` or `LaActDeny` while Extended was requested means the
PSCM does not accept Extended from openpilot. That is the answer; stop there.

**LIMIT ONSET.** For each speed band (under 9 m/s, 9–18 m/s, over 18 m/s) and requested mode: how
often `LatCtlLim` said NotReached / Close / Reached against commanded path angle in 1° buckets. The
logs so far say the answer depends on speed. Below 9 m/s the module never flagged anything up to
10.8°. At 18 m/s and above it flagged Close from 3–4°. A single onset angle blended across speeds
is not a real number, which is why the table is split.

**STEADY-STATE DELIVERY.** One line per hold: requested curvature, curvature the truck actually
drove (from yaw rate), their ratio, path angle, wheel angle. Then, for the low-speed holds (at or
below 13.5 m/s, where the Low Speed Factor applies), the median ratio, its spread, and the Low
Speed Factor that would cancel it. What to do with it:

| What the report shows | What it means | Action |
| --- | --- | --- |
| Ratio about the same at every level (IQR ≤ 0.15) | the shortfall is a gain | set Low Speed Factor to the recommendation, then repeat Run A/B once to confirm the ratio is now about 1.0 |
| Ratio falls as the level rises, or "spread is wide" | the module stops following as the demand grows | no factor fixes this. A bigger factor only reaches the limit sooner. Leave the factor, keep the data |
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
