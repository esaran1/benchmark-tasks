# Put a driveshaft's slip yoke back on in phase

A driveshaft trainer stands upright on the table in front of the robots: at its bottom the far yoke (two blue ears
holding an orange cross), then the shaft's tube, a stop collar and a 42 mm splined shaft with six splines. The slip
yoke, an orange 40 mm splined sleeve with two ears on top, stands on the table beside it. The robot must slide the slip
yoke back onto the splines, down to the collar, turned so that the shaft's output will turn evenly with its input, and
let go. The sleeve goes on at any of six turns, 60 degrees apart. Only two of them, with its ears in the same plane as
the far yoke's ears, are in phase.

A driveshaft is often taken apart at its slip yoke when it is removed for transmission, clutch or U-joint work, and the
mechanic must put it back together in phase. Bus and Truck Mechanics and Diesel Engine Specialists (O*NET 49-3031.00) "Disassemble
and overhaul ... transmissions, clutches, and differential units"; Automotive Service Technicians and Mechanics
(49-3023.00) "Repair or rebuild transmissions"; Farm Equipment Mechanics and Service Technicians (49-3041.00)
"Reassemble machines and equipment following repair". A driveline manufacturer's driveshaft manager told a truck-equipment workshop that "a
problem occurs when mechanics remove the driveshaft by pulling it out of the slip yoke", and "By being off just one
tooth, the driveline can vibrate" ([Trailer/Body Builders, What every shop must know about drivelines, 2000](https://www.trailer-bodybuilders.com/distributors-upfitters/what-every-shop-must-know-about-drivelines)).
For tractor PTO shafts: "make sure the PTO driveline U-joints are in phase (or in line with each other) when
reassembling the two halves" ([Progressive Dairy / Ag Proud, Understanding power take-off drivelines](https://agproud.com/articles/32127-equipment-hub-understanding-power-takeoff-drivelines)).
A driveshaft maker's check: "The faces of the yokes ... are aligned in the same plane when properly phased"
([4xshaft, Is my drive shaft in phase?](https://4xshaft.com/blogs/faq/driveshaft-in-phase)).

What the task tests is choosing among positions that all fit. Every one of the six spline positions slides on and
seats against the collar just as firmly; nothing in the fit, the feel or the stop tells them apart. The right one is
set by a part about 10 cm away (the far yoke's ears, at the other end of the shaft) and by knowing why it matters: each U-joint
makes the shaft after it speed up and slow down twice a turn, and the second joint cancels the first only when the
yokes at the two ends of the shaft lie in one plane. The natural move, pushing the sleeve on at whatever position it
reaches first, fits and turns, but is out of phase in every starting state of this task. Shortcuts do not help
either: in the task's layout the far yoke's ears lie along the table's x axis and the slip yoke's start 70 degrees
from them, so squaring the slip yoke to the nearer table axis (90 degrees) puts it between spline positions, where it
does not go on, and picking one of the six positions at random is in phase one time in three.

Many shafts carry factory marks for this ("Sections of the shaft assembly have line-up arrows to assist in phasing",
[Modern Driveline, Tech Talk: driveline vibration](https://www.moderndriveline.com/resources/?p=50846)); this task is
the unmarked case, the one a driveshaft maker's FAQ lists as "You accidentally pulled the slip yoke off the spline and
don't know which way to put it back on" (4xshaft, above). The phase can be read from a photo taken straight down: the
two yokes' ears have the same size and the same place on their radius (6 mm thick, 22 to 28 mm from the axis, 18 mm
wide), so in phase the orange ears hide the blue ears below them completely, and out of phase all four show.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, for yam_arena task files |
| `demo.py` | The scripted full-score demo (closed loop on the measured slip-yoke pose), its failure variants, and the gap test's batch mode (`--variants`) |
| `stage1.py`, `run_common.py` | Drop test without the robot: the sleeve released over the shaft at each spline position and between them |
| `check_task.py` | Checks that every number in `task.json` and this README matches `task.py` and `demo.py`, and the score table's boundaries |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Slide the slip yoke back onto the driveshaft's splined shaft, all the way down to the collar, turned so that the shaft's output will turn evenly with its input, then let go of it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/phase-slip-yoke-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

Dream-1 scored 0.0 in its one run of this command, running all 1,200 steps: both arms moved, the left one in over the slip
yoke and the trainer and back, but the slip yoke was never lifted; in the last frame it still stands where it started,
beside the trainer, and in the overhead camera it never moved more than about a pixel.

## Scripted demo

```bash
python demo.py task.py demo_out                                  # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=nearest python demo.py task.py out --no-record      # failure variants: see the table below
DRIVE_STATE=3 python demo.py task.py out --no-record             # another starting state (Gap test)
```

1. The script reads the slip yoke's pose and picks the in-phase position nearer its starting turn (its ears on the far
   yoke's ear line; 0 and 180 degrees are the same here, as the ears are symmetric). It plans the left arm's wrist turn
   at pick-up and at the shaft so that the grip closes across the sleeve between its ears, both wrist angles are ones
   the arm reaches well, and the turn on the way is the smallest that brings the ears round.
2. The left arm grips the sleeve from above, its tool point 28 mm above the sleeve's bottom, lifts it to 155 mm above
   the table and carries it over the shaft, turning its wrist on the way.
3. Above the shaft it reads the sleeve's pose and moves so the sleeve's axis is over the shaft's and its ears point the
   chosen way: once 10 mm higher, then three times with the sleeve's bottom 3 mm above the splines' top (the hand lags
   the command by a few tenths of a mm while moving).
4. It pushes the sleeve down at 8 mm/s, aiming its bottom 1 mm below the collar's top; the bevel on the bore's bottom
   edge centres the sleeve on the splines' tips and it slides on. If the sleeve stops more than 3 mm above the collar
   (its bore's edge or a land resting on a spline's top), the arm lifts 2 mm, re-centres it from its measured pose and
   pushes again, at most twice; if it is still stuck, the arm backs off 2 mm and lets go there.
5. It waits 0.5 s, opens the gripper, lifts 60 mm and goes back to where it started.

The grip closes across the sleeve between its ears, and the wrist stays between -150 and +60 degrees of turn; an
offline IK check of every pick-up and shaft pose of every strategy on all 10 starting states found them all reachable
within 0.1 mm and 0.1 degrees.

The demo reaches 1.0 (success) in 451 steps (22.6 s), the slip yoke seated on the collar 0.72 degrees from the far
yoke's ear line, with IK within 0.1 mm of every target. Run on all 10 starting states (`DRIVE_STATE=0` to `9`) with
this `demo.py` and `task.py`, it reached 1.0 (success) on every one, in 451 or 452 steps, phases 0.38 to 0.98 degrees;
the push never stalled.

### Failure variants

Each variant is the demo with the slip yoke turned somewhere else or not let go, run on the task's layout
(`DRIVE_STATE=0`) with this `demo.py` and `task.py`:

| `DEMO_VARIANT` | What it does | Phase (deg) | Bottom above the collar (mm) | Score |
|---|---|---|---|---|
| (none: the demo) | In phase (ears on the far yoke's ear line), pushed to the collar, let go | 0.72 | 0.0 | 1.0 |
| `nearest` | Turned to the spline position nearest its starting turn (60 degrees from in phase), pushed on, let go | 59.37 | 0.0 | 0.0 |
| `other_wrong` | Turned to the other out-of-phase position (120 degrees), pushed on, let go | 60.88 | 0.0 | 0.0 |
| `misaligned` | Turned 8 degrees off the in-phase position (inside the window, between spline positions), pushed, let go | 7.41 | 36.0 (on the spline tops) | 0.0 |
| `hold` | The demo, but the arm keeps hold of the slip yoke to the end | 0.06 | 0.2 | 0.0 (robot 0 mm from it) |
| `none` | Nothing moved | 70.0 | on the table | 0.0 |

Every variant scores what the Scoring table says it should. `nearest` and `other_wrong` seat on the collar exactly as
the demo does; only the phase tells them apart.

## Gap test

Before the demo video, README and media were made, six strategies were run with the robot on 10 starting states:
the task's layout (state 0) and nine others with the shaft moved up to 10 mm in x and y, the far yoke's ears turned
from -50 to +55 degrees, and the slip yoke started at a different place on the table with its ears 55 to 130 degrees
from the far yoke's (its nearest spline position is out of phase in every state). `DRIVE_STATE=k` selects them;
`_STATES` in `task.py` lists them. The score rule and the expected scores (correct 1.0, every other strategy 0) were
written down before the batch. Each cell is the trial's `task_score` from `demo.py --variants` (one process per state,
the scene reset between strategies), with the phase or, for `misaligned`, the slip yoke's height above the collar.
The gate versions differed from these only in the slip yoke's mass in `task.py` (120 g; now 42 g, the printed
part's) and in `demo.py`'s push, which let go of a stalled sleeve at once instead of re-centring it (with the lighter
sleeve, the `hold` run on state 0 stalled on the splines' tips; re-centring was added for that).

| State (shaft dx, dy mm; far-yoke ears deg; slip-yoke ears deg from them) | `correct` | `nearest` | `other_wrong` | `misaligned` | `hold` | `none` |
|---|---|---|---|---|---|---|
| 0 (0, 0; 0; 70) | 1.0 (0.7 deg) | 0.0 (59.3 deg) | 0.0 (61.0 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 1 (10, 0; 20; 110) | 1.0 (0.5 deg) | 0.0 (60.7 deg) | 0.0 (59.2 deg) | 0.0 (37 mm up) | 0.0 | 0.0 |
| 2 (-10, 5; -25; 55) | 1.0 (1.7 deg) | 0.0 (59.3 deg) | 0.0 (60.9 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 3 (0, -10; 40; 125) | 1.0 (0.9 deg) | 0.0 (61.3 deg) | 0.0 (59.3 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 4 (5, 10; -40; 65) | 1.0 (1.4 deg) | 0.0 (59.5 deg) | 0.0 (60.9 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 5 (-5, -5; 10; 100) | 1.0 (0.8 deg) | 0.0 (60.8 deg) | 0.0 (58.5 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 6 (10, 10; 30; 130) | 1.0 (0.7 deg) | 0.0 (61.1 deg) | 0.0 (58.7 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 7 (0, 5; -15; 75) | 1.0 (0.7 deg) | 0.0 (59.0 deg) | 0.0 (60.9 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 8 (-10, -10; 55; 115) | 1.0 (0.2 deg) | 0.0 (61.0 deg) | 0.0 (58.9 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |
| 9 (5, 0; -50; 60) | 1.0 (1.7 deg) | 0.0 (59.2 deg) | 0.0 (61.0 deg) | 0.0 (36 mm up) | 0.0 | 0.0 |

The correct strategy scored 1.0 on all 10 (phase 0.22 to 1.70 degrees, seated on the collar, 451 to 470 steps). Every
wrong strategy scored 0 on all 10: `nearest` and `other_wrong` slid on and seated on the collar just like the correct
one, but 58.55 to 61.34 degrees out of phase; `misaligned` (8 degrees off the in-phase position, inside the phase
window but between spline positions) stopped on the spline tops 35.7 to 37.1 mm above the collar (in states 1 and 2
it tipped over 18 to 22 degrees when let go); `hold` was seated in phase with the gripper still on it; `none` left it
on the table.

The drop test without the robot (`DRIVE_STATE=0 python stage1.py out.json`), run with this folder's code, sets the
sleeve centred 5 mm above the splines' top at a given turn and lets it fall for 3 s. At all six spline positions it
seated (0.0 mm above the collar), reading 0.07 and 1.26 degrees at the in-phase ones and 56.90 to 62.52 at the others;
3 and -4 degrees off the in-phase position, inside the play, it also seated in phase (1.0); 15, 30 and 90 degrees off,
it stayed on the spline tops, 36.0 mm above the collar (0). The same test on all 10 states with the prototype's
tighter splines (before the bevel and the narrower lands) seated every spline position and left 3 and -4 degrees on the
tops.

## Scoring

Measured at the end with a ruler and a photo taken from straight above the shaft:

| The slip yoke at the end | Score |
|---|---|
| Phase 10 degrees or less, and seated | 1.0 |
| Phase 10 degrees or less, on the shaft but not seated | 0.5 |
| Phase more than 10 degrees; or not on the shaft (on the spline tops, on the table, tilted, upside down) | 0 |
| Any of the above while a robot is within 5 mm of the slip yoke, or while it moved 1 mm or more in the last second | 0 |

- **Phase:** the angle, seen from above, between the line through the slip yoke's two ears and the line through the far
  yoke's two ears, taken either way round (0 to 90 degrees; the ears are symmetric, so 0 and 180 are both in phase).
  On a photo from above, a protractor on the two ear lines. In the simulation, from the slip yoke's pose.
- **Seated:** the slip yoke's bottom within 3 mm of the collar's top (a ruler or a 3 mm feeler between them), its axis
  within 3 mm of the shaft's and within 5 degrees of vertical.
- **On the shaft:** at least 15 mm of the sleeve below the top of the splines, its axis within 3 mm of the shaft's and
  within 5 degrees of vertical.
- **Released and still:** no point of either robot within 5 mm of the slip yoke (`task.py` samples the robots'
  collision meshes every 2 mm, so any surface within 3.75 mm is always caught), and the slip yoke's bottom centre moved
  less than 1 mm over the last second.

The trial ends early once the score has been 1.0 for 1 s. A seated slip yoke is either near 0 or near 60 degrees: in the gap test the in-phase
positions read 0.22 to 1.70 degrees and the out-of-phase ones 58.55 to 61.34, and the splines' play (4.2 degrees each
way, calculated) keeps any seated yoke within 4.2 degrees of one or the other, so the 10-degree window sits in a wide
gap. The score reads only the
slip yoke's end pose and the robots' distance from it, so it does not depend on which arm does the work, the order of
moves or the path. The shaft and far yoke are fixed to the table, so turning the far yoke to meet the slip yoke is not
possible.

In the simulation the sleeve slides down the splines under its own weight once it is on them and let go, so the 0.5
band is reached only by a sleeve that jams partway down; none of the scripted strategies did. A seated yoke out of
phase is a defective job, so it scores 0 however well it is seated; the 0.5 band is for an in-phase yoke that stops
short, which a real printed sleeve can do by binding on its splines, while in the simulation it always slides home.

## Comparison with the nearest tasks

| PR | Its instruction (quoted) | What this task tests that it does not |
|---|---|---|
| #903 u-joint-cross | "Install the orange cross in the blue yoke: push one of its two flat journals through one ear's hole, swing the cross level so the other flat journal lines up with the other ear's hole, then slide it back until both journals are in their holes by the same depth." | The same machine element, a different job. #903 is assembling one U-joint (a manipulation trick: a cross longer than the gap between the ears). Here the joints are already built; the task is the relation between the two yokes of a shaft, which #903 never involves. |
| #192 seat-keyed-coupling-and-prove-drive-transfer (closed; resubmitted as #930) | "Hold the lower drive hub still by its long lever. Seat the orange coupling over both unequal drive dogs. Release the lower lever and turn the orange coupling through 90 degrees so the lower hub turns with it. Release both grippers. Leave the coupling seated. All equipment is unpowered." | A keyed coupling fits one way only, and the fit itself rejects the wrong turn. Here all six turns fit and seat equally well; the fit gives no signal, and the choice has to come from where the far yoke's ears point and why that matters. |
| #1343 diamond-pin-two-hole-location | "Locate the blue bracket on the fixture by its two holes: stand one round pin in one socket and the diamond pin in the other, turned so its two flat sides face the round pin, then lay the bracket flat on the fixture over both pins." | The pin's turn is stated in the instruction, and a wrong turn jams the bracket. Here no position jams and the instruction states only the outcome (an output that turns evenly with its input); the turn has to be worked out. |
| #1432 clock-split-shell-around-shaft | "Hold the blue shaft by its upper handle and turn the orange open shell around it, with the other hand, until the shell's white wall is parallel to the green bar; keep the shaft's black stripe facing forward." | Turning a part about a shaft to an orientation the instruction names (parallel to a named bar). Here the target is not named; it follows from what two U-joints do. |
| #1376 parallel-link-branch-close | "Fit the grey parallel link onto the red pin so the yellow arm's blue wrist plate sits level, then let go." | Also a choice between assembly positions that all fit, but there the result (the wrist plate level or not) is visible on the part being placed as soon as it is fitted, and the instruction names it. Here nothing visible changes between the six positions: the effect (an output that speeds up and slows down twice a turn) shows only when the shaft runs at an angle, so the robot has to know the phasing rule and read it off a part about 10 cm away. |

Other neighbours checked in the open and closed pull requests: #893 gear-train and #719 (turning a gear so its teeth
mesh: only the right turn fits at all), #233 index-rotary-table-and-pin and #829 dividing-head (indexing to a stated
angle or count), #856 sheave-straightedge-line-up (a continuous slide into one plane, checked with a straightedge).
None of the 18 merged tasks involves a spline, a driveline or phasing. DataCenterBench Tier 1, RoboDojo (insert tubes, key,
charger), LIBERO, RoboCasa, SimplerEnv and BEHAVIOR-1K have no driveline task that we found.

## Runtime and seeds

A full 1,200-step Dream-1 trial took 179 s from launch to exit on an L40S with nothing else on the GPU. The scripted
demo command took 163 s (451 steps, including startup and writing the videos) while another simulation shared the GPU.
The layout is fixed (`-E object_jitter=0`, `DRIVE_STATE` unset); there are no seeds to choose.

## Fidelity

The parts are rigid bodies built from boxes and cylinders. The trainer is one fixed body: the far yoke, its cross,
the tube, the collar and the splined shaft do not move, and no joint turns. Only the static result a mechanic checks
before refitting the shaft is scored, the phase (from the ear lines) and the seated depth; the shaft is never turned, so
the output's uneven speed itself is not simulated. For reference (calculated, not simulated): with both joints at 30
degrees in a Z layout and the exact Cardan relation, the output leads or lags the input by up to 0.00 degrees in phase,
7.38 degrees at 60 degrees out of phase and 8.21 degrees at 90; with both joints at 10 degrees, 0.00, 0.76 and 0.88.

Real slip yokes have many more splines than six: one supplier's 1310-series transmission slip yokes have 25 to
32 splines ([Action Machine, 1310 transmission slip yokes](https://www.actionmachineinc.com/collections/step-2-choose-your-1310-transmission-slip-yoke)),
and the driveshaft manager quoted above put one tooth at 22.5 degrees (that is, 16 splines). This trainer has six, 60 degrees
apart, for three reasons. Each wrong position is then 60 degrees out, so a photo and a protractor tell it from an
in-phase one at a glance: no seated position lies between 4.2 and 55.8 degrees (the play; calculated), so the
10-degree window is far from both. Six 6 mm splines are coarse
enough for the simulator's box contacts and the robot's grip to index reliably (the gap test's 60 runs above); 27
splines on a shaft this size would be about 1.3 mm wide (half the pitch at an 11.5 mm radius; calculated). And six is even, so, as on a real shaft with an
even count, there are two in-phase positions half a turn apart.

The splines are flat-topped boxes; the sleeve's bore has a 3 mm 45-degree lead-in bevel on its bottom edge, and its six
3.5 mm lands start 6 mm up, above the bevel, so the bevel centres the sleeve before the lands meet the splines.
Between lands and splines the play is 4.2 degrees each way (calculated): a land's inner corner, 10.64 mm from the
axis, meets a spline's flat side. The bevel and the
play are this task's choice, so that the robot's hand can find the splines. The trainer stands upright, so the sleeve slides down under its own weight once it is on
the splines; on a vehicle the shaft lies roughly level and the yoke is slid on by hand. The sleeve's outside is
twelve flat segments with small gaps at their outer corners. Friction 0.4 on both parts; the slip yoke weighs
42 g.

## Real-world build

Both parts are 3D printed in PETG from the dimensions in `task.json` (solid infill; the slip yoke then weighs about
42 g, calculated from its volume of 33.1 cm3). The trainer is printed as one piece (base plate, far yoke ears, cross,
tube, collar, splined shaft) and screwed to the table through its base plate with the shaft's axis at x 27, y 5 cm (from
the midpoint between the arm bases, x forward, y left) and the far yoke's ears along x, one spline on each ear's
direction. The slip yoke stands on the table at x 21.5, y 16 cm, ears up, its ears pointing 70 degrees from the far
yoke's ear line (turned from +x towards +y). A protractor sheet under the base plate, its zero on the far yoke's ear
line, makes the phase readable from the overhead photo. Reset: lift the slip yoke off and stand it back at its mark,
ears at 70 degrees. A real trial is safe: the slip yoke weighs 42 g, nothing is powered and nothing is sharp.
