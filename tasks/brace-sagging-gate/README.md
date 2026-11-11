# Brace a sagging gate so it holds itself up

A tabletop model of a garden gate (about 1:5 scale) stands upright across the table in front of the robots. Its hinge-side stile is fixed to a
post; its two rails are pinned at all four corners, so the frame racks like a parallelogram. A steel weight on the
latch-side stile has pulled that side down 14 mm onto a rubber stop: the gate has sagged. A V-notch seat stands out
from the frame's front face in each inside corner, and a loose wooden brace (a 12 mm dowel with 15 mm ball ends) lies
on the table. The robot must fit the brace between two diagonally opposite seats so that the gate holds itself up,
with the latch side within 3 mm of square, and let go of the gate and the brace.

Hanging and bracing gates is fence and carpentry work: O*NET 47-4031.00, Fence Erectors, lists "Assemble gates, and
fasten gates into position, using hand tools." Which way the brace runs is the trade point the task tests. A fencing
supplier's guide puts it plainly: "The correct Z pattern has the diagonal running from the bottom of the hinge side to
the top of the latch side." "This orientation places the brace in compression and moves the load toward the hinge
side", and "If the brace is fitted the opposite way, the timber is placed under tension rather than compression"
([Jacksons Fencing, Which way should a garden gate be braced](https://www.jacksons-fencing.co.uk/the-edit/which-way-should-a-garden-gate-be-braced)).

A loose brace can only push. As the gate sags, the diagonal from the bottom hinge-side corner to the top latch-side
corner gets shorter (by 6.4 mm at this gate's 14 mm sag) and the other diagonal gets longer (by 7.2 mm), so:

- On the wrong diagonal (top hinge-side to bottom latch-side corner) the brace drops in easily, because that diagonal
  has grown, but it does nothing. The gate stays on its stop.
- On the right diagonal the brace is too long to go in while the gate sags. One arm has to lift the latch side square,
  the other fits the brace, and the gate is lowered onto it. Then the gate's weight pushes the brace's ends into their
  seats and the brace holds the gate up.

The physics decides the outcome, not a rule check: the score measures where the latch side ends up, with the robot
clear.

What this tests that other tasks don't:

- Squaring a racked frame (#1389, frame-square-rack: "Make it square, so that its two corner-to-corner diagonals are
  equal, keep it on its outline, then let go of it") is done on a frame lying flat, with no load and nothing that has to
  hold it square afterwards.
- Squaring a racked gantry beam (#419) is a coordinated two-ended pull against a jam.
- Lifting a load with one arm while the other slides a support under it (#338, lever-crate-dunnage: "Press the red
  lever's handle down to lift the front of the blue crate and hold it there, slide the wooden block under the crate's
  raised front edge with the other arm, then let go of the lever") and #1324 (lever-lift-and-chock-heavy-block: "Press
  down the long end of the grey lever bar to lift the red block, slide the orange chock under the red block, then let
  the lever go so the block rests on the chock") have the same lift-hold-insert motion. But there the support's place
  is given, it goes straight under the load, and anywhere under it works.
- Choosing where a member goes so it holds a structure (#880, plumb-guyed-mast-on-pegs: "put each guy's ring eye on
  the ground peg on its own side that holds the mast plumb") uses guys in tension, picked by which peg plumbs the mast.

Here the frame is under load, both seat pairs accept the brace, and the robot has to choose the one member position in
which that load keeps the frame square, against a tempting wrong one that fits more easily. The member can only push,
so the choice is compression against tension. The lift-and-hold while the other arm fits the brace is
needed only for the right diagonal; the wrong diagonal needs no hold and still scores 0.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the gate (a fixed-base articulation with four corner pins), the stop, the brace, `success()` and `score()` |
| `partkit.py` | Rigid parts built from boxes, cylinders and spheres, with revolute pins and a loop-closing joint, for yam_arena task files |
| `demo.py` | The scripted full-score demo (closed loop on the measured sag and brace pose) and its failure variants |
| `check_task.py` | Checks that every number in `task.json` and this README matches `task.py`, and the score bands' boundaries |
| `run2.py`, `run_common.py`, `compile.py`, `gate_rules.txt` | The gap test: the robot runner for every strategy, its helpers, the table builder, and the score rule fixed before the batch |
| `task_stage1.py`, `run.py` | The physics check before any robot work: the brace written into each diagonal's seats, no robot |

`partkit.py` hooks `yam_arena.scene` and the embodiment's step and reset without modifying yam_arena, so the task runs
with the standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The garden gate has sagged: its latch side has dropped onto the rubber stop. Fit the loose wooden brace between two of the corner seats so that the gate holds itself up, with the top of the latch-side stile no more than 3 mm from level with the top of the hinge-side stile, then let go of the gate and the brace." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/brace-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

Dream-1 scored 0.0 in its one run of this command and ran all 1,800 steps. Both arms reached in around the gate over
the run, but the brace was never picked up: in the last frame it still lies on the table in front of the gate, which
is still on its stop. (This run was scored before the release zone was widened to every part of the gate; the wider zone only makes the check stricter, and the brace was never seated.)

## Scripted demo

```bash
python demo.py task.py demo_out                                  # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=tens_nolift python demo.py task.py out --no-record  # failure variants: see the table below
BRACE_STATE=3 python demo.py task.py out --no-record             # another starting state (Gap test)
```

1. The right arm (the latch side's arm in the task's layout) grips the handle on the latch-side stile from above and
   lifts. It runs closed loop on the measured sag, moving the handle 0.4 mm per mm of error each step, until the latch
   side is 4 mm above square.
2. The left arm picks the brace up from the table at its middle and turns it to lie along the compression diagonal
   (the bottom hinge-side seat to the top latch-side seat). It brings the brace 70 mm in front of the seats, corrects
   its aim twice from the brace's measured pose, and pushes it in.
3. The right arm lowers the gate onto the brace, aiming for 3 mm below square, which the brace stops at 1.6 mm.
4. The left arm lets go and backs off. Then the right arm lets go of the handle, and both go back to where they started.

On the task's layout it scores 1.0 in 596 steps (the run limit is 1,800): the latch side ends 1.62 mm below
square, both brace ends in the bottom hinge-side and top latch-side seats, no robot point within 20 mm.

### Failure variants

Each was run on the task's layout through the same `task_score` path (`DEMO_VARIANT=... python demo.py task.py out
--no-record`):

| Variant | What it does | Score | Why |
|---|---|---|---|
| `comp_nolift` | Pushes the brace at the bottom hinge-side and top latch-side seats without lifting the gate | 0.0 | The brace is too long for that diagonal while the gate sags; it is not seated and the gate stays on its stop (sag 14.00 mm) |
| `tens_nolift` | Fits the brace on the other diagonal (top hinge-side to bottom latch-side seat), no lift | 0.0 | That diagonal has grown, so the brace goes in, but nothing pushes it into its seats; it drops out and the gate stays on its stop (sag 14.00 mm) |
| `tens_lift` | Lifts the gate square, fits the brace on the other diagonal, lowers the gate and lets go of both | 0.0 | As the gate drops, that diagonal grows; the brace cannot pull, drops out, and the gate goes back onto its stop (sag 14.00 mm) |
| `lift_only` | Lifts the gate square, holds it 3 s and lets go (no brace) | 0.0 | The gate drops back onto its stop (sag 14.00 mm) |
| `hold` | The correct fit, but the latch arm keeps hold of the handle to the end | 0.0 | Seated on the right diagonal, but the robot is touching the latch-side stile's handle (sag 1.97 mm, robot 0 mm) |
| `none` | Nothing moved | 0.0 | The gate stays on its stop (sag 14.00 mm) |

A robot that holds the gate up anywhere else (under a seat, by the frame, by the weight) is outside the release zone
by construction. The zone is measured to every box of the gate (both rails with their seats and back plates, both
stiles, the weight, the handle and the post), the stop and the brace. `check_task.py` puts a robot point on each face
of every one of those boxes and checks that it reads 0 mm. After review, this zone was widened from the brace, the
latch-side stile and the rails' main boxes to every part. That changes nothing in the runs above, whose robots ended
at least 20 mm from everything or, in `hold`, touching the handle; the demo and the correct runs were rerun on the
final code (see Gap test).

## Gap test

Before the demo, README or media were built, every strategy above was run with the robot on 10 starting states, under
a score rule written down before the batch (`gate_rules.txt`). The states vary which side the hinge is on (the gate is
mirrored, which swaps the arms' roles), the sag (12 to 16 mm), the frame's distance from the robots (±10 mm) and where
the brace lies (±10 mm in x, up to 20 mm in y). The rule (`gate_rules.txt`) is the score below with two differences.
It said "sag range over the last 1 s"; the batch runner sampled every 10 steps and checked the last 5 samples, which
span 2 s, a stricter check. And the bands were on the sag below square only; the final score bands its size either
way, which no run came near. `run2.py correct` is the same strategy code as `demo.py`'s full-score run.

```bash
BRACE_STATE=k python run2.py STRATEGY OUT_DIR       # k = 0..9; writes OUT_DIR/result.json
python compile.py RUNS_DIR                           # RUNS_DIR/s{k}_{strategy}/result.json -> table and mismatches
```

Result (score / latch side's sag in mm at the end):

| State | correct | comp_nolift | tens_nolift | tens_lift | lift_only | hold | none |
|---|---|---|---|---|---|---|---|
| 0 | 1.0 / 1.6 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 2.0 | 0.0 / 14.0 |
| 1 | 1.0 / 1.6 | 0.0 / 12.0 | 0.0 / 12.0 | 0.0 / 12.0 | 0.0 / 12.0 | 0.0 / 1.9 | 0.0 / 12.0 |
| 2 | 1.0 / 1.6 | 0.0 / 16.0 | 0.0 / 16.0 | 0.0 / 16.0 | 0.0 / 16.0 | 0.0 / 1.9 | 0.0 / 16.0 |
| 3 | 1.0 / 1.6 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 1.9 | 0.0 / 14.0 |
| 4 | 1.0 / 1.6 | 0.0 / 12.0 | 0.0 / 12.0 | 0.0 / 12.0 | 0.0 / 12.0 | 0.0 / 1.9 | 0.0 / 12.0 |
| 5 | 1.0 / 1.6 | 0.0 / 16.0 | 0.0 / 16.0 | 0.0 / 16.0 | 0.0 / 16.0 | 0.0 / 2.0 | 0.0 / 16.0 |
| 6 | 1.0 / 1.6 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 14.0 | 0.0 / 1.9 | 0.0 / 14.0 |
| 7 | 1.0 / 1.6 | 0.0 / 13.0 | 0.0 / 13.0 | 0.0 / 13.0 | 0.0 / 13.0 | 0.0 / 2.0 | 0.0 / 13.0 |
| 8 | 1.0 / 1.6 | 0.0 / 15.0 | 0.0 / 15.0 | 0.0 / 15.0 | 0.0 / 15.0 | 0.0 / 1.9 | 0.0 / 15.0 |
| 9 | 1.0 / 1.6 | 0.0 / 15.0 | 0.0 / 15.0 | 0.0 / 15.0 | 0.0 / 15.0 | 0.0 / 2.0 | 0.0 / 15.0 |

No mismatches against the expected scores (correct 1.0, every other strategy 0). The correct fit leaves the latch side
1.60 to 1.64 mm below square; every wrong strategy leaves it on its stop (12 to 16 mm), except `hold`, which is
seated at 1.9 to 2.0 mm but scores 0 because the robot still holds the handle.

After review widened the release zone (Failure variants), the final `task.py` and `demo.py` were rerun through
`demo.py` (`BRACE_STATE=k DEMO_VARIANT=... python demo.py task.py OUT --no-record`): every failure variant on state 0,
and the correct fit on all 10 states. The results matched the batch above. The correct fit scored 1.0 on every state,
taking 595 to 597 steps, and left the latch side 1.60 to 1.64 mm below square. `comp_nolift`, `tens_nolift`,
`tens_lift`, `lift_only` and `none` scored 0.0 with the gate on its 14.00 mm stop. `hold` scored 0.0, seated at 1.97 mm
with the robot 0 mm from the handle. The recorded demo (state 0, 596 steps, 1.0) is the demo video.

Before the robot work, a physics check (`run.py` with `task_stage1.py`, no robot) wrote the brace into each diagonal's
seats with the frame square and let go: on the compression diagonal the gate settled at 1.54 mm below square; on the
other diagonal, and with no brace, it sagged back onto its 14 mm stop.

## Scoring

`score()` at the end of the episode, all measurable with a steel rule and a photo:

- **0** unless all of these hold:
  - the brace sits in two diagonally opposite seats: each ball end rests in its notch, its centre within 3 mm of where
    it would touch both faces of the V, and between the seat's front edge and its back plate;
  - no part of either robot is within 20 mm of any part of the gate (frame, seats, weight, handle, post), the stop or
    the brace;
  - the gate is at rest: in the real world, measured 5 s after the robot lets go; in simulation, the latch side moved
    no more than 0.2 mm over the last second.
- Then, by how far the top of the latch-side stile sits from level with the top of the hinge-side stile:
  - within 3 mm: **1.0**;
  - within 6 mm: **0.5**;
  - within 9 mm: **0.2**;
  - further: **0**.

The episode ends early on success, once the score has been 1.0 for 1 s.

The outcome is close to binary by design: a seated compression brace holds the gate near square, and anything else
leaves it on its stop, so every scripted run scored 1.0 or 0. The partial bands are for real builds, where a brace cut
short or play in the pivots lets a correctly fitted brace stop the gate part way up.

Only the brace's position, the gate's position and the robot's distance count. Which arm lifts, which arm fits the
brace, the order of moves and the path don't.

## Runtime and seeds

The Dream-1 run above took 487 s of wall time for its 1,800 steps on one NVIDIA L40S with the GPU to itself. That
includes about 30 s of simulator start-up; Dream-1's action round trips took about 2.2 s each, as its log reports.
The scripted demo finishes in 596 steps. There are no random seeds: `object_jitter=0`, and the layout is fixed by `BRACE_STATE` (0, the task's
layout, by default).

## Fidelity

Rigid bodies built from boxes, cylinders and spheres.

- **The gate** is a fixed-base articulation. The hinge-side stile is welded to the world: it stands for a stile hung
  on hinges from a fixed post, which keep it plumb. The top and bottom rails are pinned to it, the latch-side stile is
  pinned to the top rail, and a loop-closing pin joins the bottom rail to the latch-side stile. So the frame is a
  pin-jointed parallelogram with light damping and no friction at its pins.
- **Corner joints:** a real gate's nailed or screwed corners resist racking a little on their own. These resist none,
  so the brace alone holds the gate. The 80 g steel weight on the latch-side stile stands in for the leverage of a
  real gate's own weight about its hinges.
- **The brace** is a loose push-fit member seated in V-notch blocks. A real brace is cut to fit and nailed, but it
  carries the load the same way, in compression on its ends; a loose brace cannot pull, so on the other diagonal it
  carries nothing.
- **Seats and fit:** the seats' V notches are 90 degrees with 22 mm plates; the brace's ball ends are 15 mm across. The
  brace is the square frame's seat-to-seat length less 0.8 mm clearance, so seated on the compression diagonal it stops
  the gate about 1.6 mm below square.
- **Friction** 0.6 (gate, brace) and 0.9 (rubber stop).

## Real-world build

Pine strips 15 x 20 mm:

- **Frame:** two stiles 150 mm long and two rails 220 mm long, joined at each corner by a single close-fitting bolt (a
  pivot) with the bolts 200 mm apart along the rails and 120 mm apart up the stiles.
- **Hinge side:** the hinge-side stile is clamped to a fixed upright post so it stays plumb.
- **Latch side:** an 80 g steel block is screwed to the latch-side stile's back face, and a 12 x 60 x 12 mm handle
  sticks out from its outer face, 100 mm above the table. A 30 x 30 x 16 mm rubber block glued to the table under
  the latch-side stile is the stop. It must be fixed: turned on edge it would stand 30 mm tall and hold the gate
  square by itself.
- **Seats:** each is two 22 x 35 x 3 mm pine plates glued at 90 degrees into a V, standing 35 mm out from the
  frame's front face, in each inside corner, opening along the diagonal, glued to the rail it sits on.
- **Brace:** a 12 mm dowel with 15 mm wooden balls on its ends, 183.9 mm between the balls' centres. Cut it to fit the
  squared frame's seats with no more than 0.8 mm to spare, as in the simulation. Calculated from the frame's geometry
  (not run): each 0.1 mm of slack along the brace lets the latch side drop about 0.2 mm, so 0.8 mm leaves it 1.66 mm
  below square. The brace's clearance plus play in the pivots must stay under about 1.4 mm in total for a correct fit
  to score 1.0.

Everything is light and below 20 cm, so trials are safe. Reset: take the brace out and lay it back on the table; the
gate drops back onto its stop.
