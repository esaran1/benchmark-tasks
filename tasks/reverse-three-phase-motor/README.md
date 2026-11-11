# Reverse a backwards-running three-phase conveyor motor

A short belt conveyor stands across the back of the table, driven by a three-phase motor through a gearbox. The motor's
terminal box faces the robots: three sockets, U, V and W from left to right, at table level, hold the plugs of three
supply leads, brown (L1), black (L2) and grey (L3), which come from a cable gland at the front of the table. The motor
was just replaced and the conveyor now runs backwards, against the arrows on its yellow guard. The robot must change
which socket each lead is plugged into so that the conveyor runs the way the arrows point, push every plug fully home,
and let go. The supply is off at the start. Once the robot has touched a plug or lead, a tester switches the supply on
whenever the robot has been clear of the terminal box, the gland and the leads for 1 s with all three plugs home: the
conveyor carries a blue tote over the green (forward) or the red (backward) line, and a pointer disc on the gearbox
turns.

Connecting and commissioning three-phase motors is electricians' and motor repairers' work. O*NET 47-2111.00,
Electricians, lists "Install ground leads and connect power cables to equipment, such as motors." O*NET 49-2092.00,
Electric Motor, Power Tool, and Related Repairers, lists "Inspect electrical connections, wiring, relays, charging
resistance boxes" and "Rewire electrical systems, and repair or replace electrical accessories." Checking a new motor's
direction and correcting it is part of every installation. The rule the task tests is standard: "The direction of
rotation of three-phase motors can be reversed by interchanging any two phases"
([Wikipedia, Three-phase electric power](https://en.wikipedia.org/wiki/Three-phase_electric_power)).

## What the robot has to work out

A three-phase motor turns the way its phase sequence goes round: L1, L2, L3 on U, V, W turns one way. Exchange any two
leads and the sequence reverses (L1, L3, L2), so the motor reverses. Moving all three leads one socket along (L3, L1,
L2 on U, V, W) looks like a bigger change but keeps the same cyclic order, so the motor still runs backwards; two
swaps in a row leave an even order too. In terms of the lead order, an odd permutation of the start reverses the motor and an even one does
not. Nothing in the scene says which lead "should" go where: the start order is whatever the backwards motor was given
(in the task's layout it is the colour-coded order brown, black, grey on U, V, W, so tidying the leads into colour
order changes nothing).

What this tests that other tasks don't:

- #26 (place-ecg-leads-correct-electrodes, merged: "Following the AHA colour code, drop each lead's clip into the white
  electrode cup at the correct body position for that lead") puts each lead on its own place by a colour code: a lookup.
  Here no colour code gives the answer; the robot must infer what a change of order does.
- #22 (plug-both-ends-into-tester, merged: "push one plug fully into the jack of the main unit ... and the other plug
  fully into the jack of the remote unit") and #20 (reseat-half-inserted-plug, merged: "Find the loose plug and push it
  fully into its port") test seating plugs. Here seating is only the motion; the score turns on which order the plugs
  end in.
- #1538 (star-delta, open: "Link this 230/400 V motor for its new 400 V supply") works in the same kind of terminal box,
  but its decision is the winding links for the supply voltage (star or delta). Here the links do not exist and the
  decision is the phase sequence of the supply leads.
- #136 (patch-cords-per-port-map: "push each plug fully into the numbered port listed for its colour") and #318
  (terminal-strip-to-plan: "fit the strip to the terminal plan") move plugs or terminals to a stated plan. Here there is
  no plan: any of the three possible swaps scores in full, and the tempting "move them all along" fails.
- #1345 (quad-props-by-rotation) matches a part to a motor's given direction; here the robot changes the direction.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: the drive (terminal box, motor, gearbox, conveyor, tote, pointer disc), the gland, the three leads, the tester, `success()` and `score()` |
| `cablekit.py` | Flexible cables (chains of rigid segments with bending springs) with plugs, for yam_arena task files; this copy adds a bent starting shape |
| `partkit.py` | Rigid parts built from boxes and cylinders, with sliders and pins, and per-step hooks, for yam_arena task files |
| `demo.py` | The scripted full-score demo and its failure variants (also the gap-test runner) |
| `check_task.py` | Checks that every number in `task.json` and this README matches `task.py`, and the score at its boundaries |
| `gap/` | The gap test: the score rule fixed before the batch (`gap_rules.txt`), expected scores, the batch worker and the table builder |
| `media.sh` | Builds the videos and the setup image from a Dream-1 log and a recorded demo |

`cablekit.py` and `partkit.py` hook `yam_arena.scene` and the embodiment's step and reset without modifying
yam_arena, so the task runs with the standard `yam_arena` embodiment and `task_score` scorer. This copy of `cablekit.py`
adds `bends`: a lead can start bent (written into the articulation's joint state when the stage is parsed and again
after every reset), so a lead that starts plugged in can still have slack.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This conveyor's motor was just replaced and now runs backwards. Change which socket each supply lead is plugged into so that the conveyor runs the way the arrows on its guard point, with every plug pushed fully home, then let go. The supply is isolated; it is switched on once you are clear." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/motor-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

Dream-1 scored 0.0 in its one run of this command and ran all 1,800 steps. Its right arm reached in around the
terminal box and the gearbox for most of the run and the left arm moved over the table, but no plug was moved: in the
last frame all three plugs are still in their starting sockets (brown U, black V, grey W). That run was made before two
fixes described under Fidelity (plastic leads, and the supply staying off until the robot touches a plug or lead); in
it the conveyor ran backwards in the first seconds, before the robot reached in.

## Scripted demo

```bash
python demo.py task.py demo_out                                  # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=cyclic python demo.py task.py out --no-record       # failure variants: see the table below
MOTOR_STATE=3 python demo.py task.py out --no-record             # another starting state (Gap test)
```

1. The left arm pinches the plug in U 10 mm from its lead end, pulls it straight out and holds it up and to the left.
2. The right arm pinches the plug in W, pulls it out, lifts it over the middle lead and carries it to U. It lowers the
   plug in front of U, slides it to 2 mm short of the face, lines its tip up with the socket from the plug's measured
   position, pushes it 5 mm in (backing off and lining up again if it did not enter), then pushes it home to 24.5 mm,
   lets go and moves away.
3. The left arm carries its plug over to W and pushes it home the same way, lets go and moves away.
4. The robot is clear, so the tester switches the supply on: the conveyor runs forward and the tote crosses the green line.

DEMORESULT

### Failure variants

VARIANTS

## Gap test

Before the README, the full-score demo's recording or the media were made, the correct strategy and two wrong ones were
run with the robot on 10 starting states, under a score rule written down and committed before the batch
(`gap/gap_rules.txt`, the score below). The states change the starting order of the leads (all six orders of brown,
black and grey over U, V and W occur; each is by definition the order that runs backwards), the side the leads bow to,
the drive's position (up to 10 mm in x and y) and its yaw (up to 5 degrees). The batch ran a frozen copy of the task
folder through `demo.py --no-record`, the same `task_score` path as a trial.

```bash
bash gap/gworker.sh FROZEN_COPY OUT MOTOR_STATE "0 1 2 3 4 5 6 7 8 9" "correct cyclic none"
python gap/compile.py OUT gap/expected.json
```

| State | Start order (U, V, W) | correct | cyclic | none |
|---|---|---|---|---|
| 0 | brown, black, grey | 1.0 (826 steps) | 0.0 (1468 steps) | 0.0 |
| 1 | black, brown, grey | 1.0 (911 steps) | 0.0 (1280 steps) | 0.0 |
| 2 | grey, black, brown | 1.0 (826 steps) | 0.0 (1800 steps) | 0.0 |
| 3 | brown, grey, black | 1.0 (895 steps) | 0.0 (1376 steps) | 0.0 |
| 4 | black, grey, brown | 1.0 (972 steps) | 0.0 (1602 steps) | 0.0 |
| 5 | grey, brown, black | 1.0 (839 steps) | 0.0 (1404 steps) | 0.0 |
| 6 | brown, black, grey | 1.0 (920 steps) | 0.0 (1390 steps) | 0.0 |
| 7 | grey, black, brown | 1.0 (855 steps) | 0.0 (1698 steps) | 0.0 |
| 8 | black, brown, grey | 1.0 (958 steps) | 0.0 (1696 steps) | 0.0 |
| 9 | brown, grey, black | 1.0 (929 steps) | 0.0 (1482 steps) | 0.0 |

No mismatches against the expected scores (correct 1.0, cyclic 0, none 0). Every correct run swapped the leads in U and
W, ended with all three plugs 23.8 to 26.0 mm in, and the tote 130 mm forward, in 826 to 972 steps. In every `none`
run the tester switched on and the conveyor ran backwards (tote 130 mm back). On 8 of the 10 states the cyclic script
completed its rearrangement: all three plugs home in a cyclic shift of the start, the tester switched on, and the
conveyor ran backwards (tote 130 mm back). On states 2 and 5 the script failed to seat its plugs (on state 2 it ran
out of steps with no plug in a socket; on state 5 the brown plug was left out), so those two zeros are for an
incomplete circuit, not for the phase sequence. That the score is 0 for every even order is also checked directly:
`check_task.py` scores all six orders of the three leads.

Before the robot work, a physics check with no robot (`MOTOR_STATE` 0, with the first socket size tried, 20 x 16.5 mm)
held the start for 8 s: the plugs stayed 24.9 mm in and the tester ran the tote 130 mm backwards (score 0); with the
start order relabelled as if two leads had been swapped, it ran the tote forwards and the score reached 1.0. The
sockets were then widened to 22 x 19 mm because the demo's second plug could not enter (it was carried 1.3 mm high and
drifted 3 to 4.5 mm sideways); the `none` runs above repeat the hold check with the final sockets (plugs 24.1 to 25.1 mm
in, conveyor backwards).

## Scoring

`score()` at the end of the episode, all measurable with a ruler and a photo. Each plug carries two tape marks, 8 mm and
15 mm from its tip. A plug is **in** a socket when it is lined up with it (its centre within 8 mm of the socket's centre
line sideways, pointing into the socket within 25.8 degrees, and not above the opening: its centre at most 5 mm above
the opening's top) and its 8 mm mark is inside the face; it is **home** when its 15 mm mark is inside too.

- **0** unless all of these hold:
  - the robot has touched a plug or lead at some point in the trial;
  - no part of either robot is within 20 mm of the terminal box, the gland or any lead (plug or cable);
  - no plug moved more than 1 mm over the last second;
  - each of the three plugs is in a different socket;
  - the order reverses the motor: two leads exchanged relative to the start (an odd permutation). The start order and
    its two cyclic shifts score 0.
- Then **1.0** if all three plugs are home and the tote has run wholly past the green (forward) line, 100 mm forward of
  where it started;
- otherwise **0.5**.

The supply is off at the start. Once the robot has touched a plug or a lead (a robot point within 2 mm of one), the
tester switches the supply on whenever the robot has been clear (20 mm), the plugs still and all three plugs home for
1 s, and off again if any of that stops holding. A robot that never touches a plug or lead scores 0, and the conveyor
never runs. The motor then runs forward for an odd order and backward for an even
one; with a plug that is not home it does not start (a missing phase). The belt moves the tote at 0.1 m/s, up to 130 mm
either way. The episode ends early on success, once the score has been 1.0 for 1 s.

Partial credit. The start is already a complete, working circuit, so no credit can go to a complete circuit as such, or
"do nothing" would earn it; and a cyclic shift, the tempting wrong answer, is a complete circuit too. The one partial
tier pays for the right decision with incomplete execution: the leads in a reversing order, each in its own socket, but
not all pushed home (the motor does not start). Moving plugs without changing the phase sequence earns nothing.

Only the final order, the plugs' depths, the tote and the robot's distance count. Which two leads are swapped (any of
the three pairs scores 1.0), which arm moves which plug, the order of moves and the path don't.

## Runtime and seeds

The Dream-1 run above took 431 s of wall time for its 1,800 steps on one NVIDIA L40S with the GPU to itself (the GPU
queue's start and end times), including the simulator's start-up. Scripted runs go at about 3 steps/s: physics and
rendering of the three leads and both robots take most of it, the scorer's checks about 13% (profiled over 300 demo
steps). The scripted demo finishes in 826 steps. There are no random seeds: `object_jitter=0`, and the layout is fixed by
`MOTOR_STATE` (0, the task's layout, by default).

## Fidelity

- **The motor is not simulated electrically.** The tester computes its direction from the lead order: an odd
  permutation of the starting order (two leads exchanged) runs it forward, an even one (unchanged, or a cyclic shift)
  backward, and a plug that is not home stops it. That is the physics of a three-phase motor's rotating field, applied
  as a rule. The conveyor's tote is a slider on the bed, driven at 0.1 m/s when the supply is on; the pointer disc is
  spun on its pin. Neither is driven through a belt or gears.
- **Leads** are chains of 12 rigid 2 cm segments with bending springs (0.01 N m/rad per joint, no twist or stretch),
  anchored at the gland. They start in a fixed bow so each has 6 cm of slack, enough for any plug to reach any socket.
- **Plugs and sockets:** plugs are rigid 55 x 16 x 14 mm boxes with no contacts or latch, held in a socket only by its
  walls and friction. Sockets are 22 x 19 mm and 26 mm deep, so a plug has 3 mm of play each side and 5 mm above it;
  real plug-in terminal blocks fit tighter. Seating is judged by depth.
- **Fixed parts:** the terminal box, motor, gearbox, conveyor and gland are one fixed body (screwed down in the real
  build).

## Real-world build

- **Motor and drive:** a small 24 V three-phase brushless motor (or a small induction motor on a low-voltage drive) with
  a sensorless driver, through a right-angle gearbox to the drive drum of a 40 cm desktop belt conveyor (belt top 5 cm
  above the table). Exchanging any two of a three-phase motor's phase leads reverses it, so the real build behaves as
  the simulation's rule does. The motor and gearbox sit in a printed housing (blue-grey and dark grey PLA), with a 5 cm
  white disc with a red radial stripe on the gearbox's front face, on the drum's shaft.
- **Terminal box:** a PLA box 60 x 180 x 45 mm screwed to the table, front face 36 cm in front of the arm bases, with
  three sockets 22 mm wide, 19 mm tall and 26 mm deep at table level, 50 mm apart, marked U, V, W from left to right.
  Each socket has a sprung contact that a plug's tip pin reaches once it is 15 mm in.
- **Leads:** three 6 mm round PVC leads 24 cm long (brown, black, grey) from a 30 x 160 x 30 mm gland plate screwed to
  the table at x 12 to 15 cm, each ending in a 55 x 16 x 14 mm printed plug in the lead's colour (15 g) with a contact
  pin at its tip, and tape marks 8 mm and 15 mm from the tip.
- **Conveyor:** a yellow guard plate along the belt's front edge with two black arrows pointing right, a red line 6 cm
  left and a green line 6 cm right of the tote's starting centre; a blue 6 x 8 x 4 cm tote (150 g) on the belt.
- **Tester:** a switch on the supply, worked by the operator (or an interlock on a light curtain around the box,
  gland and leads): on whenever the robot has been clear of them for 1 s with all three plugs home, including at the
  start, and off as soon as the robot comes near. The belt stops at end stops 130 mm either side of the tote's start.
  Trials are safe: the supply is 24 V and off whenever the robot is near the leads.

Reset: pull the three plugs out, push brown into U, black into V and grey into W until the 15 mm mark is hidden (24 mm
in), lay each lead in its bow to the left as in the setup, bring the tote back to the centre of the belt between the
lines, and turn the disc's stripe to the top. Nothing is consumed.
