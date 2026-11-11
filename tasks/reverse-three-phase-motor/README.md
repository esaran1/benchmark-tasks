# Reverse a backwards-running three-phase conveyor motor

A short belt conveyor stands across the back of the table, driven by a three-phase motor through a gearbox. The motor's
terminal box faces the robots: three single-pole power sockets, U, V and W from left to right, at table level, hold
the plugs of three supply leads, brown (L1), black (L2) and grey (L3), which come from a cable gland at the front of the table. The motor
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
direction and correcting it is part of every installation. The sockets stand for single-pole power connectors (camlock
or Powerlock style), used to feed machines from temporary power, where exchanging two of the three phase connectors is
the usual fix for a motor that runs the wrong way. The rule the task tests is standard: "The direction of
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
- #777 (buffer-socket-module-permutation: "Move the mis-racked modules between sockets, using the empty fourth socket
  as a buffer, so each module's coloured band matches its socket lamp disc") is the closest manipulation: it permutes
  parts among sockets with one held aside. Its target arrangement is given by a colour match; here no arrangement is
  given, any odd permutation of the start passes, and the tempting move-everything answer fails.
- #176 (free-outlet-keep-freezer: "Move the fan's plug from the outlet to the green tray, then plug the phone charger
  into the socket it freed") moves plugs between places named in the instruction; nothing has to be inferred.
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
  --instruction "This conveyor's motor was just replaced and now runs backwards. Rearrange the supply leads among the terminal box's three sockets so that the conveyor runs the way the arrows on its guard point, with every plug pushed fully home, then let go. The supply is isolated; it is switched on once you are clear." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/motor-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

Dream-1 scored 0.0 in its run of this command on the final code and ran all 1,800 steps. Its left arm went up to the
left end of the conveyor's guard and its right arm reached in around the gearbox and the right side of the terminal box,
but no plug was moved: in the last frame all three plugs are still in their starting sockets (brown U, black V, grey W)
and the tote has not moved, because the supply never came on. (An earlier Dream-1 run, made before the two fixes
described under Fidelity and Scoring, also scored 0.0 without moving a plug; its video is not used.)

## Scripted demo

```bash
python demo.py task.py demo_out                                  # full score: demo_*_cam.mp4, setup.png, result.json
DEMO_VARIANT=cyclic python demo.py task.py out --no-record       # failure variants: see the table below
MOTOR_STATE=3 python demo.py task.py out --no-record             # another starting state (Gap test)
```

1. The left arm pinches the plug in U 10 mm from its lead end, pulls it straight out and holds it up and to the left.
2. The right arm pinches the plug in W, pulls it out, lifts it over the middle lead and carries it to U. It lowers the
   plug in front of U, slides it to 2 mm short of the face, lines its tip up with the socket from the plug's measured
   position and pushes it 5 mm in. If the tip has not entered (the plug slipped or turned in the fingers), it lets go,
   pinches the plug again where it now lies, lines it up and tries again (four tries in all). Then it pushes the plug home to
   24.5 mm, lets go and moves away.
3. The left arm carries its plug over to W and pushes it home the same way, lets go and moves away.
4. The robot is clear, so the tester switches the supply on: the conveyor runs forward and the tote crosses the green line.

On the task's layout it scores 1.0 in 934 steps (the run limit is 1,800): brown ends 24.8 mm into W, black 24.8 mm into
V and grey 26.0 mm into U, the tote 130 mm forward, no robot point within 20 mm of the box, gland or leads.

### Failure variants

Each was run on the task's layout (state 0) with the same task file, `score()` and `success()` as a trial
(`DEMO_VARIANT=... python demo.py task.py out --no-record`), on the final scoring code, with the demo as it was before the re-grip step was added (that step only
acts when a plug fails to enter its socket):

| Variant | What it does | Score | Why |
|---|---|---|---|
| `cyclic` | Moves all three leads one socket along (U to V, V to W, W to U) and pushes them home | 0.0 | All home (brown V 23.7, black W 24.3, grey U 23.3 mm), but the phase sequence is unchanged: the supply came on and the conveyor ran backwards (tote 130 mm back) |
| `half_home` | The correct swap, but the last plug (brown, into W) is pushed in only to 11 mm | 0.5 | The order reverses the motor and each plug is in its own socket, but brown is 10.9 mm in, short of its 15 mm mark: the supply never came on |
| `unplug_one` | The correct swap, but the last plug is laid on the table in front of W | 0.0 | Brown is 2.7 mm short of the face, not in a socket; with a phase missing the motor does not start |
| `hold` | The correct swap, but the left arm keeps hold of the last plug | 0.0 | The order reverses the motor and all three are home (brown W 23.7 mm), but the robot is touching it (0 mm), so the supply stays off |
| `none` | Nothing moved | 0.0 | The robot never touches a plug or lead, so the supply stays off (plugs 23.4 mm in) |

A sixth variant, two swaps in a row (U with W, then V with W, ending in a cyclic shift), was dropped: in its one run
(with the earlier, elastic leads), pulling the black plug out of V also dragged the grey plug out of U, because after
the first swap the grey lead lies across V. It scored 0, but for a plug left out, not for the order. A cyclic shift is
already tested by `cyclic`.

## Gap test

Before the README, the recorded demo or the media were made, the correct strategy and two wrong ones were run with the
robot on 10 starting states, each batch under a score rule written down and committed before it ran
(`gap/gap_rules.txt`). The states change the starting order of the leads (all six orders of brown, black and grey over
U, V and W occur; each is by definition the order that runs backwards), the side the leads bow to, the drive's position
(up to 10 mm in x and y) and its yaw (up to 5 degrees). Each batch ran a frozen copy of the task folder through
`demo.py --no-record`, which steps the same task file and reads the same `score()` and `success()` as a trial.

```bash
bash gap/gworker.sh FROZEN_COPY OUT MOTOR_STATE "0 1 2 3 4 5 6 7 8 9" "correct cyclic none"
python gap/compile.py OUT gap/expected.json
```

There were three batches. The first (elastic leads, supply on from the start) matched on all 30 runs, but two problems
found afterwards changed the physics and the tester (see Fidelity): the leads became plastic, and the supply now stays
off until the robot touches a plug or lead. The second batch reran everything on that final scoring code; it matched
on 29 of 30 runs, all but `correct` on state 8, where the grey plug slipped in the fingers and never entered U (score
0.0, which the rule gives for that outcome). The demo now re-grips a plug whose tip has not entered after a 5 mm push,
and a third batch reran `correct` on all 10 states with that demo. Final results (`correct` from the third batch,
`cyclic` and `none` from the second; score and steps):

| State | Start order (U, V, W) | correct | cyclic | none |
|---|---|---|---|---|
| 0 | brown, black, grey | 1.0 (934) | 0.0 (1286) | 0.0 (160) |
| 1 | black, brown, grey | 1.0 (931) | 0.0 (1368) | 0.0 (160) |
| 2 | grey, black, brown | 1.0 (1085) | 0.0 (1550) | 0.0 (160) |
| 3 | brown, grey, black | 1.0 (749) | 0.0 (1606) | 0.0 (160) |
| 4 | black, grey, brown | 1.0 (890) | 0.0 (1454) | 0.0 (160) |
| 5 | grey, brown, black | 1.0 (782) | 0.0 (1362) | 0.0 (160) |
| 6 | brown, black, grey | 1.0 (904) | 0.0 (1550) | 0.0 (160) |
| 7 | grey, black, brown | 1.0 (1288) | 0.0 (1364) | 0.0 (160) |
| 8 | black, brown, grey | 1.0 (974) | 0.0 (1578, not completed) | 0.0 (160) |
| 9 | brown, grey, black | 1.0 (824) | 0.0 (1604) | 0.0 (160) |

No mismatches against the expected scores (correct 1.0, cyclic 0, none 0). Every correct run swapped the leads in U and
W and ended with all three plugs 19.1 to 26.0 mm in and the tote 130 mm forward, in 749 to 1288 steps. In every `none`
run the robot never touched a plug or lead, the supply stayed off, and the plugs stayed 22.9 to 24.6 mm in. On 9 of the 10 states the cyclic
script completed its rearrangement: all three plugs home in a cyclic shift of the start; the supply came on and the
conveyor ran backwards (tote 130 mm back). On state 8 it left the brown plug outside its socket, so that 0 is for an
incomplete circuit, not for the phase sequence. (In the first batch the cyclic script had failed the same way on states
2 and 5.) That the score is 0 for every even order is also checked directly: `check_task.py` scores all six orders of the
three leads.

Before the robot work, a physics check with no robot (`MOTOR_STATE` 0, with the first socket size tried, 20 x 16.5 mm,
and the first, elastic leads) held the start for 8 s: the plugs stayed 24.9 mm in. With the start order relabelled as if
two leads had been swapped, the tester ran the tote forwards and the score reached 1.0; with the true start order it ran
it backwards and the score stayed 0. The sockets were then widened to 22 x 19 mm because the demo's second plug could not
enter (it was carried 1.3 mm high and drifted 3 to 4.5 mm sideways).

## Scoring

`score()` at the end of the episode, all measurable with a ruler and a photo. Each plug carries two tape marks, 8 mm and
15 mm from its tip. A plug is **in** a socket when it is lined up with it (its centre within 8 mm of the socket's centre
line sideways, pointing into the socket within 25 degrees, and not above the opening: its centre at most 5 mm above
the opening's top) and its 8 mm mark is inside the face; it is **home** when its 15 mm mark is inside too.

- **0** unless all of these hold:
  - the robot has touched a plug or lead at some point in the trial;
  - no part of either robot is within 20 mm of the terminal box, the gland or any lead (plug or cable);
  - no plug moved more than 1 mm over the last second;
  - each of the three plugs is in a different socket;
  - the order reverses the motor: two leads exchanged relative to the start (an odd permutation). The start order and
    its two cyclic shifts score 0.
- Then **1.0** if all three plugs are home and the belt has carried the tote wholly past the green (forward) line,
  100 mm forward of where it started (a tote pushed there by the robot does not count);
- otherwise **0.5**.

The supply is off at the start. Once the robot has touched a plug or a lead (a robot point within 2 mm of one), the
tester switches the supply on whenever the robot has been clear (20 mm), the plugs still and all three plugs home for
1 s, and off again if any of that stops holding. The motor then runs forward for an odd order and backward for an even
one; with a plug that is not home it does not start (a missing phase). A robot that never touches a plug or lead scores
0, and the conveyor never runs. The belt moves the tote at 0.1 m/s, up to 130 mm
either way. The episode ends early on success, once the score has been 1.0 for 1 s.

Partial credit. The start is already a complete, working circuit, so no credit can go to a complete circuit as such, or
"do nothing" would earn it; and a cyclic shift, the tempting wrong answer, is a complete circuit too. The one partial
tier pays for the right decision with incomplete execution: the leads in a reversing order, each in its own socket, but
not all pushed home (the motor does not start). Moving plugs without changing the phase sequence earns nothing.

Only the final order, the plugs' depths, the tote and the robot's distance count. Which two leads are swapped (any of
the three pairs scores 1.0), which arm moves which plug, the order of moves and the path don't.

## Runtime and seeds

The Dream-1 run above took 434 s of wall time for its 1,800 steps on one NVIDIA L40S with the GPU to itself (the GPU
queue's start and end times), including the simulator's start-up. Scripted runs go at about 3 steps/s: physics and
rendering of the three leads and both robots take most of it, the scorer's checks about 13% (profiled over 300 demo
steps). The scripted demo finishes in 934 steps. There are no random seeds: `object_jitter=0`, and the layout is fixed by
`MOTOR_STATE` (0, the task's layout, by default).

## Fidelity

- **The motor is not simulated electrically.** The tester computes its direction from the lead order: an odd
  permutation of the starting order (two leads exchanged) runs it forward, an even one (unchanged, or a cyclic shift)
  backward, and a plug that is not home stops it. That is the physics of a three-phase motor's rotating field, applied
  as a rule. The conveyor's tote is a slider on the bed, driven at 0.1 m/s when the supply is on; the pointer disc is
  spun on its pin. Neither is driven through a belt or gears.
- **Leads** are chains of 12 rigid 2 cm segments with bending springs (0.01 N m/rad per joint, no twist or stretch),
  anchored at the gland. The springs' rest shape is reset to the lead's current shape every control step, so a lead is
  fully plastic: it has no restoring force and keeps whatever shape it is left in. That is a simplification: a real PVC
  lead lying on a table mostly stays where it is put, but has a little springiness. An
  earlier version with springs that rested straight let a bowed lead push its plug into a socket by itself (a plug left
  10.7 mm in crept home in about 20 s); the plastic leads stop that. The leads start in a fixed bow so each has 6 cm of
  slack, enough for any plug to reach any socket.
- **Settling of a loose plug:** a plug left lying outside a socket can still slide slowly while its lead settles over
  the other leads. In a 40 s test (the `unplug_one` variant held longer), a plug the demo put down 10 mm in front of W
  (commanded) was 2.8 mm in front of the face once the arm had cleared, then crept towards it at about 0.12 mm/s (to
  0.4 mm inside the face 30 s later). A plug inside a socket did not move (7.06 mm deep for 30 s), and the seated plugs
  stayed seated (24.8 to 26.0 mm). Calculated, not run: at 0.12 mm/s a plug at the face would take about 125 s to creep
  15 mm in, longer than the 90 s run limit; if it ever did, the tester would count it as home.
- **Plugs and sockets:** plugs are rigid 55 x 16 x 14 mm boxes with no contacts or latch, held in a socket only by its
  walls and friction. Sockets are 22 x 19 mm and 26 mm deep, so a plug has 3 mm of play each side and 5 mm above it;
  real plug-in terminal blocks fit tighter. Seating is judged by depth.
- **Fixed parts:** the terminal box, motor, gearbox, conveyor and gland are one fixed body (screwed down in the real
  build).

## Real-world build

- **Motor and drive:** a small 24 V three-phase brushless motor (or a small induction motor on a low-voltage drive) with
  a sensorless driver, through a right-angle gearbox to the drive drum of a 40 cm desktop belt conveyor (aluminium bed,
  black rubber belt, belt top 5 cm above the table). Exchanging any two of a three-phase motor's phase leads reverses it, so the real build behaves as
  the simulation's rule does. The motor and gearbox sit in a printed housing (blue-grey and dark grey PLA), with a 5 cm
  white PLA disc with a red radial stripe on the gearbox's front face, on the drum's steel shaft.
- **Terminal box:** a blue-grey PLA box 60 x 180 x 45 mm screwed to the table, front face 36 cm in front of the arm bases, with
  three single-pole sockets 22 mm wide, 19 mm tall and 26 mm deep at table level, 50 mm apart, marked U, V, W from
  left to right (printed stand-ins for camlock-style power connectors).
  Each socket has a sprung contact that a plug's tip pin reaches once it is 15 mm in.
- **Leads:** three 6 mm round PVC leads 24 cm long (brown, black, grey) from a 30 x 160 x 30 mm dark grey PLA gland plate screwed to
  the table at x 12 to 15 cm, each ending in a 55 x 16 x 14 mm printed plug in the lead's colour (15 g) with a contact
  pin at its tip, and tape marks 8 mm and 15 mm from the tip.
- **Conveyor:** a yellow PLA guard plate along the belt's front edge with two black painted arrows pointing right, a red
  tape line 6 cm left and a green tape line 6 cm right of the tote's starting centre; a blue PLA tote 6 x 8 x 4 cm
  (150 g) on the belt.
- **Tester:** a switch on the supply, worked by the operator (or an interlock on a light curtain around the box,
  gland and leads): off at the start; once the robot has touched a plug or lead, on whenever the robot has been clear
  of them for 1 s with all three plugs home, and off as soon as the robot comes near. The belt stops at end stops 130 mm either side of the tote's start.
  Trials are safe: the supply is 24 V and off whenever the robot is near the leads.

Reset: pull the three plugs out, push brown into U, black into V and grey into W until the 15 mm mark is hidden (24 mm
in), lay each lead in its bow to the left as in the setup, bring the tote back to the centre of the belt between the
lines, and turn the disc's stripe to the top. Nothing is consumed.
