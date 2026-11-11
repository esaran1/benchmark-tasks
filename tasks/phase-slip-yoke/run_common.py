"""Helpers for stage1.py: build the yam_arena embodiment on this folder's task file, read the parts' states, and hold
both arms at READY with the grippers open."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

HZ = 20
HERE = Path(__file__).resolve().parent


def make(task_path="task.py"):
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    sys.path.insert(0, str(HERE))
    emb = yam_arena_embodiment(task=str(HERE / task_path), object_jitter=0, control_hz=HZ, cam_width=320, cam_height=176)
    emb.reset(Scene(id="s", instruction="s"))
    return emb, emb.task.module


class Robot:
    def __init__(self):
        from yam_arena.constants import READY
        self.cmd = np.r_[np.array(READY, float), 1.0]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.cmd, self.cmd))


def states(emb):
    from yam_arena.scene import object_states
    return object_states(emb._env, emb.task.names)
