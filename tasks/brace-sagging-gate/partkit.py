"""partkit: rigid parts built from several boxes, optionally with spring-loaded sliders, for yam_arena task files.

A task file uses it like this (keep this file next to the task file):

    import sys, pathlib
    sys.path.insert(0, str(pathlib.Path(__file__).parent))
    from partkit import Part, Slider, on_step, on_reset

    LID = Part("lid", pos=(0.2, 0.0, 0.76), yaw=0.0, mass=0.04,
               boxes=[((0, 0, 0), (0.114, 0.064, 0.0075), (0.1, 0.3, 0.9))])
    OBJECTS = [LID, ...]

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body with several box and cylinder colliders
(a box with a pocket, a lid with tabs, a cap with lugs). Give it ``sliders`` and it becomes an articulation whose root is
the part and whose sliders are shapes on prismatic joints along one of the part's axes, each pushed to its rest stop by
a spring drive. A box may carry a fourth element, its rotation in the part frame as a (w, x, y, z) quaternion, for
chamfers and ramps; a cylinder may carry a fifth, its axis ("X", "Y" or "Z"); a slider may have its own ``friction``,
which wins its contacts (PhysX's "min" combine mode). ``fixed=True`` welds the part's body to the world (a fixed-base
articulation), for fixtures that carry sliders.

This copy merges the din-rail-mount version (rotated boxes, slider friction) with the dual-latch-lid version (slider
axis and direction, cylinder axis, cylinders on sliders) and adds ``fixed``, sphere shapes, per-part solver and damping
settings, and reads every part's pose in one batch. A fixed part with no sliders is a kinematic rigid body (Isaac Lab
rejects an articulation with no joints); a fixed part with sliders is a fixed-base articulation.

Importing this module hooks ``yam_arena.scene`` (and the embodiment's step and reset) without modifying yam_arena: ``success()/score()`` receive a :class:`PartState` (the body origin's pose, not the bounding-box centre that
yam_arena reports for primitives, plus slider positions). ``on_step`` and ``on_reset`` register callbacks that run
after every control step and after every reset, with the Isaac Lab environment.

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["Part", "PartState", "Slider", "on_reset", "on_step", "env", "write_pose", "install", "part"]

Color = tuple[float, float, float]
# centre, size, colour (part frame, m), and optionally the box's rotation in the part frame as a (w, x, y, z) quaternion
BoxSpec = tuple
CylSpec = tuple  # (centre, radius, height, colour[, axis "X" | "Y" | "Z" = "Z"]) in the part frame
SphereSpec = tuple  # (centre, radius, colour) in the part frame


@dataclass(frozen=True)
class Slider:
    """Shapes on a prismatic joint along the part's ``axis`` ("X", "Y" or "Z"). ``rest`` is where its frame sits in
    the part frame; it can move by ``travel`` in the axis's negative direction (``sign=1``: positive), and a spring of
    ``rate`` N/m pushes it back to ``rest``."""

    name: str
    rest: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    travel: float = 0.004
    rate: float = 2500.0
    damping: float = 20.0
    mass: float = 0.002
    friction: float | None = None  # its own material, which wins its contacts; None uses the part's
    axis: str = "Z"
    sign: int = -1
    overtravel: float = 0.0  # the joint may also move this far past ``rest`` in the spring's direction


@dataclass(frozen=True)
class Pin:
    """Shapes on a revolute joint: a child body whose frame sits at ``rest`` (part frame) at spawn, hinged to
    ``parent`` ("body" or an earlier pin's name) about ``axis`` ("X", "Y" or "Z") through ``anchor`` (part frame).
    ``lower``/``upper`` limit the joint (degrees, 0 at spawn); ``damping`` is the joint's viscous damping (N m s/rad)."""

    name: str
    rest: tuple[float, float, float]
    anchor: tuple[float, float, float]
    parent: str = "body"
    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    axis: str = "X"
    mass: float = 0.01
    damping: float = 0.0
    lower: float = -90.0
    upper: float = 90.0


@dataclass(frozen=True)
class Loop:
    """A revolute joint between two bodies of the part ("body" or pin names) about ``axis`` through ``anchor`` (part
    frame), excluded from the articulation: it closes a kinematic loop (PhysX articulations are trees)."""

    name: str
    body0: str
    body1: str
    anchor: tuple[float, float, float]
    axis: str = "X"


@dataclass(frozen=True)
class Part(Box):
    """A rigid part made of ``boxes``. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is
    its bounding box and ``pos`` is the body origin."""

    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    sliders: tuple[Slider, ...] = ()
    pins: tuple[Pin, ...] = ()
    loops: tuple[Loop, ...] = ()
    friction: float = 0.8
    fixed: bool = False  # welded to the world where it spawns; yam_arena never jitters it
    spheres: tuple[SphereSpec, ...] = ()
    solver_iterations: int = 32          # PhysX position iterations for this body
    linear_damping: float = 0.0          # 1/s
    angular_damping: float = 1.0         # 1/s
    sleep_threshold: float | None = None  # PhysX mass-normalised kinetic energy below which the body may sleep
    restitution: float = 0.0             # the body's material restitution (PhysX default 0)
    com: tuple | None = None             # centre of mass in the part frame (m); None: PhysX computes it from the shapes
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.boxes and not self.cylinders and not self.spheres:
            raise ValueError(f"{self.name}: a Part needs at least one box, cylinder or sphere")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Part's body needs a mass (also when it is fixed)")
        for s in self.sliders:
            if s.axis not in ("X", "Y", "Z") or s.sign not in (-1, 1):
                raise ValueError(f"{self.name}.{s.name}: axis must be X, Y or Z and sign -1 or 1")

    @property
    def free(self) -> bool:
        return not self.fixed


def _quat_matrix(wxyz) -> np.ndarray:
    w, x, y, z = wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _box(spec):
    """(centre, size, colour, rotation wxyz) of a box spec, with no rotation when it has none."""
    centre, size, color = spec[:3]
    return centre, size, color, (tuple(spec[3]) if len(spec) > 3 else (1.0, 0.0, 0.0, 0.0))


def _cyl(spec):
    """(centre, radius, height, colour, axis) of a cylinder spec, with axis "Z" when it has none."""
    centre, radius, height, color = spec[:4]
    return centre, radius, height, color, (spec[4] if len(spec) > 4 else "Z")


def _cyl_half(spec) -> np.ndarray:
    _, r, h, _, axis = _cyl(spec)
    return np.array([h / 2 if axis == "X" else r, h / 2 if axis == "Y" else r, h / 2 if axis == "Z" else r])


def part(name, pos, boxes, mass, yaw=0.0, sliders=(), color=(0.5, 0.5, 0.5), friction=0.8, cylinders=(),
         fixed=False, spheres=(), pins=(), loops=(), **physics) -> Part:
    """Build a Part; ``size`` is filled in from the body's shapes' extent. ``physics``: solver_iterations,
    linear_damping, angular_damping, sleep_threshold, restitution."""
    half = [np.abs(_quat_matrix(q)) @ (np.array(s) / 2) for _, s, _, q in map(_box, boxes)]
    half += [_cyl_half(c) for c in cylinders]
    half += [np.full(3, float(sp[1])) for sp in spheres]
    centres = ([np.array(b[0]) for b in boxes] + [np.array(c[0]) for c in cylinders]
               + [np.array(sp[0]) for sp in spheres])
    lo = np.min([c - h for c, h in zip(centres, half)], axis=0)
    hi = np.max([c + h for c, h in zip(centres, half)], axis=0)
    return Part(name=name, pos=tuple(pos), size=tuple((hi - lo).tolist()), mass=mass, color=color, yaw=yaw,
                boxes=tuple(boxes), cylinders=tuple(cylinders), sliders=tuple(sliders), friction=friction,
                fixed=fixed, spheres=tuple(spheres), pins=tuple(pins), loops=tuple(loops), **physics)


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose (``pos``, ``quat_xyzw``), velocities, bounds, and each slider's position (m, 0 at its
    rest stop; negative when pressed for ``sign=-1``, positive for ``sign=1``)."""

    sliders: dict = field(default_factory=dict)

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.quat_xyzw
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

    def to_local(self, world_point) -> np.ndarray:
        return self.rotation().T @ (np.asarray(world_point, float) - self.pos)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the part under ``prim_path`` (pxr only): a plain Xform holding the body (and sliders) as siblings."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction,
                                        restitution=spec.restitution)
    pm.func(phys, pm)
    materials = {}
    for s in spec.sliders:
        if s.friction is not None:
            materials[s.name] = f"{prim_path}/Looks/physics_{s.name}"
            sm = sim_utils.RigidBodyMaterialCfg(static_friction=s.friction, dynamic_friction=s.friction,
                                                friction_combine_mode="min")
            sm.func(materials[s.name], sm)
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float, main_body: bool = False):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mapi = UsdPhysics.MassAPI.Apply(prim)
        mapi.CreateMassAttr(mass)
        if main_body and spec.com is not None:
            mapi.CreateCenterOfMassAttr(Gf.Vec3f(*spec.com))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(spec.solver_iterations if main_body else 32)
        rb.CreateAngularDampingAttr(spec.angular_damping if main_body else 1.0)  # small stiff parts chatter otherwise
        if main_body and spec.linear_damping:
            rb.CreateLinearDampingAttr(spec.linear_damping)
        if main_body and spec.sleep_threshold is not None:
            rb.CreateSleepThresholdAttr(spec.sleep_threshold)
        if main_body and spec.fixed and not (spec.sliders or spec.pins):
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)   # fixed with no joints: kinematic
        return prim

    def collide(prim, color, material=phys):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0005)  # below the 1 mm fits, so tight parts do not touch on every face at once
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def boxes(path: str, specs, offset=(0.0, 0.0, 0.0), material=phys):
        for i, (centre, size, color, rot) in enumerate(map(_box, specs)):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            if rot != (1.0, 0.0, 0.0, 0.0):
                cube.AddOrientOp().Set(Gf.Quatf(*rot))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color, material)

    def cylinders(path: str, specs, offset=(0.0, 0.0, 0.0), material=phys):
        for i, (centre, radius, height, color, axis) in enumerate(map(_cyl, specs)):
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr(axis)
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*(np.array(centre) - np.array(offset))))
            collide(cyl.GetPrim(), color, material)

    def spheres(path: str, specs, material=phys):
        for i, (centre, radius, color) in enumerate(specs):
            sph = UsdGeom.Sphere.Define(stage, f"{path}/sph_{i}")
            sph.CreateRadiusAttr(radius)
            sph.AddTranslateOp().Set(Gf.Vec3d(*centre))
            collide(sph.GetPrim(), color, material)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.mass, main_body=True)
    boxes(main, spec.boxes)
    cylinders(main, spec.cylinders)
    spheres(main, spec.spheres)
    for s in spec.sliders:
        path = f"{prim_path}/{s.name}"
        body(path, s.rest, s.mass)
        boxes(path, s.boxes, offset=s.rest, material=materials.get(s.name, phys))
        cylinders(path, s.cylinders, offset=s.rest, material=materials.get(s.name, phys))
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{path}/{s.name}_slide")
        j.CreateAxisAttr(s.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(main)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*s.rest))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(-s.travel if s.sign < 0 else -s.overtravel)
        j.CreateUpperLimitAttr(s.overtravel if s.sign < 0 else s.travel)
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(s.rate)
        drive.CreateDampingAttr(s.damping)
        drive.CreateTargetPositionAttr(0.0)
    rests = {"body": (0.0, 0.0, 0.0)}
    for p in spec.pins:
        path = f"{prim_path}/{p.name}"
        rests[p.name] = p.rest
        body(path, p.rest, p.mass)
        boxes(path, p.boxes, offset=p.rest)
        cylinders(path, p.cylinders, offset=p.rest)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{p.name}_pin")
        j.CreateAxisAttr(p.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{p.parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(p.anchor) - np.array(rests[p.parent]))))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(np.array(p.anchor) - np.array(p.rest))))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(p.lower)
        j.CreateUpperLimitAttr(p.upper)
    for lp in spec.loops:
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/{lp.name}_loop")
        j.CreateAxisAttr(lp.axis)
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{lp.body0}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{lp.body1}")])
        j.CreateLocalPos0Attr(Gf.Vec3f(*(np.array(lp.anchor) - np.array(rests[lp.body0]))))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(*(np.array(lp.anchor) - np.array(rests[lp.body1]))))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
    if spec.sliders or spec.pins:
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    if spec.fixed and (spec.sliders or spec.pins):
        # Isaac Lab's own fix_root_link: a world fixed joint on the body, and the articulation root moved to its parent
        # (the PhysX parser only treats it as a fixed-base articulation that way).
        from isaaclab.sim import schemas

        schemas.modify_articulation_root_properties(main, schemas.ArticulationRootPropertiesCfg(fix_root_link=True),
                                                    stage=stage)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            # Implicit actuators carry the springs; their default target is each slider's rest stop.
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_slide"], stiffness=s.rate,
                                                     damping=s.damping) for s in spec.sliders}
            actuators.update({p.name: ImplicitActuatorCfg(joint_names_expr=[f"{p.name}_pin"], stiffness=0.0,
                                                          damping=p.damping) for p in spec.pins})
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    kind = ObjectType.ARTICULATION if (spec.sliders or spec.pins) else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
                                                                       math.cos(spec.yaw / 2))),
    )


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    probe = PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                      low=np.zeros(3), high=np.zeros(3))
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = np.array([probe.to_world(c) for c in corners])
    sliders = {}
    if spec.sliders or spec.pins:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        sliders = {s.name: float(q[names.index(f"{s.name}_slide")]) for s in spec.sliders}
        sliders.update({p.name: float(q[names.index(f"{p.name}_pin")]) for p in spec.pins})
    return PartState(pos=probe.pos, quat_xyzw=probe.quat_xyzw, lin_vel=probe.lin_vel, ang_vel=probe.ang_vel,
                     low=pts.min(0), high=pts.max(0), sliders=sliders)


def _part_states(environment, specs) -> dict:
    """Every part's state, reading all poses and velocities from the GPU in one transfer."""
    import torch

    if not specs:
        return {}
    base = environment.unwrapped
    assets = [base.scene[s.name] for s in specs]
    origin = _np(base.scene.env_origins[0])
    zeros = None
    rows = []
    for a in assets:
        pose = _t(a.data.root_link_pose_w)[0]
        if hasattr(a.data, "root_vel_w"):
            vel = _t(a.data.root_vel_w)[0]
        else:
            zeros = zeros if zeros is not None else torch.zeros(6, dtype=pose.dtype, device=pose.device)
            vel = zeros
        rows.append(torch.cat([pose, vel]))
    data = torch.stack(rows).detach().cpu().numpy().astype(np.float64)
    pos, quat, lin, ang = data[:, :3] - origin, data[:, 3:7], data[:, 7:10], data[:, 10:13]
    x, y, z, w = quat.T
    rot = np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                    np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                    np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    out = {}
    for i, spec in enumerate(specs):
        sliders = {}
        if spec.sliders or spec.pins:
            names = list(assets[i].joint_names)
            q = _np(assets[i].data.joint_pos)[0]
            sliders = {s.name: float(q[names.index(f"{s.name}_slide")]) for s in spec.sliders}
            sliders.update({p.name: float(q[names.index(f"{p.name}_pin")]) for p in spec.pins})
        sliders.update({p.name: float(q[names.index(f"{p.name}_pin")]) for p in spec.pins})
        pts = pos[i] + (signs * np.array(spec.size) / 2) @ rot[i].T
        out[spec.name] = PartState(pos=pos[i], quat_xyzw=quat[i], lin_vel=lin[i], ang_vel=ang[i], low=pts.min(0),
                                   high=pts.max(0), sliders=sliders)
    return out


def write_pose(environment, name: str, pos, quat_xyzw, lin_vel=(0, 0, 0), ang_vel=(0, 0, 0)) -> None:
    """Teleport a part's body (env 0) to a world pose (relative to the env origin) with the given velocity."""
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _t(base.scene.env_origins[0])
    pose = torch.tensor([*pos, *quat_xyzw], dtype=origin.dtype, device=origin.device).unsqueeze(0)
    pose[:, :3] += origin
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([*lin_vel, *ang_vel], dtype=origin.dtype, device=origin.device).unsqueeze(0)
    asset.write_root_velocity_to_sim_index(root_velocity=v)


def body_poses(environment, name: str) -> dict:
    """Every body of an articulated part (env 0): {body name: (pos (m, env frame), quat_xyzw)}."""
    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.body_link_pose_w)[0]
    return {n: (pose[i, :3] - origin, pose[i, 3:7]) for i, n in enumerate(asset.body_names)}


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    """Register ``fn(env)`` to run after every control step, before the task file is scored."""
    _STEP.append(fn)
    return fn


def on_reset(fn):
    """Register ``fn(env)`` to run after every reset, once objects have settled."""
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_partkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        out.update(_part_states(environment, [_SPECS[n] for n in names if n in _SPECS]))
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._partkit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_partkit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _patch_rigid_object()
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._partkit_patched = True


def _patch_rigid_object() -> None:
    """Speed only, no change in behaviour: Isaac Lab's RigidObject.write_data_to_sim runs before every physics substep
    for every rigid object and, even when no external force or torque has been set, zeroes its seven wrench buffers
    (already zero). With hundreds of rigid parts (the beads) that bookkeeping costs more than the physics, so it is
    skipped when neither wrench composer is active; when one is, the original runs unchanged."""
    try:
        from isaaclab_physx.assets.rigid_object.rigid_object import RigidObject
    except ImportError:
        return
    if getattr(RigidObject, "_partkit_fast_write", False):
        return
    orig = RigidObject.write_data_to_sim

    def write_data_to_sim(self):
        if self._instantaneous_wrench_composer.active or self._permanent_wrench_composer.active:
            orig(self)

    RigidObject.write_data_to_sim = write_data_to_sim
    RigidObject._partkit_fast_write = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        orig_exec = loader.exec_module

        def exec_module(module):
            orig_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
