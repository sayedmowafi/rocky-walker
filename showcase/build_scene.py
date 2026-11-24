"""Rocky Walker showcase: build the Blender scene from the Fusion meshes and animate it with
frames exported by `python -m rocky.showcase export` (the real motion code).

Run headless (Blender's own Python; no repo imports):

    /Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
        --python showcase/build_scene.py -- \
        --meshes cad/meshes --config config/rocky.toml \
        --anim out/anim/dance.json --out out/render/dance.mp4 [--still out/render/hero.png] \
        [--frames N] [--engine eevee|workbench] [--save out/render/rocky.blend]

Hierarchy (matches rocky.kinematics): torso -> leg{i}_mount (at the mount, yaw psi_i)
-> leg{i}_coxa (about Z by theta1) -> leg{i}_femur (at x = coxa, about Y by -theta2)
-> leg{i}_tibia (at x = femur, about Y by -theta3). The code's femur/knee axes are -y.
"""

import argparse
import json
import math
import sys
import tomllib
from pathlib import Path

import bpy
from mathutils import Matrix, Vector

COLORS = {
    "blue": (0.06, 0.20, 0.62, 1.0),
    "white": (0.85, 0.85, 0.82, 1.0),
    "black": (0.025, 0.025, 0.03, 1.0),
    "grey": (0.22, 0.22, 0.24, 1.0),
    "silver": (0.6, 0.6, 0.62, 1.0),
}
ROUGHNESS = {"blue": 0.45, "white": 0.6, "black": 0.5, "grey": 0.55, "silver": 0.3}


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    p = argparse.ArgumentParser()
    p.add_argument("--meshes", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--anim", type=Path, required=True)
    p.add_argument("--out", type=Path, default=None)
    p.add_argument("--still", type=Path, default=None)
    p.add_argument("--still-frame", type=int, default=None)
    p.add_argument("--frames", type=int, default=None)
    p.add_argument("--engine", choices=["eevee", "workbench"], default="eevee")
    p.add_argument("--save", type=Path, default=None)
    p.add_argument("--cam", default="0.5,-0.5,0.3", help="camera offset from the torso, metres")
    p.add_argument("--aim-z", type=float, default=0.0, help="aim this far above the hip plane")
    p.add_argument("--lens", type=float, default=40.0)
    p.add_argument("--hide", default="", help="comma-separated mesh-name prefixes to leave out")
    return p.parse_args(argv)


def clear_scene():
    for obj in list(bpy.data.objects):
        bpy.data.objects.remove(obj, do_unlink=True)


def material(color: str) -> bpy.types.Material:
    name = f"rocky_{color}"
    mat = bpy.data.materials.get(name)
    if mat:
        return mat
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes.get("Principled BSDF")
    bsdf.inputs["Base Color"].default_value = COLORS[color]
    bsdf.inputs["Roughness"].default_value = ROUGHNESS[color]
    if color == "silver":
        bsdf.inputs["Metallic"].default_value = 0.9
    return mat


def import_mesh(path: Path, origin_mm, color: str) -> bpy.types.Mesh:
    """Import an STL once; return its mesh data in metres, in its segment's joint frame."""
    bpy.ops.wm.stl_import(filepath=str(path))
    obj = bpy.context.selected_objects[0]
    mesh = obj.data
    bpy.data.objects.remove(obj, do_unlink=True)
    mesh.transform(Matrix.Scale(0.001, 4) @ Matrix.Translation(-Vector(origin_mm)))
    mesh.materials.clear()
    mesh.materials.append(material(color))
    for poly in mesh.polygons:
        poly.use_smooth = False
    mesh.name = path.stem
    return mesh


def empty(name, parent=None, location=(0, 0, 0), rot_z=0.0):
    e = bpy.data.objects.new(name, None)
    e.empty_display_size = 0.01
    bpy.context.scene.collection.objects.link(e)
    e.parent = parent
    e.location = location
    e.rotation_mode = "XYZ"
    e.rotation_euler = (0, 0, rot_z)
    return e


def mesh_object(name, mesh, parent):
    o = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(o)
    o.parent = parent
    return o


def build(args):
    cfg = tomllib.loads(args.config.read_text())
    R = cfg["body"]["mount_radius_mm"] / 1000
    LC, LF = cfg["leg"]["coxa_mm"] / 1000, cfg["leg"]["femur_mm"] / 1000
    manifest = json.loads((args.meshes / "manifest.json").read_text())
    meshes = {}
    hidden = tuple(h for h in args.hide.split(",") if h)
    for part in manifest["parts"]:
        if hidden and part["file"].startswith(hidden):
            continue
        meshes.setdefault(part["segment"], []).append(
            import_mesh(args.meshes / part["file"], part["origin_mm"], part["color"])
        )

    torso = empty("torso")
    torso.rotation_mode = "QUATERNION"
    for m in meshes.get("torso", []):
        mesh_object(m.name, m, torso)
    joints = []
    for i in range(5):
        psi = 2 * math.pi * i / 5
        mount = empty(f"leg{i}_mount", torso, (R * math.cos(psi), R * math.sin(psi), 0), psi)
        for m in meshes.get("leg_static", []):
            mesh_object(f"leg{i}_{m.name}", m, mount)
        coxa = empty(f"leg{i}_coxa", mount)
        femur = empty(f"leg{i}_femur", coxa, (LC, 0, 0))
        tibia = empty(f"leg{i}_tibia", femur, (LF, 0, 0))
        for seg, parent in (("coxa", coxa), ("femur", femur), ("tibia", tibia)):
            for m in meshes.get(seg, []):
                mesh_object(f"leg{i}_{m.name}", m, parent)
        joints += [coxa, femur, tibia]
    return torso, joints


def animate(torso, joints, frames, n_frames):
    scene = bpy.context.scene
    prev_q = None
    for k, fr in enumerate(frames[:n_frames]):
        f = k + 1
        m = Matrix([fr["pose"][r * 4 : r * 4 + 4] for r in range(4)])
        loc, rot, _ = m.decompose()
        if prev_q is not None:
            rot.make_compatible(prev_q)
        prev_q = rot
        torso.location = loc
        torso.rotation_quaternion = rot
        torso.keyframe_insert("location", frame=f)
        torso.keyframe_insert("rotation_quaternion", frame=f)
        a = fr["angles"]
        for j, obj in enumerate(joints):
            th = a[j]
            obj.rotation_euler = (0, 0, th) if j % 3 == 0 else (0, -th, 0)
            obj.keyframe_insert("rotation_euler", frame=f)
    for obj in [torso, *joints]:
        if obj.animation_data and obj.animation_data.action:
            for fc in _fcurves(obj.animation_data.action):
                for kp in fc.keyframe_points:
                    kp.interpolation = "LINEAR"
    scene.frame_start, scene.frame_end = 1, min(n_frames, len(frames))


def _fcurves(action):
    if hasattr(action, "fcurves") and len(getattr(action, "fcurves", [])):
        return list(action.fcurves)
    out = []  # Blender 4.4+ layered actions
    for layer in getattr(action, "layers", []):
        for strip in layer.strips:
            for bag in strip.channelbags:
                out += list(bag.fcurves)
    return out


def stage(torso, args):
    scene = bpy.context.scene
    engine = args.engine
    # floor
    bpy.ops.mesh.primitive_plane_add(size=20, location=(0, 0, 0))
    floor = bpy.context.active_object
    floor.name = "floor"
    mat = bpy.data.materials.new("floor")
    mat.use_nodes = True
    nodes, links = mat.node_tree.nodes, mat.node_tree.links
    bsdf = nodes.get("Principled BSDF")
    checker = nodes.new("ShaderNodeTexChecker")
    checker.inputs["Scale"].default_value = 100.0  # 10 cm squares on a 20 m plane
    checker.inputs["Color1"].default_value = (0.55, 0.56, 0.58, 1)
    checker.inputs["Color2"].default_value = (0.47, 0.48, 0.5, 1)
    links.new(checker.outputs["Color"], bsdf.inputs["Base Color"])
    bsdf.inputs["Roughness"].default_value = 0.8
    floor.data.materials.append(mat)

    # lights
    sun = bpy.data.lights.new("sun", "SUN")
    sun.energy = 2.4
    sun.angle = math.radians(8)
    sun_o = bpy.data.objects.new("sun", sun)
    sun_o.rotation_euler = (math.radians(50), 0, math.radians(35))
    scene.collection.objects.link(sun_o)
    fill = bpy.data.lights.new("fill", "AREA")
    fill.energy = 60
    fill.size = 2.0
    fill_o = bpy.data.objects.new("fill", fill)
    fill_o.location = (-1.0, 1.2, 1.2)
    fill_o.rotation_euler = (math.radians(-50), math.radians(-30), 0)
    scene.collection.objects.link(fill_o)
    scene.world = scene.world or bpy.data.worlds.new("world")
    scene.world.use_nodes = True
    bg = scene.world.node_tree.nodes.get("Background")
    bg.inputs["Color"].default_value = (0.18, 0.19, 0.21, 1)
    bg.inputs["Strength"].default_value = 0.6

    # camera follows the torso's position (not its rotation)
    rig = bpy.data.objects.new("camera_rig", None)
    scene.collection.objects.link(rig)
    c = rig.constraints.new("COPY_LOCATION")
    c.target = torso
    cam_data = bpy.data.cameras.new("camera")
    cam_data.lens = args.lens
    cam = bpy.data.objects.new("camera", cam_data)
    scene.collection.objects.link(cam)
    cam.parent = rig
    cam.location = tuple(float(v) for v in args.cam.split(","))
    aim = bpy.data.objects.new("camera_aim", None)
    scene.collection.objects.link(aim)
    aim.parent = rig
    aim.location = (0, 0, args.aim_z)
    track = cam.constraints.new("TRACK_TO")
    track.target = aim
    track.track_axis = "TRACK_NEGATIVE_Z"
    track.up_axis = "UP_Y"
    scene.camera = cam

    r = scene.render
    r.resolution_x, r.resolution_y = 1280, 720
    r.fps = 30
    if engine == "workbench":
        r.engine = "BLENDER_WORKBENCH"
    else:
        for name in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
            try:
                r.engine = name
                break
            except TypeError:
                continue
        eevee = getattr(scene, "eevee", None)
        if eevee is not None and hasattr(eevee, "taa_render_samples"):
            eevee.taa_render_samples = 48
    scene.view_settings.view_transform = (
        "AgX"
        if "AgX"
        in [
            i.identifier for i in scene.view_settings.bl_rna.properties["view_transform"].enum_items
        ]
        else "Filmic"
    )


def render(args):
    scene = bpy.context.scene
    r = scene.render
    if args.still:
        args.still.parent.mkdir(parents=True, exist_ok=True)
        scene.frame_set(args.still_frame or max(1, scene.frame_end // 2))
        r.image_settings.file_format = "PNG"
        r.filepath = str(args.still.resolve())
        bpy.ops.render.render(write_still=True)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        s = r.image_settings
        if hasattr(s, "media_type"):
            s.media_type = "VIDEO"
        s.file_format = "FFMPEG"
        r.ffmpeg.format = "MPEG4"
        r.ffmpeg.codec = "H264"
        r.ffmpeg.constant_rate_factor = "HIGH"
        r.filepath = str(args.out.resolve())
        bpy.ops.render.render(animation=True)


def main():
    args = parse_args()
    clear_scene()
    torso, joints = build(args)
    anim = json.loads(args.anim.read_text())
    frames = anim["frames"]
    n = args.frames or len(frames)
    animate(torso, joints, frames, n)
    stage(torso, args)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.save.resolve()))
    render(args)
    print(f"rendered {min(n, len(frames))} frames")


main()
