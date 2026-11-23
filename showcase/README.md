# Showcase: Rocky in Blender

Renders the Fusion design walking and dancing, animated by the real motion code (the same
`MotionController` and `rocky.moves` player that will drive the servos).

```bash
# 1. export animation frames from the motion code
uv run python -m rocky.showcase export all out/anim/

# 2. render one clip headless (Blender 4.2+; tested on 5.2)
/Applications/Blender.app/Contents/MacOS/Blender --background --factory-startup \
  --python showcase/build_scene.py -- \
  --meshes cad/meshes --config config/rocky.toml \
  --anim out/anim/dance.json --out out/render/dance.mp4
```

Options: `--still out/render/hero.png --still-frame 360` renders one frame, `--frames N` renders
only the first N frames, `--engine workbench` is a fast preview, and `--save out/render/rocky.blend`
keeps the scene so you can open it in Blender.

Clips: `walk-forward`, `walk-diagonal`, `turn` (8 s after 1 s standing) and `dance` (the
21 s routine in `moves/routine.toml`). Feet that touch the floor stay planted: the export checks
that no planted foot slides more than 1 mm between frames.
