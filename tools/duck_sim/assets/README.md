# Viewer-only articulated jaw

`head_fixed.stl` and `jaw_moving.stl` partition the original reference
`output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/assets/head_roll.stl`
(SHA256 `dcbe8ae093a842399e94eecc6f41a533575df9e8367f26690d737a9d38f8b327`).
The 1,272-triangle connected jaw component is corroborated by the 88 × 67.697 ×
30.100 mm dimensions of `structure/jaw.stl`. The other 17,286 triangles stay fixed.
Every original triangle occurs exactly once in the two meshes, in the original
head-local frame. The closed-mouth surface is unchanged.

Two concentric CAD holes, radii 5 and 8 mm, independently give the hinge at
`[0, 0.0028, -0.0179]` metres (maximum fit residual 7 nm). The head-local axis is
`[-1, 0, 0]`; positive rotation lowers the jaw. `visual_jaw.py` adds this geometry
only to the viewer. ID34 uses its existing calibrated encoder feedback. The
0–30 degree manual slider is a display preview range, not a hardware limit.

The original MuJoCo tree, inertias, meshes, policy dimensions and ONNX file remain
unchanged. `test_jaw.py` verifies triangle preservation and independent jaw motion.
