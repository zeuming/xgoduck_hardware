"""Rebuild CAD engineering references with explicit, right-handed camera views.

Requires numpy, mujoco, onnxruntime and Pillow. No hardware APIs are used.
The front camera is on CAD +X looking toward -X: anatomical left (+Y) appears
on the image right. Depth is h cross v and triangles are painted far to near.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco as mj
import numpy as np
import onnxruntime as ort
from PIL import Image, ImageDraw, ImageFont

from audit_pose import forward_kinematics
from rl_core import POLICY_IDS, POLICY_JOINT_NAMES, policy_profile


ROOT = Path(__file__).resolve().parents[2]
FONT_DEFAULT = Path('/System/Library/Fonts/Hiragino Sans GB.ttc')
VIEWS = {
    'front': (np.array([0., 1., 0.]), np.array([0., 0., 1.]), '正面｜鸭子左在画面右'),
    'side': (np.array([1., 0., 0.]), np.array([0., 0., 1.]), '右侧视｜鸭嘴朝右'),
    'top': (np.array([0., -1., 0.]), np.array([1., 0., 0.]), '俯视｜鸭嘴朝上'),
}
TITLES = ['左髋旋转', '左髋侧摆', '左髋俯仰', '左膝', '左踝',
          '颈俯仰', '头俯仰', '头旋转', '头侧摆',
          '右髋旋转', '右髋侧摆', '右髋俯仰', '右膝', '右踝']
BG, PANEL, INK, MUTED, LINE = '#f8faf7', '#ffffff', '#193b37', '#5d716b', '#d4e0d9'


def pose_meshes(model, q):
    data = mj.MjData(model)
    data.qpos[:] = model.qpos0
    for name, angle in q.items():
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = angle
    mj.mj_forward(model, data)
    rotation, position = forward_kinematics(model, data.qpos)
    if max(np.max(np.abs(rotation-data.xmat.reshape(-1, 3, 3))),
           np.max(np.abs(position-data.xpos))) > 1e-10:
        raise ValueError('Independent FK disagrees with MuJoCo')
    meshes = []
    for gid in range(model.ngeom):
        if model.geom_group[gid] != 2 or model.geom_type[gid] != mj.mjtGeom.mjGEOM_MESH:
            continue
        mesh = model.geom_dataid[gid]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = np.asarray(model.mesh_vert[start:start+count], dtype=float)
        fstart, fcount = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
        faces = np.asarray(model.mesh_face[fstart:fstart+fcount], dtype=int)
        world = np.einsum('nj,ij->ni', vertices, data.geom_xmat[gid].reshape(3, 3)) + data.geom_xpos[gid]
        meshes.append((int(model.geom_bodyid[gid]), world[faces]))
    if not meshes:
        raise ValueError('MJCF has no visual meshes in group 2')
    anchors = {sid: data.xanchor[model.joint(name).id].copy()
               for sid, name in zip(POLICY_IDS, POLICY_JOINT_NAMES)}
    return meshes, anchors, data


def descendants(model, body):
    found = {body}
    for child in range(body+1, model.nbody):
        if model.body_parentid[child] in found:
            found.add(child)
    return found


def projection(mesh_sets, view, box):
    h, v, _ = VIEWS[view]
    points = np.concatenate([triangles.reshape(-1, 3) for meshes in mesh_sets for _, triangles in meshes])
    coords = np.column_stack((np.einsum('ij,j->i', points, h), np.einsum('ij,j->i', points, v)))
    lower, upper = coords.min(0), coords.max(0)
    x0, y0, x1, y1 = box
    scale = min((x1-x0)/(upper[0]-lower[0]), (y1-y0)/(upper[1]-lower[1]))
    middle = (lower+upper)/2
    center = np.array([(x0+x1)/2, (y0+y1)/2])

    def project(points):
        points = np.asarray(points)
        uv = np.stack((np.einsum('...j,j->...', points, h), np.einsum('...j,j->...', points, v)), axis=-1)
        uv = (uv-middle)*[scale, -scale]+center
        return uv
    return project, np.cross(h, v)


def paint(draw, meshes, project, depth, changed=frozenset(), ghost=False, gold_head=False, model=None):
    polygons = []
    light = np.array([.6, -.5, .8]); light /= np.linalg.norm(light)
    for body, triangles in meshes:
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        norm = np.linalg.norm(normals, axis=1)
        normals /= np.maximum(norm[:, None], 1e-12)
        brightness = .62+.38*np.abs(np.einsum('ij,j->i', normals, light))
        if ghost:
            color = np.array([210, 217, 210])
        elif body in changed:
            color = np.array([27, 148, 118])
        elif gold_head and model.body(body).name in ('jaw_soft', 'yaw_roll_motion'):
            color = np.array([219, 177, 48])
        else:
            color = np.array([154, 176, 168])
        xy = project(triangles)
        distances = np.einsum('ij,j->i', triangles.mean(1), depth)
        for coordinates, distance, shade in zip(xy, distances, brightness):
            polygons.append((distance, coordinates, tuple(np.clip(color*shade, 0, 255).astype(int))))
    # Positive depth points toward the camera; larger values must be drawn last.
    for _, coordinates, color in sorted(polygons, key=lambda row: row[0]):
        draw.polygon([tuple(point) for point in coordinates], fill=color)


def font(path, size):
    return ImageFont.truetype(str(path), size)


def text(draw, xy, value, face, fill=INK):
    draw.text(xy, value, font=face, fill=fill)


def leader_labels(draw, anchors, project, pairs, x, box, face):
    entries = [(sid, project(anchors[sid])) for sid in pairs]
    entries.sort(key=lambda item: item[1][1])
    ys = []
    for _, anchor in entries:
        ys.append(max(float(anchor[1]), ys[-1]+29 if ys else box[1]))
    if ys and ys[-1] > box[3]:
        ys = [value-(ys[-1]-box[3]) for value in ys]
    for (sid, anchor), y in zip(entries, ys):
        label = str(sid)
        width = draw.textlength(label, font=face)
        endpoint = (x+width+5 if x < anchor[0] else x-5, y+9)
        draw.line([tuple(anchor), endpoint], fill='#13695b', width=2)
        draw.ellipse((anchor[0]-3, anchor[1]-3, anchor[0]+3, anchor[1]+3), fill='#13695b')
        text(draw, (x, y), label, face, '#13695b')


def render_zero(model, home, output, font_path):
    # Preserve dimensions and the front/side image regions cropped by imu_wizard_ui.
    image = Image.new('RGB', (1400, 840), BG); draw = ImageDraw.Draw(image)
    title, small, normal = font(font_path, 26), font(font_path, 15), font(font_path, 22)
    text(draw, (30, 20), 'XGO-Duck｜CAD 机械零姿态与模型默认姿态', title)
    text(draw, (30, 60), '左、中：14 个关节均为 0 rad；右：当前行走 ONNX 元数据 HOME。嘴部 ID 34 不在模型中。', small, MUTED)
    text(draw, (30, 84), '正面从鸭嘴前方看：鸭子自身左侧在画面右侧。机械零姿态不等于站立姿态。', small, INK)
    zero = dict.fromkeys(POLICY_JOINT_NAMES, 0.)
    zero_mesh, zero_anchor, _ = pose_meshes(model, zero)
    home_mesh, home_anchor, _ = pose_meshes(model, dict(zip(POLICY_JOINT_NAMES, home)))
    for i, (heading, view, meshes, anchors) in enumerate([
        ('零姿态｜正面', 'front', zero_mesh, zero_anchor),
        ('零姿态｜右侧面', 'side', zero_mesh, zero_anchor),
        ('模型 HOME｜右侧面', 'side', home_mesh, home_anchor),
    ]):
        x0 = 25+i*458; panel = (x0, 108, x0+440, 708)
        draw.rounded_rectangle(panel, radius=12, fill=PANEL, outline=LINE, width=1)
        text(draw, (x0+14, 120), heading, normal)
        text(draw, (x0+14, 153), VIEWS[view][2], small, MUTED)
        box = (x0+80, 190, x0+360, 663)
        project, depth = projection([meshes], view, box)
        paint(draw, meshes, project, depth, gold_head=True, model=model)
        if view == 'front':
            # Anatomical right is screen left in a genuine front view.
            leader_labels(draw, anchors, project, [20, 21, 22, 23, 24], x0+17, box, small)
            leader_labels(draw, anchors, project, [10, 11, 12, 13, 14], x0+405, box, small)
            leader_labels(draw, anchors, project, [30, 32], x0+20, (x0, 215, x0, 355), small)
            leader_labels(draw, anchors, project, [31, 33], x0+403, (x0, 215, x0, 355), small)
            text(draw, (x0+30, 680), '鸭子右腿 20–24', small, MUTED)
            text(draw, (x0+245, 680), '鸭子左腿 10–14', small, MUTED)
        else:
            ids = [12, 13, 14, 30, 31]
            leader_labels(draw, anchors, project, ids, x0+405, box, small)
            if i == 2:
                q = dict(zip(POLICY_IDS, np.degrees(home)))
                text(draw, (x0+14, 665), f'11: {q[11]:+.2f}°   21: {q[21]:+.2f}°（髋侧摆）', small, MUTED)
                text(draw, (x0+14, 687), f'12: {q[12]:+.1f}°   14: {q[14]:+.1f}°   30/31: {q[30]:+.1f}°', small, MUTED)
            else:
                text(draw, (x0+14, 680), '侧面标注左腿；右腿对应 20–24。', small, MUTED)
    text(draw, (30, 736), '依据 MJCF 视觉网格和刚体运动学生成；未运行物理仿真。先核对实物装配、关节 ID 和正方向，再保存零点。', small, MUTED)
    text(draw, (30, 769), '所有图片仅用于姿态参照；不能据此扩大舵机范围、确认机械间隙或证明站立与行走能力。', small, MUTED)
    image.save(output)


def render_directions(model, output, font_path):
    image = Image.new('RGB', (1600, 1350), BG); draw = ImageDraw.Draw(image)
    title, normal, small, tiny = [font(font_path, size) for size in (27, 23, 15, 15)]
    text(draw, (28, 18), '正方向核对｜各关节单独从 0° 转到 +10°', title)
    text(draw, (28, 63), '灰色为零位；绿色叠加 +10° 后的部件，不表示遮挡关系。正面从鸭嘴看：鸭子左在画面右。', small, MUTED)
    text(draw, (28, 84), '11 正角：左腿外摆；21 正角：右腿内摆。两者均朝鸭子自身左侧移动。', small, INK)
    zero = dict.fromkeys(POLICY_JOINT_NAMES, 0.)
    baseline, _, _ = pose_meshes(model, zero)
    checks = []
    for index, (sid, name, heading) in enumerate(zip(POLICY_IDS, POLICY_JOINT_NAMES, TITLES)):
        row, column = divmod(index, 5); x0, y0 = 20+column*316, 104+row*387
        draw.rounded_rectangle((x0, y0, x0+302, y0+370), radius=11, fill=PANEL, outline=LINE)
        text(draw, (x0+12, y0+12), f'{sid}｜{heading}', normal)
        view = 'front' if sid in (11, 21, 33) else 'top' if sid in (10, 20, 32) else 'side'
        text(draw, (x0+12, y0+49), VIEWS[view][2], tiny, MUTED)
        changed = dict(zero); changed[name] = np.radians(10)
        meshes, _, data = pose_meshes(model, changed)
        joint = model.joint(name).id
        moving = descendants(model, int(model.jnt_bodyid[joint]))
        project, depth = projection([baseline, meshes], view, (x0+22, y0+77, x0+278, y0+330))
        paint(draw, baseline, project, depth, ghost=True)
        # Overlay only the moving subassembly; near-side stationary legs would
        # otherwise hide the illustrated pitch/knee/ankle motion.
        paint(draw, [(body, triangles) for body, triangles in meshes if body in moving],
              project, depth, changed=moving)
        if sid in (11, 21):
            foot = 'left_foot' if sid == 11 else 'right_foot'
            _, _, before = pose_meshes(model, zero)
            delta = project(data.site(foot).xpos)-project(before.site(foot).xpos)
            if delta[0] <= 0:
                raise ValueError('Front-view positive roll must move toward image right')
            checks.append(dict(id=sid, front_image_foot_delta_px=delta.tolist()))
        text(draw, (x0+10, y0+344), '按绿色方向摆：读数增选 +1，减选 -1', tiny, MUTED)
    text(draw, (28, 1284), '先释放扭矩并支撑机身。参考图表示 CAD 正方向；实物装配和编码器方向需要实际核对。', small, MUTED)
    text(draw, (28, 1317), '不要只按画面左右选方向；先确认该关节 ID。无需点击动作按钮，勿推到机械限位。', small, MUTED)
    image.save(output)
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mjcf', type=Path, default=ROOT/'output/xgoduck_rl_reference/src/mjlab_microduck/robot/xgoduck/robot_walk.xml')
    parser.add_argument('--policy', type=Path, default=ROOT/'output/x5_policy_bundle/models/xgoduck_walk.onnx')
    parser.add_argument('--out-dir', type=Path, default=Path(__file__).resolve().parent/'assets')
    parser.add_argument('--font', type=Path, default=FONT_DEFAULT)
    args = parser.parse_args()
    options = ort.SessionOptions(); options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(args.policy), sess_options=options, providers=['CPUExecutionProvider'])
    home = policy_profile(session)['home_rad']
    model = mj.MjModel.from_xml_path(str(args.mjcf.resolve()))
    # Camera basis is right-handed: h x v points from the robot toward camera.
    for h, v, _ in VIEWS.values():
        if abs(np.dot(h, v)) > 1e-12 or not np.isclose(np.linalg.norm(np.cross(h, v)), 1):
            raise ValueError('Invalid camera basis')
    zero = dict.fromkeys(POLICY_JOINT_NAMES, 0.)
    meshes, _, data = pose_meshes(model, zero)
    project, _ = projection([meshes], 'front', (0, 0, 300, 600))
    if project(data.site('left_foot').xpos)[0] <= project(data.site('right_foot').xpos)[0]:
        raise ValueError('True front view must place anatomical left on image right')
    args.out_dir.mkdir(parents=True, exist_ok=True)
    render_zero(model, home, args.out_dir/'zero-reference.png', args.font)
    checks = render_directions(model, args.out_dir/'positive-direction.png', args.font)
    report = dict(source='https://github.com/LuwuDynamics/xgoduck_rl',
                  model_file=str(args.mjcf.resolve()), model_sha256=hashlib.sha256(args.mjcf.read_bytes()).hexdigest(),
                  policy_file=str(args.policy.resolve()), policy_sha256=hashlib.sha256(args.policy.read_bytes()).hexdigest(),
                  zero_angles='all 14 hinge joints zero',
                  default_angles_deg=dict(zip(POLICY_JOINT_NAMES, np.degrees(home).tolist())),
                  method='MuJoCo visual mesh transforms cross-checked with independent rigid FK; painter-depth orthographic projection',
                  camera_bases={name: dict(horizontal=h.tolist(), vertical=v.tolist(), camera_side=np.cross(h, v).tolist())
                                for name, (h, v, _) in VIEWS.items()},
                  front_view='camera on CAD +X, anatomical left (+Y) on image right',
                  positive_roll_checks=checks,
                  corrected_error='Previous images labelled front used h=-Y, v=+Z, depth=-X with ascending depth paint: an actual rear view was mislabelled as front.',
                  generator=str(Path(__file__).resolve()),
                  not_verified_against_physical_assembly=True, hardware_connections=0, motor_commands=0)
    (args.out_dir/'zero-reference.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
