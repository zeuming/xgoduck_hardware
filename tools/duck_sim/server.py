#!/usr/bin/env python3
"""Local XGO Duck viewer with simulation and a read-only SSH feedback relay."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import mimetypes
from pathlib import Path
import sys
import threading
import signal
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from types import SimpleNamespace
from urllib.parse import urlsplit
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
REFERENCE = ROOT / "output/xgoduck_rl_reference"
ROBOT = REFERENCE / "src/mjlab_microduck/robot/xgoduck"
MODEL = ROOT / "output/x5_policy_bundle/models/xgoduck_walk.onnx"
RECORD = ROOT / "output/x5_gait_trial/20261009/forward-immediate-collapse-v8/closed-state-and-traces.json"
sys.path.insert(0, str(ROOT / "tools/x5_policy"))

import numpy as np
from audit_policy_sim import load_dependencies, make_sim, refresh_observation_data
from rl_core import (POLICY_IDS, POLICY_INDEX, POLICY_JOINT_NAMES, ACTION_ALPHA,
                     ACTION_ALPHA_HEAD, State, action_to_deg, build_obs, policy_profile)
from live import LiveFeedback
from visual_jaw import with_visual_jaw

LABELS = ["左髋偏航", "左髋侧摆", "左髋俯仰", "左膝", "左踝", "颈俯仰", "头俯仰",
          "头偏航", "头侧摆", "右髋偏航", "右髋侧摆", "右髋俯仰", "右膝", "右踝"]


def vec(element, name, default):
    return [float(n) for n in element.get(name, default).split()]


def model_tree():
    root = ET.parse(ROBOT / "robot_walk.xml").getroot()
    def body(el):
        joint = el.find("joint")
        j = None
        if joint is not None:
            index = POLICY_JOINT_NAMES.index(joint.get("name"))
            j = dict(name=joint.get("name"), index=index, id=POLICY_IDS[index], label=LABELS[index],
                     axis=vec(joint, "axis", "0 0 1"), pos=vec(joint, "pos", "0 0 0"),
                     range=np.rad2deg(vec(joint, "range", "-3.14 3.14")).tolist())
        return dict(name=el.get("name"), pos=vec(el, "pos", "0 0 0"),
                    quat=vec(el, "quat", "1 0 0 0"), joint=j,
                    meshes=[dict(name=g.get("mesh"), pos=vec(g, "pos", "0 0 0"),
                                 quat=vec(g, "quat", "1 0 0 0"))
                            for g in el.findall("geom") if g.get("class") == "visual"],
                    children=[body(b) for b in el.findall("body")])
    assets = {m.get("name", Path(m.get("file")).stem): dict(file=m.get("file"), scale=vec(m, "scale", "1 1 1"))
              for m in root.findall("asset/mesh")}
    return body(root.find("worldbody/body")), assets


def tilt(gravity):
    return math.degrees(math.acos(float(np.clip(-np.asarray(gravity)[2] / np.linalg.norm(gravity), -1, 1))))


def gravity_quaternion(gravity):
    """Minimal rotation taking body-frame gravity to world down; yaw unobserved."""
    g = np.asarray(gravity, dtype=float)
    g /= np.linalg.norm(g)
    down = np.array([0., 0., -1.])
    if np.dot(g, down) < -.999999:
        return [0., 1., 0., 0.]
    q = np.r_[1 + np.dot(g, down), np.cross(g, down)]
    return (q / np.linalg.norm(q)).tolist()


def load_replays():
    if not RECORD.exists():
        return []
    entries = json.loads(RECORD.read_text())["entries"]
    result = []
    for entry in entries:
        value = entry.get("value")
        if not isinstance(value, dict) or not value.get("frames"):
            continue
        cfg = value["calibration"]["joints"]
        ids = value["motor_ids"]
        order = [ids.index(sid) for sid in POLICY_IDS]
        jaw_index = ids.index(34) if 34 in ids else None
        start = value["frames"][0]["monotonic"]
        rows = list(value["frames"])
        if value.get("terminal"):
            rows.append(value["terminal"])
        frames, markers = [], []
        previous_mode, was_forward = None, False
        for row in rows:
            actual, gravity = row.get("actual_angle_deg"), row.get("gravity")
            if not actual or not gravity:
                continue
            t = row["monotonic"] - start
            target_ticks = row.get("attempted_target_ticks")
            target = ([(tick - j["zero"]) * 360 / 4095 * j["sign"]
                       for tick, j in zip(target_ticks, cfg)] if target_ticks else None)
            command = row.get("command") or [0, 0, 0]
            mode = row.get("mode", "unknown")
            if mode != previous_mode:
                markers.append(dict(t=t, label={"hold": "固定站姿", "walk": "零速策略"}.get(mode, mode)))
                previous_mode = mode
            if command[0] > 0 and not was_forward:
                markers.append(dict(t=t, label="前进指令"))
                was_forward = True
            jaw = round(actual[jaw_index], 5) if jaw_index is not None and jaw_index < len(actual) else None
            jaw_target = round(target[jaw_index], 5) if target and jaw_index is not None and jaw_index < len(target) else None
            frames.append(dict(t=t, q=[round(actual[i], 5) for i in order] + [jaw],
                               target=[round(target[i], 5) for i in order] + [jaw_target] if target else None,
                               base=[0, 0, .139439], quat=gravity_quaternion(gravity),
                               tilt=tilt(gravity), command=command, mode=mode,
                               event=row.get("event"), error=row.get("request_error")))
        if not frames:
            continue
        frames.sort(key=lambda f: f["t"])
        reason = value.get("reason", "")
        markers.append(dict(t=frames[-1]["t"], label="记录结束"))
        name = entry["name"]
        result.append(dict(id=name, title=("前进中止 · 历史 8° 阈值" if "connection-2-trial-2" in name
                           else "站姿检查 · " + name.removesuffix(".json")),
                           kind="replay", frames=frames, markers=markers, reason=reason,
                           source=str(RECORD.relative_to(ROOT)), duration=frames[-1]["t"],
                           truncated=value.get("truncated", False),
                           note="实测关节角与 IMU 倾斜；机身位置固定，偏航未知。虚影为当帧尝试发送的目标，并非舵机已执行。"))
    return result


class Simulator:
    def __init__(self):
        self.mj, ort, self.load_model, self.controller_type = load_dependencies(REFERENCE / "vendor/bam")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(MODEL), sess_options=options, providers=["CPUExecutionProvider"])
        self.profile = policy_profile(self.session)
        self.lock = threading.Lock()
        self.args = SimpleNamespace(xml=ROBOT / "robot_walk.xml", bam_params=ROBOT / "params/1910_m6.json",
                                    kp=5., voltage=7.7, physics_dt_ms=5., mass_kg=None)

    def run(self, speed=0.12, duration=6.):
        if (not math.isfinite(speed) or not -.2 <= speed <= .3 or
                not math.isfinite(duration) or not 1 <= duration <= 15):
            raise ValueError("速度须在 -0.20～0.30 m/s，时长须在 1～15 秒")
        if not self.lock.acquire(blocking=False):
            raise ValueError("正在计算上一段物理仿真，请稍后再试")
        try:
            return self._run(speed, duration)
        finally:
            self.lock.release()

    def _run(self, speed, duration):
        mj, home = self.mj, self.profile["home_rad"]
        model, data, bam, controller, qi, vi, trunk, _, mass = make_sim(
            self.args, mj, self.load_model, self.controller_type)
        observed = mj.MjData(model)
        data.qpos[:7] = [0, 0, .14, 1, 0, 0, 0]
        data.qpos[qi] = home
        mj.mj_forward(model, data)
        # Place the soles 1 mm above the floor using actual compiled mesh transforms.
        foot_geoms = [model.geom(side + "_foot_collision").id for side in ("left", "right")]
        zmin = []
        for gid in foot_geoms:
            mesh = model.geom_dataid[gid]
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            points = model.mesh_vert[start:start + count] @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
            zmin.append(points[:, 2].min())
        data.qpos[2] += .001 - min(zmin)
        mj.mj_forward(model, data)
        controller.reset(data.qpos)
        bam.actuator.q_target_smooth = data.qpos[qi].copy()
        controller.last_ts = 0.
        previous = np.zeros(14, np.float32)
        smooth = previous.copy()
        frames, reason = [], "完成指定仿真时长"
        initial_positions = None
        for step in range(round(duration / .005) + 1):
            refresh_observation_data(mj, model, data, observed)
            rotation = observed.xmat[trunk].reshape(3, 3)
            gravity = rotation.T @ np.array([0., 0., -1.])
            degrees = np.rad2deg(observed.qpos[qi])
            if not np.isfinite(data.qpos).all():
                raise ValueError("物理求解出现非有限状态，已中止此段仿真")
            if step % 4 == 0:
                angles, vels = np.zeros(15), np.zeros(15)
                angles[POLICY_INDEX] = degrees
                vels[POLICY_INDEX] = np.rad2deg(observed.qvel[vi])
                velocity = np.zeros(6)
                mj.mj_objectVelocity(model, observed, mj.mjtObj.mjOBJ_BODY, trunk, velocity, 1)
                state = State([0, 0, 0], [0, 0, 0], angles.tolist(), vels.tolist(), gravity.tolist(), velocity[:3].tolist())
                command = [speed * min(1., data.time / .5), 0, 0]
                obs = build_obs(state, previous, np.array(command, np.float32), np.zeros(4, np.float32), home_rad=home)
                action = self.session.run(None, {self.session.get_inputs()[0].name: obs.reshape(1, 61)})[0][0]
                if action.shape != (14,) or not np.isfinite(action).all():
                    raise ValueError("模型输出无效")
                alpha = np.full(14, ACTION_ALPHA)
                alpha[5:9] = ACTION_ALPHA_HEAD
                smooth = alpha * smooth + (1 - alpha) * action
                target = np.array(action_to_deg(smooth, home_rad=home, action_scale=self.profile["scale"]))[POLICY_INDEX]
                controller.q_target = np.deg2rad(target)
                previous = action.copy()
                contacts = {side: False for side in ("left", "right")}
                for contact in observed.contact:
                    for side, gid in zip(contacts, foot_geoms):
                        if gid in (contact.geom1, contact.geom2) and model.geom("policy_audit_floor").id in (contact.geom1, contact.geom2):
                            contacts[side] = True
                frame = dict(t=float(data.time), q=degrees.tolist(), target=target.tolist(),
                             base=data.qpos[:3].tolist(), quat=data.qpos[3:7].tolist(),
                             tilt=tilt(gravity), command=command, mode="walk", contacts=contacts)
                frames.append(frame)
                if initial_positions is None:
                    initial_positions = {model.body(i).name: observed.xpos[i].tolist() for i in range(1, model.nbody)}
                if frame["tilt"] > 65 or frame["base"][2] < .045:
                    reason = "仿真模型已失稳，时间轴停在失稳姿态；可复位调整指令"
                    break
            controller.update()
            mj.mj_step(model, data)
        return dict(kind="simulation", frames=frames, duration=frames[-1]["t"], reason=reason,
                    markers=[dict(t=0., label="起始站姿"), dict(t=.5, label="速度渐入完成")],
                    speed=speed, mass=mass, kp=5, voltage=7.7,
                    initial_body_positions=initial_positions,
                    note="MuJoCo 接触物理 + ONNX 策略 + BAM 舵机模型；使用 CAD 质量与参考摩擦参数，尚未标定为这台实机。")


class App:
    def __init__(self):
        self.sim = Simulator()
        tree, assets = model_tree()
        tree, assets = with_visual_jaw(tree, assets)
        self.meta = dict(tree=tree, assets=assets, home=np.rad2deg(self.sim.profile["home_rad"]).tolist() + [0.],
                         joint_names=list(POLICY_JOINT_NAMES) + ['jaw'], labels=LABELS + ['嘴部开合'], motor_ids=POLICY_IDS + [34],
                         policy_motor_ids=POLICY_IDS,
                         policy_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
                         model="xgoduck_walk.onnx", physics="MuJoCo " + self.sim.mj.__version__,
                         mesh_source=str(ROBOT.relative_to(ROOT)), jaw="ID34 实测随动 / 手动预览；行走物理模型仍为 14 关节")
        self.replays = load_replays()
        self.live = LiveFeedback()


def handler(app):
    class Handler(BaseHTTPRequestHandler):
        def send(self, status, data, mime="application/json; charset=utf-8"):
            payload = json.dumps(data, ensure_ascii=False, allow_nan=False).encode() if not isinstance(data, bytes) else data
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(payload)))
            self.send_header("Cache-Control", "no-cache")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/api/model":
                return self.send(200, app.meta)
            if path == "/api/replays":
                return self.send(200, app.replays)
            if path == "/api/live":
                value = app.live.snapshot(touch=True)
                feedback = value.get('feedback')
                if feedback and feedback.get('gravity'):
                    feedback['quat'] = gravity_quaternion(feedback['gravity'])
                    feedback['tilt'] = tilt(feedback['gravity'])
                return self.send(200, value)
            if path.startswith('/assets/viewer/'):
                file = (HERE / 'assets' / path.removeprefix('/assets/viewer/')).resolve()
                allowed = (HERE / 'assets').resolve()
            elif path.startswith("/assets/"):
                file = (ROBOT / "assets" / path.removeprefix("/assets/")).resolve()
                allowed = (ROBOT / "assets").resolve()
            else:
                file = (HERE / ("index.html" if path == "/" else path.lstrip("/"))).resolve()
                allowed = HERE
            if not file.is_relative_to(allowed) or not file.is_file() or file.suffix not in (".html", ".js", ".css", ".stl", ".txt"):
                return self.send(404, {"error": "未找到资源"})
            self.send(200, file.read_bytes(), mimetypes.guess_type(str(file))[0] or "application/octet-stream")

        def do_POST(self):
            if self.path not in ("/api/simulate", "/api/live/connect", "/api/live/disconnect"):
                return self.send(404, {"error": "未找到接口"})
            # Requests are same-origin and JSON-only; this server never operates hardware.
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + self.headers.get("Host", ""):
                return self.send(403, {"error": "仅允许同源请求"})
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 1024 or self.headers.get_content_type() != "application/json":
                    raise ValueError("需要 JSON 仿真参数")
                params = json.loads(self.rfile.read(length))
                if not isinstance(params, dict):
                    raise ValueError("参数应为对象")
                if self.path == '/api/live/connect':
                    return self.send(200, app.live.start())
                if self.path == '/api/live/disconnect':
                    return self.send(200, app.live.stop())
                result = app.sim.run(float(params.get("speed", .12)), float(params.get("duration", 6)))
                self.send(200, result)
            except (ValueError, TypeError) as exc:
                self.send(400, {"error": str(exc)})
            except Exception as exc:
                print("Simulation error:", repr(exc), file=sys.stderr)
                self.send(500, {"error": "仿真计算失败，请检查本地服务日志"})

        def log_message(self, fmt, *args):
            if str(args[1]) != "200":
                super().log_message(fmt, *args)
    return Handler


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8788)
    args = parser.parse_args()
    app = App()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler(app))
    print(f"Duck Sim Lab: http://127.0.0.1:{args.port}", flush=True)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(SystemExit()))
    try:
        server.serve_forever()
    finally:
        app.live.stop()
        server.server_close()
