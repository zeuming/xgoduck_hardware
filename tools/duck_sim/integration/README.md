# X5 8767 simulation integration

Entry: `http://192.168.1.11:8767/sim/` (also available through the USB address when connected). The native workbench sidebar includes a link opening this entry in a separate tab.

The original developer-machine viewer is unchanged. `prepare_bundle.py` builds a separate X5 copy with relative assets, the workbench CSRF token, and a local STATE-only reader instead of SSH back to itself.

- X5 app: `/opt/xgoduck-sim/app`; isolated dependencies: `/opt/xgoduck-sim/deps`.
- Backend: `xgoduck-sim.service`, loopback `127.0.0.1:8788`, Nice 10, CPU quota 100%, memory limit 1 GiB. Enabled at boot. No developer-machine tunnel required.
- Public gateway: `/sim/` in the existing HTTP frontend. Fixed upstream, session authentication, Host/Origin/CSRF validation, bounded requests and six proxy slots. No controller RPC is used for simulation.
- Simulation, replay and default joint preview do not connect UART; optional slider following uses the native controller. Live feedback is explicitly started and retains the existing reader's STATE-only packet whitelist and exclusive ownership checks. Do not start it while another workbench owns UART.
- Native calibration and motion policy are preserved. Installation updates archive hashes and seal; X5 backups are kept under `/opt/xgoduck-workbench-before-sim-*`.

Build with `python3 tools/duck_sim/integration/prepare_bundle.py`. The generated archive is `output/duck_sim/8767-integration/sim.tar.gz`. `install_backend.py` consumes this archive on stdin on X5 after dependencies are available. `install_gateway.py` is a one-time reviewed patch for a workbench without the sim gateway; it refuses active workbench operations and changed package integrity. Re-read current installation before any later deployment. Do not replace a live package with an old local snapshot.

Tests: `python3 -m unittest discover -s tools/duck_sim/integration -p test_sim_web.py -v`.

## Optional joint slider control (installed 2026-10-12; physical acceptance pending)

The new panel calls the existing native `/api/motion` directly. The simulation
proxy and STATE-only reader remain unchanged. Install `policy/joint_follow.py`
and the additive native runtime/workbench edits together with the viewer files.
`tools/x5_policy/prepare_joint_follow.py --source <retrieved-tree> --output <new-dir>`
prepares these native edits and records before/after hashes without deploying.
It requires exact source anchors and refuses reuse of an existing output path.
Retrieve and verify the current installed archive and installation seal before
applying that output; do not deploy the historical local source tree wholesale.
The general native bundle builder includes the new controller module.

Offline checks: `test_joint_follow.py`, `test_joint_follow_web.py` in x5_policy,
and `node tools/duck_sim/test_joint_follow.mjs`. These do not prove physical
following. The first live acceptance must start from supported, stationary,
in-range feedback and exercise one small joint displacement plus explicit STOP.

Deployment uses `install_joint_follow.py --stage <review-stage> --install` after
its default validation pass. It pins the retrieved installation, boot identity,
calibration and viewer hashes; requires proven idle/no-UART state; backs up the
native installation and four viewer files; and rolls back startup failures.
The 2026-10-12 install changed only three native members and four viewer assets.
Both services are active; calibration revision 25 and bytes are unchanged.
HTTP resource and inactive-request checks passed with zero UART opens/commands.
The in-app browser timed out, so online interaction and physical following
remain unverified. Local deployment receipts are under
`output/duck_sim/joint-follow-20261012/deployment/`.
