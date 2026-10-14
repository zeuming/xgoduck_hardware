# X5 local wireless gamepad control

Installed on `root@192.168.1.11` on 2026-10-10. Open
<http://192.168.1.11:8767/#motion-models> and locate **无线手柄 · X5 本地控制**.
The latest gamepad adapter listens at boot without webpage setup. It stays torque-OFF until a fresh explicit action button. Earlier deployment records below are historical.

## Verified input and deployment

- Receiver USB `0079:181c`, Linux `/dev/input/js0`, 8 axes / 15 buttons.
- `xgoduck-gamepad.service` only reads input and publishes `/run/xgoduck-gamepad/input.json`; it cannot open UART or command motors.
- Physical button presses/releases were read and matched with the operator: R1 = 7, A = 0, B = 1, SELECT = 10. All were released at the end. Evidence: `output/duck_sim/native-gamepad/button-evidence.json`.
- The controller process consumes input locally and listens at boot. Webpage and gamepad model buttons share the current runtime without handover. The webpage does not relay joystick commands or provide its ongoing heartbeat.
- The installed simulation gateway and every existing policy/model/calibration byte were retained. No firmware, servo gains, tilt boundary or walking limits changed.
- Earlier R1 combinations and release-to-cancel mappings are superseded by the single-button mapping below.

## Operation (shared webpage and gamepad buttons)

2026-10-12: the webpage and gamepad operate the same current model. Neither side requires ownership handover. The model sessions load at service startup. The input worker starts at boot with a neutral baseline; an explicit A/X invokes the existing guarded connection if needed. No button is replayed at startup or reconnection.

- A (0): request default HOLD, matching the webpage's default stance.
- X (3): start WALK at zero, or return a running WALK to zero.
- Y (4): from WALK toggle continuous 0.30 m/s forward / zero, including a WALK started by the webpage.
- B (1): return WALK to zero, including a WALK started by the webpage.
- START (11): request default HOLD; both webpage and gamepad remain usable.
- SELECT (10): explicit STOP / torque release, highest priority; support the robot.
- L1 (6): load/connect through the webpage's existing `connect` action, without starting HOLD/WALK. A/X still connect automatically if needed.
- R1 (7): unload/disconnect through the webpage's existing `disconnect` action, including its STOP/release checks. It preserves model files; no combination key is required.
- L2 / input button **8**: run `getup` once, up to 8 seconds including its default-pose stage.
- R2 / input button **9**: run `pick` once for 4 seconds, including the existing mouth action.
- Right-stick press / input button **14**: run `roulade` once for about 1.9 seconds.

The three new physical labels must be checked against the page's live input numbers. A fresh dedicated model key confirms the current action area and fall protection, matching the existing webpage confirmation. Each single action ends with STOP/release; held keys never repeat or start walking afterwards. SELECT interrupts every model. Input loss during a single action requests STOP, while the existing WALK-to-HOLD recovery remains unchanged. New model keys do not chain while another single action is running.

The motion page groups all four official policies with individual start and no-motion preview buttons, their loaded/running state, and corresponding gamepad keys. Normal page/gamepad model starts share the existing authenticated entry point; generation, firmware, calibration, feedback and saved mechanical range checks remain in the controller. No model weights, motion timing, servo gains or calibration are changed. These controls require physical acceptance separately; local fake-device checks do not establish standing, recovery, grasping or rolling performance.

Installed all-model controls on 2026-10-12 with seal `f093df42a559e4db70cbc7096b3bc09759131d8ba985e0baebafe875706b032f`. Exactly five service files changed; every other bundle member and calibration revision 50 were retained. Post-install checks found all four models loaded, all four start/preview cards served, the new key map published, both services active, and no UART owner or automatic motion. Evidence: `output/duck_sim/all-models-20261012/validation.json`. Passed 79 targeted adapter/staged tests, 20 canonical IPC tests, four-model CPU inference and fake-controller completion checks, and page/panel checks. The deployed-package IPC suite retains three failures also reproduced on the unchanged baseline (older fixture lacks `release_torque` and excludes runtime-owned heartbeats); this is not a full gate pass. No physical action test or Git commit was performed.

The webpage can issue default HOLD, WALK and velocity commands after gamepad operation. Existing current-generation, calibration, feedback, firmware, model and mechanical range checks remain. Joint-slider maintenance retains its separate ownership requirement. Stale browser heartbeats are not granted control. Input loss requests the existing HOLD transition and never restarts WALK automatically. The receiver does not expose verified handset radio liveness.

Earlier exclusive-takeover instructions and deployment records below are historical and are superseded by this operation section.

## Implementation and verification

`tools/x5_policy/native_gamepad.py` runs separate local input and heartbeat workers in the existing sole-controller process. Model buttons share normal HOLD/WALK operation; joint-slider maintenance keeps its ownership guard. Heartbeats stop if the input worker stalls. Stale/gapped events are not replayed. Input failure requests HOLD through existing guards; failure to restore HOLD is reported and does not bypass the watchdog. Sensor timestamps are never renewed by input polling.

Passed 94 Python tests covering local ownership, input-file replacement, monitoring, separate-process HTTP/IPC, existing native APIs, packaging and input reading; also passed 8 JS panel scenarios. These use fake hardware or read-only input. **Standing/walking under gamepad control remains untested.**

`install_native_control.py` merges four reviewed files into the pinned current installation, verifies saved calibration revision 24, refuses active UART ownership, retains secrets/backups on X5, and checks no UART was opened after restart. Installation report and input evidence are under `output/duck_sim/native-gamepad/`. The older `install_on_x5.py` is for its specifically pinned historical deployment, not the current X5 seal.

Read-only diagnostics:

```sh
ssh root@192.168.1.11 'systemctl status xgoduck-gamepad --no-pager'
ssh root@192.168.1.11 'cat /run/xgoduck-gamepad/input.json'
```

`initial: true` events in `monitor.py` are synthetic initialization, not real button presses. Never infer radio presence from the receiver's USB identity or input-file publication time.

## Input failure diagnosis and correction (2026-10-10)

The previous input reader sampled its clock before opening an atomically replaced snapshot. If the process yielded before the open, the newly published file could appear to come from the future and unnecessarily revoke gamepad ownership. It now timestamps after reading and decoding. Genuine future timestamps and the existing 500 ms stale limit still reject input. Failure details retain the reason, snapshot age and input session/sequence so that a later recovery cannot hide the original reason. The original reported incident had only a generic error, so its precise cause cannot be retrospectively proven.

A deterministic regression reproduces the old race. On the real X5 input file, a comparison with an **injected 20 ms pre-open delay** rejected 39/100 reads in the old version and 0/100 in the fixed version. These are injected scheduling-test counts, not a measured natural failure rate. Evidence: `output/duck_sim/gamepad-input-race/`.

## Control-source monitoring and browser isolation

Expand **查看控制来源与心跳记录** in the gamepad panel. It shows accepted/rejected requests per source, local/browser heartbeat counts and intervals, input failure details, and the existing controller timing/last-stop fields. An accepted request is not proof of physical execution. GET status reads do not renew the control lease. Old-page commands and automatic page-hidden stop requests cannot take over the gamepad; an explicit emergency stop remains available.

A bounded diagnostic is published at most once per second under the current RAM runtime's `native-evidence/control-source.json`. It survives closing a browser, but not a service restart or reboot; save evidence before restarting if an incident needs investigation. A failure to write this optional monitor does not add a control stop path.

Same-X5, fake-motor tests passed, with robot device access denied:

- No browser requests for 30.01 s: 291 local heartbeats, zero browser heartbeats; maximum interval 117.78 ms.
- Eight concurrent readers for 10.05 s: 1,476 GETs, 101 local heartbeats, zero browser heartbeats; maximum interval 103.69 ms.
- Suspending the test HTTP child for 550 ms did not interrupt local heartbeats (maximum interval 103.63 ms).
- Monitor-file failure preserved control behavior.

Evidence: `output/duck_sim/control-isolation/`. These test input/HTTP isolation and scheduling, **not real model inference, servo response, physical gait, or handset radio loss**. Initial connection, fixed stance and explicit takeover still use the webpage. The HTTP server child exiting, an IPC failure, or restarting the workbench service still closes the controller under the existing supervisor behavior. Closing a browser tab is distinct from stopping that service.

The monitor/race fixes are installed with installation seal `17ad7ed4d771b263c54233d9699adc5f0afe4858467fb6990bda30387776f128`. Final read-only verification found the controller disconnected, gamepad inactive, no process owning UART, calibration revision 24 retained, and the simulation gateway retained. No motion or firmware flashing was performed during that gamepad installation. In a subsequent explicitly authorized battery-off update on 2026-10-10, the bounded position-read retry candidate was flashed and read back. Two earlier powered checks had no valid joint-position feedback in 40 samples each. After a subsequently observed X5 reboot, a new read-only OFF check passed 40/40 samples with all 15 joints online, fresh IMU data, no fault bits and zero servo UART errors recorded in that MCU boot. The recovery cause is not established; armed retry, stance and walking acceptance remain pending. See `tools/esp32_motion/position_retry_candidate/README.md` and `output/duck_sim/position-read-retry-install/installation-status.json` for current firmware status; no standing or walking was attempted.

## Single-button update installed (2026-10-12)

The current deployed controller uses the official continuous runtime. The new adapter follows that runtime and page: A default stance; X zero-speed model/return to zero; Y toggle continuous 0.30 m/s forward / zero; B return to zero; START default stance and hand back; SELECT STOP/release. R1, sticks and D-pad no longer trigger movement. The page-bottom table states these operations. Earlier timed-trial and R1 instructions above describe historical installations and are superseded.

Installed seal: `24374d46ab183341f63039bdd687a0bb4111dc9f51c04fe04d8dbcd0ae58144c`. Only `service/native_gamepad.py` and `service/gamepad_panel.html` changed from the latest reviewed package. Calibration revision **38** and its SHA-256, models, policy implementation, firmware and simulation gateway were preserved. Live page checks confirmed the table and all six button numbers; the controller remained disconnected, the gamepad inactive and the fresh shutdown/startup proof reported no UART opened. No standing or walking was started. Evidence: `tmp/gamepad-single-buttons-20261012/verification.json`.

Passed 27 local single-button/ownership tests, 19 HTTP/IPC tests and 8 panel scenarios; the existing log suite also passed 16 tests after updating its input fixture. A later run of one existing no-reply timing test briefly observed its busy rejection instead of the expected expiry; the isolated rerun and final full 19-test IPC run passed.

Current network URL: <http://192.168.66.44:8767/#motion-models>. The update closes the previous OFF/released feedback connection and leaves hardware disconnected; reconnect explicitly through the webpage before handing over to the gamepad. Physical X/Y/START labels and movement behavior still require operator acceptance. No Git commit was created.

## Standalone boot operation (2026-10-12)

The adapter now listens independently at service startup, requires a neutral input baseline and accepts a fresh A/X without any browser setup. The explicit key triggers the existing guarded connection and start. A brief release during connection remains the same press; another key, SELECT, input-session change or missing events cancels the pending start. STOP/reconnection require neutral plus a new press. Standalone START returns to HOLD and keeps the local heartbeat. Browser-owned active control is preserved; SELECT still has priority.

Startup trust faults use the workbench's existing scoped connection/feedback or unopened-maintenance review, with no local clearing of fault flags. Failed reviews never start motion. Current X5 verification found the receiver online and `standalone_ready=true`, `browser_setup_required=false`, with no UART opened at service restart. The current boot's CH341/USB history still fails the installed transport checker (`Device not responding to setup address`, descriptor/address errors); the read-only connection acceptance did **not** pass and no stance/walk was attempted. This change does not claim the USB fault repaired.

Verified 39 handover/standalone tests, 45 calibration tests, 19 HTTP/IPC tests and 8 panel scenarios. Deployment keeps calibration revision 42, all model/policy/firmware bytes and the simulation gateway. Evidence: `tmp/gamepad-boot-20261012/`. No Git commit was created.

Final standalone installation seal: `c9d342d3ef42437218cca5880f7d20a326657e96ed3032e7fd8a6a0ea8ff4257`. Only the gamepad adapter and bottom-panel instructions differ from this turn's baseline package.

## Shared-button correction (2026-10-12)

The operator's A/X input was confirmed in the live receiver stream at sequence 24, with all four models already loaded. The old adapter refused these presses because a browser owner remained, including after an OFF fault with a newer hardware generation. This was a software ownership mismatch, not missing webpage model loading.

The adapter now follows the current connection independently of which source last started it. Normal HOLD/WALK starts and WALK commands share the authenticated workbench entry point; each request still validates the latest hardware and page generation. Maintenance/slider ownership is unchanged. The bottom panel removes takeover controls and keeps webpage controls available after a gamepad start. Existing installed runtime-owned heartbeats and every model, firmware and calibration byte are preserved.

Passed 44 adapter tests (including five shared-button cases), 20 HTTP/IPC tests (including browser commands after gamepad start), five shared-button tests compiled from the exact staged workbench method, and eight panel scenarios. The broader 50-test workbench suite reports nine failures and two errors in both the unchanged baseline and modified run; its older page-stop/UI assumptions predate this correction. Evidence is under `tmp/gamepad-shared-20261012/`. No Git commit was created.

Installed after the operator explicitly paused other work and supported the robot. Installation seal: `2578d7a5882e162b778a14217d7baaf535a74aa1c4a9a00db4d8bd2693838514`. Exactly `service/native_gamepad.py`, `service/native_workbench.py`, and `service/gamepad_panel.html` changed from seal `ef4b9d21581550590a27450470bc3b8c6bbc4386c0fb6f52b6511191abcc694f`; every other bundle member was compared byte-for-byte and retained, including the latest runtime-owned model heartbeat. Saved calibration revision **50** and SHA-256 `100393d63ba44fb6b0834ba26695ddef060d03e7c059f1c2e363c9dc56e8b7f3` were preserved.

Live verification confirmed `shared_model_buttons=true`, `standalone_ready=true`, `browser_setup_required=false`, all four models loaded (`walk`, `getup`, `pick`, `roulade`), and the exact new panel served in the page. The receiver was online with all buttons released. Startup remained disconnected with `no_uart_opened=true`; no process owned UART. This verifies installation and readiness, not real standing/walking or repair of previously observed servo feedback/range faults. Current page: <http://192.168.1.11:8767/#motion-models>. Refresh only to see the updated page instructions; no webpage action is required before a fresh A/X press. Evidence: `tmp/gamepad-shared-20261012/installation-result.json` and `verification.json`. No Git commit was created.

## Real runtime context correction and load/unload keys (2026-10-12)

The operator's subsequent A presses reached X5 and the first press successfully connected, but no START request followed. The adapter incorrectly expected `armed=false` in `NativeRuntime.control_context()`. That method deliberately publishes a small ownership projection and omits `armed`; its `release_confirmed` field is the corresponding OFF/release check. The adapter now uses that actual field. Real start validation still runs through the unchanged workbench and controller gates. Y toggling reads the current published command rather than expecting it in the ownership projection, so it also cancels forward velocity most recently set by the webpage.

L1 (6) and R1 (7) now alias the existing webpage connect/load and stop/disconnect operations. They never imply a model START, held keys never replay, multiple operation keys do not chain, and SELECT remains first. Models are CPU-preloaded by the current runtime; the page's unload/disconnect releases the hardware connection and retains model files/sessions. The bottom table names both operations and their single buttons.

Passed 51 adapter tests, 20 HTTP/IPC tests, and eight panel scenarios. Shared-button tests now execute the real `NativeRuntime.control_context()` method on a fake controller, covering the previously missed contract. The earlier fixture returned a full hardware snapshot and therefore hid this defect. Deployment evidence is under `tmp/gamepad-context-20261012/`; physical stance/walking acceptance remains separate. No Git commit was created.

Installed seal `77ef303991f9c2a9d8a799dba7ead7101d04097940281fd0955e042a86966097`. Only adapter/panel changed; all other bundle members were compared and retained. Calibration revision 50 was preserved. The read-only input service was paused during the final idle disconnect/update to prevent new A presses reconnecting the old version; it was restarted successfully. Post-start status confirms standalone/shared readiness, four loaded models, and no automatic UART opening. The live page serves the exact new eight-button mapping. Real button retest is requested separately; installation success is not physical movement proof.

The operator subsequently confirmed a physical response. Live requests show a successful A→HOLD start and local heartbeats. Earlier repeated A presses during the 2.56 s connection check correctly canceled that pending start; a later fresh A started HOLD. This is attended button/HOLD-response evidence, not stable standing or walking acceptance.

The retest also exposed a heartbeat racing an X→WALK generation change: a rejected older heartbeat wrongly revoked the session and requested HOLD. The adapter now discards only a superseded heartbeat when the same valid connection has a newer generation, or the heartbeat's input session has already retired. It does not retry the old request, report it as accepted, or send a replacement START. Failures with unchanged generation remain errors. Passed 54 adapter tests, including deterministic generation-race, retired-session and genuine-failure checks. The remote RAM retest records disappeared at a subsequent device reboot before download; a summary of the live diagnostic output and operator confirmation is saved at `tmp/gamepad-context-20261012/attended-retest-summary.json`. Subsequent update evidence: `tmp/gamepad-heartbeat-20261012/`. No Git commit was created.

The operator explicitly stopped/supported the robot for the final correction. X5 rebooted before installation, so the old boot pin was rejected; identity, unchanged seal and calibration were rechecked against boot `cc1cc810-4aa7-47ea-9d1b-2f831ddd0c39`. Final installed seal: `aadffcaa137e217769ede6dac705eb73423c6beb1982f34351583661341ce569`. Only `service/native_gamepad.py` changed from seal `77ef303991f9c2a9d8a799dba7ead7101d04097940281fd0955e042a86966097`; every other member is byte-identical. Calibration 50, live L1/R1 table, four loaded models, restored input reader and standalone/shared readiness were verified. Startup opened no UART. Physical mode-switch retest after this last correction remains pending. Installation/verification reports are saved under `tmp/gamepad-heartbeat-20261012/`.
