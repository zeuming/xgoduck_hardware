#!/usr/bin/env python3
"""Package a compiled motion image without opening serial ports or flashing."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core-dir", default="/tmp/xgoduck-pio-core")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    project = Path(__file__).resolve().parent
    output = args.output or project.parents[1] / "output/esp32_motion/firmware-ram-gains"
    output.mkdir(parents=True, exist_ok=True)
    build = project / ".pio/build/esp32s3_motion"
    boot_app = Path(args.core_dir) / "packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin"
    segments = []
    for offset, source in [(0, build / "bootloader.bin"), (0x8000, build / "partitions.bin"),
                           (0xe000, boot_app), (0x10000, build / "firmware.bin")]:
        data = source.read_bytes()
        destination = output / source.name
        if destination.exists() and destination.read_bytes() != data:
            raise RuntimeError(f"Refusing to replace different packaged artifact: {destination}; use a fresh --output")
        shutil.copyfile(source, destination)
        segments.append({"offset": offset, "file": source.name, "size": len(data),
                         "sha256": hashlib.sha256(data).hexdigest()})
    sources = {}
    for name in ["platformio.ini", "motion_partitions.csv", "firmware/main.cpp", "firmware/motion_protocol.h", "firmware/maintenance_impl.h", "firmware/off_register_details.h"]:
        sources[name] = hashlib.sha256((project / name).read_bytes()).hexdigest()
    manifest = {"firmware": "XGODUCK_MOTION_1", "protocol": 1, "chip": "esp32s3",
                "flash_size": "16MB", "flash_mode": "dio", "flash_freq": "80m",
                "host_baud": 921600, "starts_disarmed": True, "hardware_tested": False,
                "policy_position": {"capability_request": 24, "capability_ack": [24, 0],
                                    "target_payload_bytes": 35, "profile_byte": 1,
                                    "extra_policy_slew_ticks_s": None,
                                    "legacy_target_payload_bytes": 34,
                                    "legacy_pose_slew_ticks_s": 600,
                                    "reset_on_stop_and_arm": True,
                                    "range_and_watchdog_checks_unchanged": True},
                "imu_recovery": {"missing_sample_ms": 1000, "retry_interval_ms": 1000,
                                 "reset_poll_budget_ms": 20,
                                 "requires_all_torque_off_readbacks": True,
                                 "servo_and_torque_readback_max_age_ms": 50,
                                 "disabled_during_maintenance_or_arm": True,
                                 "preserves_fault_latch": True,
                                 "automatic_rearm": False},
                "arm": {"refresh_actual_feedback_after_preparation": True,
                        "refresh_full_positions_during_post_enable_torque_check": True,
                        "revalidate_current_positions_before_enable": True,
                        "torque_write_reply_window_us": 1500,
                        "torque_write_reply_window_enable_only": True,
                        "disable_preserves_tx_only_and_requires_readback": True,
                        "torque_failure_first_id_and_outcome": True,
                        "failure_ack_bytes": 22,"failure_detail_version": 1},
                "background_torque": {"scan_start_interval_us": 250000,
                                      "motors_per_loop": 3,
                                      "readback_validity_us": 500000,
                                      "position_freshness_us": 50000,
                                      "target_watchdog_us": 250000,
                                      "automatic_rearm": False},
                "first_safety_stop": {"target_failure_ack_bytes": 40,
                                      "failure_detail_version": 1,
                                      "cookie_bound": True,
                                      "preserved_across_rejected_target_and_stop": True,
                                      "reset_on_explicit_valid_arm": True,
                                      "extra_bus_transactions": 0,
                                      "feedback_and_watchdog_limits_unchanged": True},
                "goal_read_details": {"request_type": 22, "reply_type": 150,
                                      "version": 1, "reply_bytes": 78,
                                      "mapped_id_request_bytes": 1,
                                      "register_reads": [[42, 2], [56, 6]],
                                      "off_only": True, "requires_zero_cookie": True,
                                      "requires_known_torque_off": True,
                                      "torque_readback_max_age_us": 500000,
                                      "position_sample_max_age_us": 50000,
                                      "requires_completed_position_cycle": True,
                                      "requires_healthy_imu": False,
                                      "minimum_attempt_interval_us": 500000,
                                      "attempt_retries": False,
                                      "read_poll_deadline_us": 1500,
                                      "hard_total_latency_bound_claimed": False,
                                      "servo_writes": False,
                                      "refreshes_feedback_or_watchdog": False,
                                      "preserves_existing_safety_checks": True,
                                      "mcu_goals_are_retained_off_values": True},
                "off_register_details": {"request_type": 23, "reply_type": 151,
                                         "version": 1, "reply_bytes": 202,
                                         "mapped_id_request_bytes": 1,
                                         "header_bytes": 42, "read_block_bytes": 20,
                                         "register_reads": [[0, 6], [16, 2], [33, 1], [40, 1],
                                                            [42, 6], [48, 4], [62, 2], [69, 2]],
                                         "raw_register_semantics_verified": False,
                                         "raw_validity": "normal_reply_and_exact_size_only",
                                         "off_only": True, "requires_zero_cookie": True,
                                         "requires_known_torque_off": True,
                                         "torque_readback_max_age_us": 500000,
                                         "position_sample_max_age_us": 50000,
                                         "requires_completed_position_cycle": True,
                                         "requires_healthy_imu": False,
                                         "shared_rate_limit_with_request_type": 22,
                                         "minimum_attempt_interval_us": 500000,
                                         "attempt_retries": False,
                                         "read_poll_deadline_us": 1500,
                                         "hard_total_latency_bound_claimed": False,
                                         "servo_writes": False,
                                         "refreshes_feedback_or_watchdog": False,
                                         "preserves_existing_safety_checks": True,
                                         "mcu_goals_are_retained_off_values": True},
                "live_voltage": {"capability": "XGODUCK_TELEMETRY_1", "caps_request_type": 20,
                                 "state_request_type": 19, "state_reply_type": 147,
                                 "state_bytes": 186, "legacy_state_bytes": 106,
                                 "register": 62, "register_size": 1, "volts_per_count": 0.1,
                                 "reads_per_loop": 1, "validity_us": 1000000,
                                 "per_servo_age_us": True, "retains_dated_last_value": True,
                                 "read_only": True, "disabled_during_maintenance": True,
                                 "hd1910_scale_hardware_verified": False,
                                 "manufacturer_operating_range_v": [4.0, 8.4],
                                 "battery_remaining_capacity_estimated": False},
                "maintenance": {"capability": "XGODUCK_MAINT_1", "requires_explicit_enter": True,
                                "exclusive_with_motion": True, "typed_operations_only": True,
                                "watchdog_ms": 1000, "move_timeout_ms": 10000,
                                "successful_center_releases_torque": True,
                                "center_failure_ack_bytes": 101,
                                "center_check_request_type": 16,
                                "holding_goal_may_enable_torque": True,
                                "center_check_releases_torque": True,
                                "center_accepts_verified_holding_torque": [0, 1],
                                "voltage": {"capability": "XGODUCK_VOLTAGE_1",
                                            "caps_request_type": 17, "read_request_type": 18,
                                            "register": 62, "size": 1, "volts_per_count": 0.1,
                                            "hd1910_scale_hardware_verified": False,
                                            "reply_bytes": 8, "read_only": True,
                                            "requires_maintenance_cookie": True,
                                            "disabled_during_center": True}},
                "ram_run_gains": {"kp_register": 50, "kd_register": 51, "kp": 6, "kd": 20,
                                  "mouth_id": 34, "mouth_kp": 10, "applied_only_on_arm": True,
                                  "readback_required_before_enable": True,
                                  "bounded_write_reply_before_read_us": 1500,
                                  "write_retries": False},
                "segments": segments, "sources": sources}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"output": str(output), "segments": segments, "hardware_tested": False}, indent=2))


if __name__ == "__main__":
    main()
