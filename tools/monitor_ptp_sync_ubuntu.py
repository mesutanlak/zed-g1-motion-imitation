#!/usr/bin/env python3
"""Record and optionally gate linuxptp lock for a calibration session."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any


STATE_RE = re.compile(r"(?:to|portState\s+)\s*(MASTER|SLAVE|FAULTY|LISTENING|UNCALIBRATED)", re.I)
OFFSET_RE = re.compile(r"master offset\s+(-?\d+)\s+(s[012])\b.*?path delay\s+(-?\d+)", re.I)
PMC_STATE_RE = re.compile(r"^\s*portState\s+(MASTER|SLAVE|FAULTY|LISTENING|UNCALIBRATED)\s*$", re.I | re.M)
PMC_OFFSET_RE = re.compile(r"^\s*master_offset\s+(-?\d+)\s*$", re.M)
PMC_GM_RE = re.compile(r"^\s*gmPresent\s+(true|false)\s*$", re.I | re.M)


def command(*args: str) -> tuple[int, str]:
    try:
        result = subprocess.run(
            args, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            check=False, timeout=3.0,
        )
    except subprocess.TimeoutExpired:
        return 124, "command timed out"
    return result.returncode, result.stdout


def pmc_status(role: str) -> tuple[int, str]:
    # ptp4l's read-only management socket accepts GET requests from an
    # unprivileged client. Give pmc a writable client socket outside /var/run.
    with tempfile.TemporaryDirectory(prefix="g1-ptp-monitor-") as directory:
        return command(
            "pmc", "-u", "-b", "0", "-f", f"/etc/linuxptp/g1-zed-{role}.cfg",
            "-i", f"{directory}/client", "-s", "/var/run/ptp4lro",
            "GET PORT_DATA_SET", "GET TIME_STATUS_NP",
        )


def snapshot(role: str, interface: str, maximum_offset_ns: int) -> dict[str, Any]:
    service_code, service_text = command("systemctl", "is-active", "g1-zed-ptp.service")
    link_code, link_text = command("ip", "-o", "link", "show", "dev", interface)
    pmc_code, pmc_text = pmc_status(role)
    _journal_code, journal = command(
        "journalctl", "-u", "g1-zed-ptp.service", "-n", "240", "--no-pager", "-o", "cat"
    )
    states = STATE_RE.findall(journal)
    offsets = OFFSET_RE.findall(journal)
    pmc_states = PMC_STATE_RE.findall(pmc_text)
    pmc_offsets = PMC_OFFSET_RE.findall(pmc_text)
    pmc_gm = PMC_GM_RE.findall(pmc_text)
    # The journal is diagnostic only: summary_interval=0 commonly prints RMS
    # lines without the "master offset ... s2" text used by older versions.
    state = pmc_states[-1].upper() if pmc_states else None
    offset_ns = int(pmc_offsets[-1]) if pmc_offsets else None
    gm_present = pmc_gm[-1].lower() == "true" if pmc_gm else None
    servo_state = offsets[-1][1].lower() if offsets else None
    path_delay_ns = int(offsets[-1][2]) if offsets else None
    service_active = service_code == 0 and service_text.strip() == "active"
    link_up = link_code == 0 and "state UP" in link_text
    expected_state = "MASTER" if role == "main-pc" else "SLAVE"
    locked = bool(
        service_active
        and link_up
        and pmc_code == 0
        and state == expected_state
        and (
            role == "main-pc"
            or (
                gm_present is True
                and offset_ns is not None
                and abs(offset_ns) <= maximum_offset_ns
            )
        )
    )
    return {
        "schema": "g1_zed_ptp_lock/v1",
        "timestamp_unix_ns": time.time_ns(),
        "role": role,
        "interface": interface,
        "service_active": service_active,
        "link_up": link_up,
        "port_state": state,
        "journal_port_state": states[-1].upper() if states else None,
        "expected_port_state": expected_state,
        "servo_state": servo_state,
        "gm_present": gm_present,
        "measurement_source": "linuxptp_pmc_readonly",
        "pmc_available": pmc_code == 0,
        "master_offset_ns": offset_ns,
        "path_delay_ns": path_delay_ns,
        "maximum_offset_ns": maximum_offset_ns,
        "locked": locked,
    }


def main() -> int:
    # Bash may launch this monitor as an asynchronous job with SIGINT ignored.
    # Restore Python's handler so the camera launcher's Ctrl+C can write a
    # clean journal footer before calibration validates the PTP evidence.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    parser = argparse.ArgumentParser(description="PTP kilidini JSONL olarak izler")
    parser.add_argument("--role", choices=("main-pc", "laptop"), required=True)
    parser.add_argument("--interface", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--maximum-offset-ns", type=int, default=1_000_000)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--require-locked", action="store_true")
    parser.add_argument("--fail-on-unlock-samples", type=int, default=0)
    args = parser.parse_args()
    if args.interval_s <= 0.0 or args.maximum_offset_ns < 0:
        parser.error("interval ve offset siniri gecersiz")
    output = None
    if args.output is not None:
        path = args.output.expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        output = path.open("a", encoding="utf-8", buffering=1)
    ever_unlocked = False
    consecutive_unlocked = 0
    count = 0
    try:
        while True:
            value = snapshot(args.role, args.interface, args.maximum_offset_ns)
            count += 1
            ever_unlocked = ever_unlocked or not bool(value["locked"])
            consecutive_unlocked = 0 if value["locked"] else consecutive_unlocked + 1
            line = json.dumps(value, ensure_ascii=False, allow_nan=False)
            if output is not None:
                output.write(line + "\n")
            if args.once or output is None:
                print(line)
            if args.once:
                return 0 if (value["locked"] or not args.require_locked) else 3
            if (
                args.fail_on_unlock_samples > 0
                and consecutive_unlocked >= args.fail_on_unlock_samples
            ):
                print("PTP kilidi kayboldu; kalibrasyon kaydi durdurulmali.", file=sys.stderr)
                return 3
            time.sleep(args.interval_s)
    except KeyboardInterrupt:
        return 3 if args.require_locked and ever_unlocked else 0
    finally:
        if output is not None:
            output.write(json.dumps({
                "schema": "g1_zed_ptp_lock_footer/v1",
                "timestamp_unix_ns": time.time_ns(),
                "samples": count,
                "ever_unlocked": ever_unlocked,
                "clean_shutdown": True,
            }, ensure_ascii=False) + "\n")
            output.close()


if __name__ == "__main__":
    raise SystemExit(main())
