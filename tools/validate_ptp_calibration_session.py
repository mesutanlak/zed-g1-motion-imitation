#!/usr/bin/env python3
"""Require clean, continuously locked main-PC and laptop PTP journals."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="Kalibrasyon PTP yan kayitlarini dogrular")
    parser.add_argument("records", type=Path, nargs="+")
    args = parser.parse_args()
    roles: set[str] = set()
    failed = False
    for raw_path in args.records:
        path = raw_path.expanduser().resolve()
        snapshots: list[dict] = []
        footer = None
        try:
            with path.open(encoding="utf-8") as handle:
                for line in handle:
                    value = json.loads(line)
                    if value.get("schema") == "g1_zed_ptp_lock/v1":
                        snapshots.append(value)
                    elif value.get("schema") == "g1_zed_ptp_lock_footer/v1":
                        footer = value
        except (OSError, json.JSONDecodeError) as exc:
            print(f"HATA: PTP kaydi okunamadi: {path}: {exc}", file=sys.stderr)
            failed = True
            continue
        role = str(snapshots[0].get("role")) if snapshots else "missing"
        roles.add(role)
        unlocked = sum(not bool(item.get("locked")) for item in snapshots)
        offsets = [
            abs(int(item["master_offset_ns"]))
            for item in snapshots
            if item.get("master_offset_ns") is not None
        ]
        clean = bool(footer and footer.get("clean_shutdown") is True)
        print(
            f"PTP {role} | ornek={len(snapshots)} unlocked={unlocked} "
            f"max_offset_ns={max(offsets) if offsets else 'N/A'} clean={clean} | {path}"
        )
        if not snapshots or unlocked or not clean:
            failed = True
    if roles != {"main-pc", "laptop"}:
        print(f"HATA: main-pc ve laptop PTP kaydi birlikte gerekli; bulunan={sorted(roles)}", file=sys.stderr)
        failed = True
    return 3 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
