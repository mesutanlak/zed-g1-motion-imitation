"""The calibration lock gate must accept current PMC data, not stale log text."""

from unittest.mock import patch

from tools import monitor_ptp_sync_ubuntu as monitor


def _snapshot(pmc_text: str):
    def fake_command(*args):
        if args[:2] == ("systemctl", "is-active"):
            return 0, "active\n"
        if args[:3] == ("ip", "-o", "link"):
            return 0, "2: enp130s0: <BROADCAST,UP> state UP\n"
        if args[0] == "journalctl":
            return 0, "port 1: LISTENING to SLAVE\nrms 123 max 250 freq 0 delay 100\n"
        raise AssertionError(args)

    with patch.object(monitor, "command", side_effect=fake_command), patch.object(
        monitor, "pmc_status", return_value=(0, pmc_text)
    ):
        return monitor.snapshot("laptop", "enp130s0", 1_000_000)


def test_summary_only_journal_uses_readonly_pmc_offset():
    result = _snapshot("portState SLAVE\nmaster_offset 250\ngmPresent true\n")
    assert result["locked"] is True
    assert result["master_offset_ns"] == 250
    assert result["servo_state"] is None


def test_missing_grandmaster_or_large_offset_stays_unlocked():
    assert _snapshot("portState SLAVE\nmaster_offset 250\ngmPresent false\n")["locked"] is False
    assert _snapshot("portState SLAVE\nmaster_offset 1000001\ngmPresent true\n")["locked"] is False
