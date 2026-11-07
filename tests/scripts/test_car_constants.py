import argparse
import datetime
import io
import json

import car_constants as cc
import pytest


class FakePort:
    """Answers each written line from a script of reply lists."""

    def __init__(self, *replies):
        self.replies = list(replies)
        self.written = []
        self.pending = []

    def write(self, data):
        self.written.append(data.decode())
        self.pending = [f"{r}\n".encode() for r in self.replies.pop(0)]
        return len(data)

    def readline(self):
        return self.pending.pop(0) if self.pending else b""


GET_REPLY = [
    "cal.source=builtin",
    "cal.format=1",
    "cal.eeprom=empty",
    "cal.crc=none",
    "cal.written=none",
    "pid.ff_fwd_us=69.000",
    "pid.kp=20.000",
    "ok get",
]


def args(verb, *pairs, date=None):
    ns = argparse.Namespace(verb=verb, pairs=list(pairs), date=date)
    if verb in ("export", "import"):
        ns.file = pairs[0]
    return ns


def test_command_returns_lines_through_ok():
    port = FakePort(GET_REPLY)
    assert cc.command(port, "cal get") == GET_REPLY
    assert port.written == ["cal get\n"]


def test_command_raises_on_err():
    with pytest.raises(cc.ConstantsError, match="car moving"):
        cc.command(FakePort(["err car moving"]), "cal set pid.kp=1")


def test_command_times_out_without_ok():
    ticks = iter(range(100))
    with pytest.raises(TimeoutError):
        cc.command(FakePort([]), "cal get", timeout=3, clock=lambda: next(ticks))


def test_parse_get_and_constant_keys():
    values = cc.parse_get(GET_REPLY)
    assert values["cal.source"] == "builtin"
    assert cc.constant_keys(values) == {"pid.ff_fwd_us": 69.0, "pid.kp": 20.0}


def test_set_line_is_sorted_and_compact():
    assert (
        cc.set_line({"pid.kp": 20.0, "pid.ff_fwd_us": 69.5})
        == "cal set pid.ff_fwd_us=69.5 pid.kp=20"
    )


def test_save_uses_today_unless_given():
    assert cc.today_ymd(datetime.date(2026, 11, 6)) == "20261106"
    port = FakePort(["cal.crc=beef", "ok save"])
    out = io.StringIO()
    cc.run(port, args("save", date="20261106"), out)
    assert port.written == ["cal save 20261106\n"]
    assert "cal.crc=beef ok save" in out.getvalue()


def test_export_then_import_round_trip(tmp_path):
    f = tmp_path / "car.json"
    cc.run(FakePort(GET_REPLY), args("export", str(f)), io.StringIO())
    doc = json.loads(f.read_text())
    assert doc["source"] == "builtin"
    assert doc["constants"] == {"pid.ff_fwd_us": 69.0, "pid.kp": 20.0}

    port = FakePort(["ok set"], ["cal.crc=1234", "ok save"])
    cc.run(port, args("import", str(f), date="20261106"), io.StringIO())
    assert port.written == ["cal set pid.ff_fwd_us=69 pid.kp=20\n", "cal save 20261106\n"]


def test_set_passes_pairs_through():
    port = FakePort(["ok set"])
    cc.run(port, args("set", "pid.ff_rev_us=110", "pid.kv_ff_rev=28"), io.StringIO())
    assert port.written == ["cal set pid.ff_rev_us=110 pid.kv_ff_rev=28\n"]
