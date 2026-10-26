import profile_capture as pc
import pytest

LINES = [
    "P,1000,period,tx_slot,152,995000,15100,6300,11800",
    "P,1000,period,drive_loop,160,1000000,9000,6000,8500",
    "P,1000,section,irq_late,10000,3000,31",
    "P,1000,counter,tx_frames,152",
    "P,2000,period,tx_slot,200,1000000,5100,5050,5100",
    "P,2000,period,drive_loop,1000,1000000,1200,1050,1100",
    "P,2000,section,irq_late,10000,1000,4",
    "P,2000,counter,tx_frames,200",
]


def records():
    return [pc.parse_line(line) for line in LINES]


def test_parse_line_kinds():
    assert pc.parse_line(LINES[0]) == {
        "t_ms": 1000,
        "kind": "period",
        "name": "tx_slot",
        "n": 152,
        "sum_us": 995000,
        "max_us": 15100,
        "p50_us": 6300,
        "p99_us": 11800,
    }
    assert pc.parse_line(LINES[2])["max_us"] == 31
    assert pc.parse_line(LINES[3])["value"] == 152


@pytest.mark.parametrize(
    "line",
    ["", ">Target_MPS:0.5", "P,1000,period,tx_slot,1,2", "P,x,counter,a,1", "P,1,mystery,a,1"],
)
def test_parse_line_ignores_other_output(line):
    assert pc.parse_line(line) is None


def test_covered_seconds_uses_report_times():
    assert pc.covered_seconds([]) == 0.0
    assert pc.covered_seconds([5000]) == 1.0
    assert pc.covered_seconds([1000, 2000]) == pytest.approx(2.0)
    assert pc.covered_seconds([1000, 2025, 3050, 4075]) == pytest.approx(4.1)


def test_rate_is_not_inflated_by_long_windows():
    lines = [f"P,{t},period,tx_slot,205,1000000,5100,5050,5100" for t in (1000, 2025, 3050)]
    s = pc.summarize(pc.parse_line(line) for line in lines)
    assert s.windows == 3
    assert s.periods["tx_slot"]["rate_hz"] == pytest.approx(200.0)


def test_summarize_combines_seconds():
    s = pc.summarize(records())
    assert s.windows == 2
    assert s.seconds == pytest.approx(2.0)
    tx = s.periods["tx_slot"]
    assert tx["rate_hz"] == pytest.approx(176.0)
    assert tx["mean_us"] == pytest.approx(1995000 / 352)
    assert tx["max_us"] == 15100
    assert tx["p99_us"] == 11800
    assert tx["p50_us"] == pytest.approx((6300 + 5050) / 2)
    assert s.sections["irq_late"]["max_us"] == 31
    assert s.counters["tx_frames"]["per_s"] == pytest.approx(176.0)


def test_targets_flag_the_bad_numbers():
    results = {t.label: ok for t, _, ok in pc.check_targets(pc.summarize(records()))}
    assert results["telemetry rate"] is False
    assert results["telemetry gap max"] is False
    assert results["drive loop period max"] is False
    assert results["command to PWM max"] is None


def test_targets_pass_on_a_good_run():
    good = [
        pc.parse_line("P,1000,period,tx_slot,200,1000000,5100,5050,5100"),
        pc.parse_line("P,1000,period,drive_loop,1000,1000000,1200,1050,1100"),
        pc.parse_line("P,1000,section,cmd_to_pwm,60,30000,900"),
        pc.parse_line("P,1000,section,irq_late,10000,1000,4"),
    ]
    assert all(ok for _, _, ok in pc.check_targets(pc.summarize(good)))


def test_summary_reports_pi_side_loss():
    text = pc.format_summary(pc.summarize(records()), pi_rate_hz=158.4)
    assert "Pi received 158.4 Hz of 176.0 Hz sent (10.0% lost)" in text


def test_csv_row_keeps_every_field():
    assert (
        pc.to_csv_row(pc.parse_line(LINES[0]))
        == "1000,period,tx_slot,152,995000,15100,6300,11800,"
    )
    assert pc.to_csv_row(pc.parse_line(LINES[3])) == "1000,counter,tx_frames,,,,,,152"
