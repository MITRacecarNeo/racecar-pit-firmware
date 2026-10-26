import build_flags
import pytest
import telemetry_probe


def test_latest_label_takes_the_newest_labeled_subject():
    subjects = "docs: typo\n[v0.9.0.4-pcb] link: x\n[v0.9.0.3-pcb] pit: y\n"
    assert build_flags.latest_label(subjects) == "v0.9.0.4-pcb"


def test_latest_label_is_dev_without_labels():
    assert build_flags.latest_label("pit: v0.9.0; info reply\nMerge pull request #8\n") == "dev"


@pytest.mark.parametrize("subject", ["[v0.9.0-pcb] x", "[v0.9.0.3] x", " [v0.9.0.3-pcb] x"])
def test_latest_label_ignores_malformed_labels(subject):
    assert build_flags.latest_label(subject) == "dev"


def test_build_flags_carry_all_three_defines():
    flags = build_flags.build_flags().split()
    assert [f.split("=")[0] for f in flags] == ["-DFW_GIT_HASH", "-DFW_GIT_DIRTY", "-DFW_LABEL"]


def test_gap_stats_on_a_steady_stream():
    stamps = [i * 0.005 for i in range(201)]
    stats = telemetry_probe.gap_stats(stamps)
    assert stats is not None
    assert stats.count == 200
    assert stats.median == pytest.approx(5.0)
    assert stats.max == pytest.approx(5.0)
    assert stats.rate_hz == pytest.approx(200.0)


def test_gap_stats_sees_one_missed_slot():
    stamps = [i * 0.005 for i in range(100)] + [0.505 + i * 0.005 for i in range(100)]
    stats = telemetry_probe.gap_stats(stamps)
    assert stats is not None
    assert stats.max == pytest.approx(10.0)
    assert stats.median == pytest.approx(5.0)


def test_gap_stats_needs_three_stamps():
    assert telemetry_probe.gap_stats([0.0, 0.005]) is None


def test_message_rate_ignores_the_wait_before_the_first_message():
    arrivals = [3.0 + i * 0.005 for i in range(1401)]
    assert telemetry_probe.message_rate(arrivals) == pytest.approx(200.0)
    assert telemetry_probe.message_rate([1.0]) == 0.0


def test_count_changes():
    assert telemetry_probe.count_changes([]) == 0
    assert telemetry_probe.count_changes([1, 1, 2, 2, 2, 3, 1]) == 4
