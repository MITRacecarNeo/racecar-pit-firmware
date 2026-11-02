import esc_cal
import pytest


class FakeTeensy:
    """The calibration build's command rules (src/esc_cal.cpp) behind a serial port."""

    PWM = {"neutral": 1500, "full_forward": 2000, "full_reverse": 1000, "done": 1500}

    def __init__(self, reply=True):
        self.step = "neutral"
        self.aborted = False
        self.reply = reply
        self.out = b""
        self.sent = []

    def write(self, data):
        for c in data.decode():
            self.sent.append(c)
            if c == "n":
                self.step = "done" if self.step == "full_reverse" else "neutral"
                self.aborted = False
            elif c == "f" and self.step in ("neutral", "done"):
                self.step = "full_forward"
            elif c == "r" and self.step == "full_forward":
                self.step = "full_reverse"
            if self.reply:
                pwm, aborted = self.PWM[self.step], int(self.aborted)
                self.out += f"esc_cal step={self.step} pwm={pwm} aborted={aborted}\n".encode()

    def flush(self):
        pass

    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n):
        data, self.out = self.out[:n], self.out[n:]
        return data


@pytest.fixture(autouse=True)
def no_wait(monkeypatch):
    monkeypatch.setattr(esc_cal, "FINISH_WAIT_SEC", 0.0)
    monkeypatch.setattr(esc_cal, "REPLY_TIMEOUT_SEC", 0.2)


def answers(*replies):
    it = iter(replies)
    return lambda prompt: next(it)


def test_parse_status():
    assert esc_cal.parse_status("esc_cal step=full_forward pwm=2000 aborted=0\r\n") == (
        esc_cal.Status("full_forward", 2000, False)
    )
    assert esc_cal.parse_status("esc_cal step=neutral pwm=1500 aborted=1") == esc_cal.Status(
        "neutral", 1500, True
    )
    assert esc_cal.parse_status(">Sent_PWM:1500") is None


def test_full_sequence_ends_at_neutral():
    fake = FakeTeensy()
    link = esc_cal.EscCalLink(fake)
    assert esc_cal.run(link, answers("y", "", "", "")) == 0
    steps = [c for c in fake.sent if c != "?"]
    assert steps == ["n", "f", "r", "n"]
    assert fake.step == "done"


def test_no_throttle_unless_the_esc_is_in_calibration_mode():
    fake = FakeTeensy()
    link = esc_cal.EscCalLink(fake)
    assert esc_cal.run(link, answers("n")) == 1
    assert "f" not in fake.sent and "r" not in fake.sent


def test_no_reply_means_wrong_build():
    link = esc_cal.EscCalLink(FakeTeensy(reply=False))
    assert esc_cal.run(link, answers("y")) == 1


def test_fallback_to_neutral_stops_the_run():
    fake = FakeTeensy()
    link = esc_cal.EscCalLink(fake)

    def read(prompt):
        if "twice" in prompt:
            fake.step, fake.aborted = "neutral", True
            fake.write(b"?")
        return "y" if "?" in prompt else ""

    assert esc_cal.run(link, read) == 1
    assert "r" not in fake.sent


def test_close_sends_neutral():
    fake = FakeTeensy()
    link = esc_cal.EscCalLink(fake)
    link.close()
    assert fake.sent[-1] == "n"
