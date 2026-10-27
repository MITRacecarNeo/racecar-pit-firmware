from pathlib import Path

import upload


class FakeRunner:
    """Answers each loader call from a list of (code, output); records the calls."""

    def __init__(self, loader_results):
        self.results = list(loader_results)
        self.calls = []

    def __call__(self, cmd, timeout):
        self.calls.append(list(cmd))
        if cmd[0].endswith("teensy_reboot"):
            return 1, "Unable to soft reboot"
        return self.results.pop(0)


TOOLS = Path("/tools")
HEX = Path("firmware.hex")
OK = (0, "Found HalfKay Bootloader\nProgramming....\nBooting\n")
WRITE_ERROR = (1, "Found HalfKay Bootloader\nProgramming...error writing to Teensy\n")


def test_first_write_fails_second_succeeds():
    runner = FakeRunner([WRITE_ERROR, OK])
    assert upload.upload(TOOLS, HEX, runner=runner) == 2
    assert runner.calls[0] == ["/tools/teensy_reboot", "-s"]
    assert runner.calls[1] == [
        "/tools/teensy_loader_cli",
        "-mmcu=TEENSY41",
        "-w",
        "-v",
        "firmware.hex",
    ]
    assert len(runner.calls) == 3


def test_success_on_the_first_write_stops_there():
    runner = FakeRunner([OK])
    assert upload.upload(TOOLS, HEX, runner=runner) == 1
    assert len(runner.calls) == 2


def test_gives_up_after_the_attempt_limit():
    runner = FakeRunner([WRITE_ERROR] * 3)
    assert upload.upload(TOOLS, HEX, attempts=3, runner=runner) == 0
    assert len(runner.calls) == 4


def test_an_error_in_the_output_counts_as_failure_even_with_exit_zero():
    assert not upload.write_ok(0, "error writing to Teensy")
    assert upload.write_ok(0, "Booting")
    assert not upload.write_ok(1, "Booting")
