"""Print the git build flags for the firmware; PlatformIO runs it from platformio.ini."""

from __future__ import annotations

import re
import subprocess

# Internal commit labels: "[v0.9.0.3-pcb] scope: summary".
LABEL = re.compile(r"^\[(v\d+\.\d+\.\d+\.\d+-pcb)\]")


def _git(*args: str) -> str:
    try:
        out = subprocess.run(["git", *args], capture_output=True, text=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return ""
    return out.stdout.strip()


def latest_label(subjects: str) -> str:
    """Label of the newest labeled subject in `git log` order, or "dev"."""
    for subject in subjects.splitlines():
        match = LABEL.match(subject)
        if match:
            return match.group(1)
    return "dev"


def build_flags() -> str:
    commit = _git("rev-parse", "--short=7", "HEAD") or "unknown"
    dirty = 1 if _git("status", "--porcelain", "--untracked-files=no") else 0
    label = latest_label(_git("log", "--format=%s", "-n", "200"))
    return f"-DFW_GIT_HASH={commit} -DFW_GIT_DIRTY={dirty} -DFW_LABEL={label}"


if __name__ == "__main__":
    print(build_flags())
