"""Print the git build flags for the firmware; PlatformIO runs it from platformio.ini."""

from __future__ import annotations

import subprocess


def _git(*args: str) -> str:
    try:
        out = subprocess.run(["git", *args], capture_output=True, text=True, check=True)
    except (OSError, subprocess.CalledProcessError):
        return ""
    return out.stdout.strip()


def build_flags() -> str:
    commit = _git("rev-parse", "--short=7", "HEAD") or "unknown"
    dirty = 1 if _git("status", "--porcelain", "--untracked-files=no") else 0
    return f"-DFW_GIT_HASH={commit} -DFW_GIT_DIRTY={dirty}"


if __name__ == "__main__":
    print(build_flags())
