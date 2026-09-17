from __future__ import annotations

import json
import os
import re
import shutil
import time
from pathlib import Path
from typing import Any


def wall_time_ms() -> int:
    return time.time_ns() // 1_000_000


def monotonic_ns() -> int:
    return time.monotonic_ns()


class ServiceError(Exception):
    def __init__(self, status: int, code: str, message: str):
        super().__init__(message)
        self.status = status
        self.code = code
        self.message = message

    def payload(self) -> dict[str, Any]:
        return {"error": {"code": self.code, "message": self.message}}


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def append_jsonl(path: Path, value: Any) -> None:
    with path.open("a", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False))
        stream.write("\n")
        stream.flush()


def load_json(path: Path, default: Any) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return default


def safe_label(value: object) -> str:
    if not isinstance(value, str):
        return ""
    value = value.strip()[:80]
    value = re.sub(r"[^0-9A-Za-z\u4e00-\u9fff_-]+", "-", value)
    return value.strip("-_")[:48]


def directory_size(path: Path) -> int:
    total = 0
    try:
        for item in path.rglob("*"):
            if item.is_file():
                total += item.stat().st_size
    except OSError:
        pass
    return total


def disk_status(path: Path) -> dict[str, int | float]:
    path.mkdir(parents=True, exist_ok=True)
    usage = shutil.disk_usage(path)
    return {
        "total_bytes": usage.total,
        "used_bytes": usage.used,
        "free_bytes": usage.free,
        "free_gb": round(usage.free / (1024**3), 2),
    }


def fourcc_text(value: float) -> str:
    number = int(value) & 0xFFFFFFFF
    return "".join(chr((number >> (8 * index)) & 0xFF) for index in range(4))

