"""Inspect collected episodes without changing their contents."""

import argparse
import json
from pathlib import Path

from zekeep_embodied.dataset import inspect_episode


def audit(root, output):
    paths = sorted(root.glob("episode_*"))
    if not paths:
        raise ValueError(f"No episodes in {root}")
    report = {"root": str(root.resolve()), "valid": [], "excluded": []}
    for path in paths:
        try:
            if path.name.endswith(".partial") or path.resolve().name.endswith(".partial"):
                raise ValueError("Incomplete episode (.partial)")
            metadata, rows, _ = inspect_episode(path)
            if metadata["status"] != "success" or metadata.get("simulated", False):
                raise ValueError("Not a successful real capture")
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            report["excluded"].append({"episode": path.name, "reason": str(exc)})
        else:
            report["valid"].append({"episode": path.name, "frames": len(rows), "task": metadata["task"]})
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.root, args.output)
    print(json.dumps({"valid": len(report["valid"]), "excluded": len(report["excluded"])}, indent=2))


if __name__ == "__main__":
    main()
