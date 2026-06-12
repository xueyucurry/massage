from __future__ import annotations

import argparse
import sys
from pathlib import Path


def _append_realman_root() -> None:
    for parent in Path(__file__).resolve().parents:
        if (parent / "rm_demo").is_dir():
            if str(parent) not in sys.path:
                sys.path.append(str(parent))
            return


_append_realman_root()


def main() -> None:
    parser = argparse.ArgumentParser(description="Clear RealMan six-axis force zero via JSON controller.")
    parser.add_argument("--host", default="192.168.1.18")
    args = parser.parse_args()

    from rm_demo.rm_json import query_json

    reply = query_json(args.host, {"command": "clear_force_data"}, timeout=2.0)
    print(reply)
    if not bool(reply.get("clear_state", False)):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
