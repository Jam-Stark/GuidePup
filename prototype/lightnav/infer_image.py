#!/usr/bin/env python3
"""Send one real RGB image to the official LightNav WebSocket service."""

import argparse
import base64
import json
from pathlib import Path
import time

from websockets.sync.client import connect


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default="ws://127.0.0.1:8050")
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    def request(ws, action: str, data: dict) -> dict:
        ws.send(json.dumps({"action": action, "data": data}))
        response = json.loads(ws.recv(timeout=120))
        if response["data"]["rc"] != 0:
            raise RuntimeError(response)
        return response

    with connect(args.server, max_size=64 * 1024 * 1024) as ws:
        request(ws, "login", {"clientId": "forcecontrol-image"})
        request(ws, "reset", {})
        started = time.monotonic()
        response = request(
            ws,
            "next",
            {
                "seq": 0,
                "image": base64.b64encode(args.image.read_bytes()).decode("ascii"),
                "instruction": args.instruction,
            },
        )
        result = {
            "image": str(args.image.resolve()),
            "instruction": args.instruction,
            "client_wall_ms": (time.monotonic() - started) * 1000,
            "response": response,
        }
    serialized = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized + "\n")
    print(serialized)


if __name__ == "__main__":
    main()
