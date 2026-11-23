"""Minimal client for the Fusion 360 MCP server: run Fusion API Python in the open design.

Usage:
    uv run python -m tools.fusion_mcp run path/to/script.py [--read-only]

The server URL comes from FUSION_MCP_URL (default http://127.0.0.1:50463/mcp). Fusion picks
a new port when it restarts; find it with: lsof -nP -iTCP -sTCP:LISTEN | grep Autodesk
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_URL = "http://127.0.0.1:50463/mcp"


class FusionError(RuntimeError):
    """The Fusion MCP server is unreachable or the script failed."""


class FusionClient:
    def __init__(self, url: str | None = None, timeout: float = 300):
        self.url = url or os.environ.get("FUSION_MCP_URL", DEFAULT_URL)
        self.timeout = timeout
        self._session: str | None = None
        self._id = 0

    def _post(self, body: dict) -> dict | None:
        headers = {
            "Content-Type": "application/json",
            "Accept": "application/json, text/event-stream",
        }
        if self._session:
            headers["Mcp-Session-Id"] = self._session
        req = urllib.request.Request(self.url, json.dumps(body).encode(), headers)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                self._session = resp.headers.get("Mcp-Session-Id") or self._session
                raw = resp.read().decode()
        except (urllib.error.URLError, OSError) as e:
            raise FusionError(
                f"cannot reach the Fusion MCP server at {self.url} ({e}). Is Fusion open with its "
                "MCP add-in running? If Fusion restarted, its port changed: set FUSION_MCP_URL."
            ) from e
        if not raw.strip():
            return None
        if "data:" in raw and not raw.lstrip().startswith("{"):
            raw = "\n".join(
                line[5:].strip() for line in raw.splitlines() if line.startswith("data:")
            )
        return json.loads(raw)

    def _request(self, method: str, params: dict | None = None) -> dict:
        self._id += 1
        msg = self._post(
            {"jsonrpc": "2.0", "id": self._id, "method": method, "params": params or {}}
        )
        if msg is None or "error" in msg:
            raise FusionError(f"{method} failed: {msg and msg.get('error')}")
        return msg["result"]

    def _connect(self) -> None:
        if self._session is not None:
            return
        self._request(
            "initialize",
            {
                "protocolVersion": "2025-06-18",
                "capabilities": {},
                "clientInfo": {"name": "rocky-walker", "version": "0.1"},
            },
        )
        self._post({"jsonrpc": "2.0", "method": "notifications/initialized"})
        self._session = self._session or ""

    def ping(self) -> bool:
        try:
            self._connect()
            return True
        except FusionError:
            return False

    def run_script(self, source: str, read_only: bool = False) -> str:
        """Run a Fusion API script (must define ``run(_context)``); return what it printed."""
        self._connect()
        result = self._request(
            "tools/call",
            {
                "name": "fusion_mcp_execute",
                "arguments": {
                    "featureType": "script",
                    "object": {"script": source, "readOnly": read_only},
                },
            },
        )
        text = "".join(c.get("text", "") for c in result.get("content", []))
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            payload = {"message": text, "success": not result.get("isError")}
        if result.get("isError") or not payload.get("success", False):
            raise FusionError(f"Fusion script failed:\n{payload.get('message') or payload}")
        return payload.get("message", "")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.fusion_mcp")
    sub = parser.add_subparsers(dest="cmd", required=True)
    run = sub.add_parser("run", help="run a Fusion API script file")
    run.add_argument("script", type=Path)
    run.add_argument("--read-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        print(FusionClient().run_script(args.script.read_text(), read_only=args.read_only), end="")
    except FusionError as e:
        print(e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
