import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from tools.fusion_mcp import FusionClient, FusionError


def test_unreachable_server_message():
    with pytest.raises(FusionError, match="FUSION_MCP_URL"):
        FusionClient("http://127.0.0.1:9/mcp", timeout=2).run_script("def run(_c): pass")


def _stub(tool_payload: dict):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            self.send_response(200)
            if body.get("method") == "tools/call":
                result = {"content": [{"type": "text", "text": json.dumps(tool_payload)}]}
                data = json.dumps({"jsonrpc": "2.0", "id": body["id"], "result": result})
                self.send_header("Content-Type", "text/event-stream")
                self.end_headers()
                self.wfile.write(f"event: message\ndata: {data}\n\n".encode())
            elif "id" in body:
                self.send_header("Content-Type", "application/json")
                self.send_header("Mcp-Session-Id", "abc")
                self.end_headers()
                self.wfile.write(
                    json.dumps({"jsonrpc": "2.0", "id": body["id"], "result": {}}).encode()
                )
            else:
                self.end_headers()

    server = HTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, f"http://127.0.0.1:{server.server_port}/mcp"


def test_parses_sse_and_json():
    server, url = _stub({"message": "hi\n", "success": True})
    try:
        assert FusionClient(url, timeout=5).run_script("x") == "hi\n"
    finally:
        server.shutdown()
    server, url = _stub({"message": "boom", "success": False})
    try:
        with pytest.raises(FusionError, match="boom"):
            FusionClient(url, timeout=5).run_script("x")
    finally:
        server.shutdown()
