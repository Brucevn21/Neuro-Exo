"""Local mobile UI; authenticated API, no cloud account or Internet required."""

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading

WEB_ROOT = Path(__file__).parent / "web"


class MobileServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, runtime):
        self.runtime = runtime
        self.token = secrets.token_urlsafe(24)
        super().__init__(address, Handler)

    def start(self):
        thread = threading.Thread(target=self.serve_forever, daemon=True)
        thread.start()
        return thread


class Handler(BaseHTTPRequestHandler):
    timeout = 5

    def log_message(self, *_):
        pass  # Do not put trial information or connection tokens in access logs.

    def reply(self, status, data, content_type="application/json", download=None):
        body = json.dumps(data).encode() if content_type == "application/json" else data
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; object-src 'none'; base-uri 'none'; frame-ancestors 'none'")
        if download:
            self.send_header("Content-Disposition", f'attachment; filename="{download}"')
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def authorized(self):
        supplied = self.headers.get("Authorization", "")
        if not secrets.compare_digest(supplied.encode("utf-8"), ("Bearer " + self.server.token).encode("utf-8")):
            self.reply(401, {"error": "Open the complete phone link printed by the Python app."})
            return False
        return True

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        assets = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                  "/style.css": ("style.css", "text/css; charset=utf-8")}
        if path in assets:
            filename, mime = assets[path]
            self.reply(200, (WEB_ROOT / filename).read_bytes(), mime)
            return
        if not self.authorized():
            return
        state = self.server.runtime.snapshot()
        if path == "/api/state":
            self.reply(200, state)
        elif path.startswith("/api/trials/"):
            trial_id = path.removeprefix("/api/trials/")
            trial = next((t for t in state["trials"] if t["id"] == trial_id), None)
            if trial is None:
                self.reply(404, {"error": "Trial not found"})
                return
            try:
                data = (self.server.runtime.controller.log_dir / trial["file"]).read_bytes()
                self.reply(200, data, "text/csv; charset=utf-8", trial["file"])
            except OSError:
                self.reply(404, {"error": "CSV is unavailable"})
        else:
            self.reply(404, {"error": "Not found"})

    def do_POST(self):
        if not self.authorized():
            return
        if self.path != "/api/action":
            self.reply(404, {"error": "Not found"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 2048:
                raise ValueError("Invalid request size")
            body = json.loads(self.rfile.read(size))
            if not isinstance(body, dict):
                raise ValueError("Expected an object")
            action = body.pop("action")
            arguments = {"connect": {"source", "identifier"}, "start": {"mode", "setting", "target"},
                         "stop": {"reason"}, "disconnect": set()}
            if action not in arguments or not set(body) <= arguments[action]:
                raise ValueError("Unknown action or arguments")
            if action == "start" and (type(body.get("setting")) is not int or type(body.get("target")) is not int or not isinstance(body.get("mode"), str)):
                raise ValueError("Start requires mode, integer setting, and integer target")
            if action == "connect" and any(not isinstance(value, str) for value in body.values()):
                raise ValueError("Connection fields must be text")
            if action == "stop" and body.get("reason", "operator_stop") not in ("operator_stop", "operator_ended"):
                raise ValueError("Unknown stop reason")
            self.server.runtime.submit(action, **body)
            self.reply(202, {"accepted": True})
        except (ValueError, KeyError, TypeError) as exc:
            self.reply(400, {"error": str(exc)})
