#!/usr/bin/env python3
"""
SkySim Localhost Web Server
Serves the SkySim web simulator on http://localhost:8000
"""

import http.server
import socketserver
import os
import sys
import webbrowser
from pathlib import Path

PORT = 8000
WEB_DIR = Path(__file__).parent / "web"

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(WEB_DIR), **kwargs)

    def end_headers(self):
        # Enable CORS and disable aggressive caching for local development
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def guess_type(self, path):
        # Ensure proper MIME types for wasm and js modules
        if path.endswith(".wasm"):
            return "application/wasm"
        if path.endswith(".js"):
            return "application/javascript"
        return super().guess_type(path)


def run_server(port=PORT):
    os.chdir(str(WEB_DIR))
    
    # Try port, or increment if in use
    current_port = port
    max_attempts = 10
    httpd = None
    
    for attempt in range(max_attempts):
        try:
            socketserver.TCPServer.allow_reuse_address = True
            httpd = socketserver.TCPServer(("0.0.0.0", current_port), CustomHTTPRequestHandler)
            break
        except OSError:
            print(f"Port {current_port} is busy, trying {current_port + 1}...")
            current_port += 1

    if not httpd:
        print("Error: Could not bind to an open port.")
        sys.exit(1)

    url = f"http://localhost:{current_port}"
    print("=" * 65)
    print(f"[*] SkySim Drone Simulator is live on localhost!")
    print(f"[*] URL: {url}")
    print(f"[*] Serving directory: {WEB_DIR}")
    print(f"[*] Controls: W/S throttle, A/D yaw, Arrows pitch/roll, Space to arm")
    print("=" * 65)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping SkySim server...")
        httpd.server_close()


if __name__ == "__main__":
    port_arg = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    run_server(port_arg)
