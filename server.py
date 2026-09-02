#!/usr/bin/env python3
"""
Garuda Hive Localhost Simulation & Web Server
Serves the full authoritative C++20 simulation kernel, REST API, WebSocket telemetry stream,
and web simulator on http://localhost:8000
"""

import os
import sys
from pathlib import Path

# Add MSYS2 UCRT runtime to DLL search path on Windows
if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
    ucrt_bin = Path(r"C:\msys64\ucrt64\bin")
    if ucrt_bin.exists():
        os.add_dll_directory(str(ucrt_bin))

PORT = 8000
ROOT_DIR = Path(__file__).parent
WEB_DIR = ROOT_DIR / "web"

def main():
    port_arg = int(sys.argv[1]) if len(sys.argv) > 1 else PORT
    try:
        import uvicorn
        import garuda_server
        print("=" * 65)
        print(" [*] Launching GARUDA HIVE V2 Authoritative Simulation Server")
        print(f" [*] Web Simulator & API URL: http://localhost:{port_arg}")
        print(f" [*] WebSocket Stream: ws://localhost:{port_arg}/ws/telemetry")
        print(" [*] C++20 Physics Engine: Active (400 Hz)")
        print("=" * 65)
        uvicorn.run(garuda_server.app, host="0.0.0.0", port=port_arg, log_level="info")
    except Exception as e:
        print(f"[!] Falling back to basic HTTP server due to: {e}")
        import http.server
        import socketserver
        os.chdir(str(WEB_DIR))
        socketserver.TCPServer.allow_reuse_address = True
        with socketserver.TCPServer(("0.0.0.0", port_arg), http.server.SimpleHTTPRequestHandler) as httpd:
            print(f"[*] Basic static web server running at http://localhost:{port_arg}")
            try:
                httpd.serve_forever()
            except KeyboardInterrupt:
                pass

if __name__ == "__main__":
    main()
