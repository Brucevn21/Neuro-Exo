#!/usr/bin/env python3
"""Run the NeuroExo desktop app and optional phone interface."""

import argparse
from pathlib import Path
import socket

from controller import Runtime


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mobile", action="store_true", help="Enable phone access on the local Wi-Fi network")
    parser.add_argument("--headless", action="store_true", help="Only run the web interface")
    parser.add_argument("--host", help="Web bind address (default: 0.0.0.0 with --mobile, otherwise 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--log-dir", type=Path, default=Path(__file__).parent / "telemetry_data")
    args = parser.parse_args()
    if args.host and not (args.mobile or args.headless):
        parser.error("--host requires --mobile or --headless")
    runtime = Runtime(args.log_dir)
    server = None
    try:
        mobile_url = None
        if args.mobile or args.headless:
            from server import MobileServer
            host = args.host or ("0.0.0.0" if args.mobile else "127.0.0.1")
            server = MobileServer((host, args.port), runtime)
            server.start()
            port = server.server_address[1]
            print(f"Computer: http://127.0.0.1:{port}/#{server.token}", flush=True)
            if host == "0.0.0.0":
                addresses = sorted({item[4][0] for item in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET) if not item[4][0].startswith("127.")})
                links = [f"http://{ip}:{port}/#{server.token}" for ip in addresses]
                mobile_url = links[0] if links else f"http://<computer Wi-Fi IPv4>:{port}/#{server.token}"
                for link in links or [mobile_url]:
                    print(f"Phone (same Wi-Fi): {link}", flush=True)
            else:
                mobile_url = f"http://{host}:{port}/#{server.token}"
            print("Keep the complete link private; it grants control of this device.", flush=True)
        print(f"Trial files: {args.log_dir.resolve()}", flush=True)
        if args.headless:
            import threading
            threading.Event().wait()
        else:
            import tkinter as tk
            from desktop import Desktop
            root = tk.Tk()
            app = Desktop(root, runtime, mobile_url)

            def close():
                app.closed = True
                root.destroy()

            root.protocol("WM_DELETE_WINDOW", close)
            root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if server:
            server.shutdown()
            server.server_close()
        runtime.close()


if __name__ == "__main__":
    main()
