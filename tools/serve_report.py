#!/usr/bin/env python3
"""Lightweight HTTP server for viewing simulation reports on iPad / mobile browsers."""
import os
import sys
import glob
from http.server import HTTPServer, SimpleHTTPRequestHandler

PORT = 8080
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

class ReportHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=self.get_latest_dir(), **kwargs)

    @classmethod
    def get_latest_dir(cls):
        latest = os.path.join(ROOT, "runs", "latest")
        if os.path.exists(latest):
            return latest
        all_runs = sorted(glob.glob(os.path.join(ROOT, "runs", "20*")))
        if all_runs:
            return all_runs[-1]
        return ROOT

    def do_GET(self):
        self.directory = self.get_latest_dir()
        if self.path in ("/", "/index.html"):
            report_file = os.path.join(self.directory, "report.html")
            if os.path.exists(report_file):
                self.path = "/report.html"
            else:
                self.send_response(200)
                self.send_header("Content-type", "text/html; charset=utf-8")
                self.end_headers()
                self.wfile.write(b"""
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>LineSim Telemetry</title>
<style>
  body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #121214; color: #eee; padding: 24px; text-align: center; }
  .card { background: #1e1e24; border-radius: 12px; padding: 24px; max-width: 500px; margin: 40px auto; box-shadow: 0 4px 20px rgba(0,0,0,0.5); }
  h1 { font-size: 20px; color: #4af; }
  p { font-size: 15px; color: #aaa; line-height: 1.6; }
  code { background: #2a2a35; padding: 4px 8px; border-radius: 6px; color: #4af; }
</style>
</head>
<body>
<div class="card">
  <h1>LineSim iPad Report Server</h1>
  <p>No run completed yet.</p>
  <p>Run a scenario in the terminal:<br><br><code>./linesim run --track tracks/all_elements.json</code></p>
  <p>Then refresh this page to view trajectory curves and telemetry!</p>
</div>
</body>
</html>
""")
                return
        return super().do_GET()

def run():
    server_address = ("", PORT)
    httpd = HTTPServer(server_address, ReportHandler)
    print(f"[LineSim] Serving reports at http://localhost:{PORT} (dir: {ReportHandler.get_latest_dir()})")
    print(f"Open port {PORT} in VS Code Ports tab or Safari on iPad.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[LineSim] Report server stopped.")

if __name__ == "__main__":
    run()
