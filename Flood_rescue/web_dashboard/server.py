import http.server
import socketserver
import urllib.parse
import subprocess
import os

PORT = 8000

class FloodServer(http.server.SimpleHTTPRequestHandler):

    def do_GET(self):

        # Handle algorithm run request
        if self.path.startswith("/run"):

            query = urllib.parse.urlparse(self.path).query
            params = urllib.parse.parse_qs(query)

            algo = params.get("algo", ["astar"])[0]

            print("Requested Algorithm:", algo)

            # Map algorithm to MATLAB file
            if algo == "astar":
                matlab_file = "original"

            elif algo == "ara":
                matlab_file = "ARA_algo"

            elif algo == "rrt":
                matlab_file = "RRT_algo"

            else:
                matlab_file = "original"

            try:
                print("Running MATLAB file:", matlab_file)

                subprocess.Popen(
                    ["matlab", "-batch", matlab_file],
                    cwd="../"   # run MATLAB from Flood_rescue folder
                )

                self.send_response(200)
                self.end_headers()
                self.wfile.write(b"Simulation Started")

            except Exception as e:

                self.send_response(500)
                self.end_headers()
                self.wfile.write(str(e).encode())

        else:
            super().do_GET()


# Start server
with socketserver.TCPServer(("", PORT), FloodServer) as httpd:

    print("Flood Rescue Dashboard Running")
    print("Open: http://localhost:8000")

    httpd.serve_forever()