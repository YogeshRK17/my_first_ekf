from http.server import SimpleHTTPRequestHandler
from socketserver import TCPServer

HOST = "0.0.0.0"
PORT = 8080

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory="public", **kwargs)

with TCPServer((HOST, PORT), Handler) as httpd:
    print(f"Behavior Tree Visualizer running at http://{HOST}:{PORT}")
    httpd.serve_forever()
