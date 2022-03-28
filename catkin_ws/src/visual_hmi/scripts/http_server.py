#! /usr/bin/env python3

import http.server
import socketserver

class MyRequestHandler(http.server.SimpleHTTPRequestHandler):

    def do_GET(self):
        if self.path == "/":
            self.path += "gui.html"

        print("path: " + self.path)

        return http.server.SimpleHTTPRequestHandler.do_GET(self)

requestHandler = MyRequestHandler
server = socketserver.TCPServer(('0.0.0.0', 8090), requestHandler)

try:
	server.serve_forever()
except:
	server.socket.close()