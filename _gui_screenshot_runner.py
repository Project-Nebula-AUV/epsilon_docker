#!/usr/bin/env python3
"""Scratch helper (not part of the package): runs the same loop as
simulator_node.py's main(), but also serves the live pygame screen over
a small local HTTP server so the sim GUI can be viewed in a browser
without a real display attached. Safe to delete after use.

    python3 _gui_screenshot_runner.py [port]
    Then browse to http://<sub-host>:<port>/
"""
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pygame
import rclpy
from PIL import Image

from robosub.nodes.simulator_node import SimulatorNode

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else int(os.environ.get('SIM_WEB_PORT', '8765'))
SAVE_INTERVAL = float(os.environ.get('SIM_SCREENSHOT_INTERVAL', '0.5'))
FRAME_PATH = os.environ.get('SIM_FRAME_PATH', '/tmp/sim_live_frame.png')

PAGE = b"""<!DOCTYPE html>
<html><head><title>RoboSub Sim (live)</title>
<style>
  body { background:#111; margin:0; display:flex; justify-content:center;
         align-items:center; height:100vh; }
  img { max-width:100%; max-height:100%; }
</style></head>
<body><img id="f" src="/frame.png">
<script>
setInterval(() => {
  document.getElementById('f').src = '/frame.png?' + Date.now();
}, 400);
</script>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path.startswith('/frame.png'):
            try:
                with open(FRAME_PATH, 'rb') as f:
                    data = f.read()
            except FileNotFoundError:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/png')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)


def _serve_forever():
    httpd = ThreadingHTTPServer(('127.0.0.1', PORT), Handler)
    httpd.serve_forever()


def main():
    threading.Thread(target=_serve_forever, daemon=True).start()
    print(f"[gui] serving live sim view on http://127.0.0.1:{PORT}/ (localhost only)", flush=True)

    rclpy.init()
    node = SimulatorNode()
    sim = node.sim
    clock = pygame.time.Clock()
    last_shot = 0.0

    try:
        while sim.running and rclpy.ok():
            dt = clock.tick(60) / 1000.0
            if dt > 0.1:
                dt = 0.1
            rclpy.spin_once(node, timeout_sec=0)

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    node.publish_sim_control('quit')

            if not sim.paused:
                sim.generateCameraView()
                sim.applyPhysics(dt, node._commands)
                sim.lastThrusterCommands = node._commands
                node.publish_sensors(dt)
                sim.render()

                now = time.time()
                if now - last_shot >= SAVE_INTERVAL:
                    last_shot = now
                    # pygame.image.save() silently writes TGA (not PNG) on
                    # this SDL build even when given a .png path, and
                    # browsers can't decode TGA in <img> tags. Encode a real
                    # PNG via Pillow instead.
                    w, h = sim.screen.get_size()
                    raw = pygame.image.tostring(sim.screen, 'RGB')
                    img = Image.frombytes('RGB', (w, h), raw)
                    tmp_path = FRAME_PATH + '.tmp'
                    img.save(tmp_path, format='PNG')
                    os.replace(tmp_path, FRAME_PATH)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
