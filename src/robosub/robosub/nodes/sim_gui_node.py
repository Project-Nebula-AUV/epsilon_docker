#!/usr/bin/env python3
"""Simulator with a live browser view — the canonical way to run sim tests.

Runs the same loop as simulator_node.py's main() and additionally serves the
pygame screen as a self-refreshing PNG over HTTP, so every run can be watched
from a browser with no display attached:

    ros2 run robosub sim_gui_node [--ros-args -p fuse_depth:=true ...]
    Then browse to http://<sub-host>:8765/

Port from SIM_WEB_PORT (default 8765). Binds all interfaces — the sub lives
on a trusted local network and the page is read-only.
"""
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pygame
import rclpy

from robosub.nodes.simulator_node import SimulatorNode

PORT = int(os.environ.get('SIM_WEB_PORT', '8765'))
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
    httpd = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    httpd.serve_forever()


def main(args=None):
    threading.Thread(target=_serve_forever, daemon=True).start()
    print(f"[gui] live sim view on http://0.0.0.0:{PORT}/", flush=True)

    rclpy.init(args=args)
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
                    # PNG via OpenCV instead.
                    frame = np.transpose(
                        pygame.surfarray.array3d(sim.screen),
                        (1, 0, 2))[:, :, ::-1]   # RGB -> BGR for imwrite
                    tmp_path = FRAME_PATH + '.tmp.png'
                    cv2.imwrite(tmp_path, frame)
                    os.replace(tmp_path, FRAME_PATH)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()


if __name__ == '__main__':
    main()
