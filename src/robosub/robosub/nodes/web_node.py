#!/usr/bin/env python3
"""
Web control node — serves a browser UI for controlling the sub and
playing back recorded videos.

Endpoints:
  GET  /                     Control page
  GET  /api/status           JSON: {"status": "TaskName|state"}
  GET  /api/control/<cmd>    Publish cmd to /sim/control (start/pause/resume/reset/quit)
  GET  /api/videos           JSON: sorted list of MP4 filenames (newest first)
  GET    /api/video/<filename>  Stream an MP4 file (supports range requests for seeking)
  DELETE /api/video/<filename>  Delete a recording

Topics published:
  /sim/control   std_msgs/String

Topics subscribed:
  /sub/status    std_msgs/String
"""
import os
import threading

from flask import Flask, jsonify, request, send_from_directory, render_template_string

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RECORDINGS_DIR = os.path.expanduser('~/robosub_recordings')

# ---------------------------------------------------------------------------
# HTML page
# ---------------------------------------------------------------------------

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RoboSub Control</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: 'Courier New', monospace; background: #0d1117; color: #c9d1d9; padding: 20px; }
  h1 { color: #58a6ff; margin-bottom: 16px; font-size: 1.4em; letter-spacing: 2px; }
  h2 { color: #8b949e; font-size: 0.95em; margin: 16px 0 8px; text-transform: uppercase; letter-spacing: 1px; }
  .card { background: #161b22; border: 1px solid #30363d; border-radius: 6px; padding: 12px 16px; margin-bottom: 12px; }
  #status { color: #3fb950; font-size: 0.95em; }
  #status.waiting { color: #d29922; }
  .buttons { display: flex; flex-wrap: wrap; gap: 8px; }
  button {
    padding: 8px 18px; font-family: inherit; font-size: 0.88em;
    border: none; border-radius: 4px; cursor: pointer; font-weight: bold;
    transition: opacity 0.15s;
  }
  button:hover { opacity: 0.85; }
  .btn-start  { background: #238636; color: #fff; }
  .btn-pause  { background: #9e6a03; color: #fff; }
  .btn-resume { background: #1f6feb; color: #fff; }
  .btn-reset  { background: #6e40c9; color: #fff; }
  .btn-stop   { background: #da3633; color: #fff; }
  .video-list { max-height: 200px; overflow-y: auto; }
  .video-item {
    padding: 6px 10px; margin-bottom: 3px; cursor: pointer;
    background: #0d1117; border: 1px solid #21262d; border-radius: 4px;
    font-size: 0.85em; color: #58a6ff;
  }
  .video-item:hover { background: #161b22; border-color: #58a6ff; }
  .video-item.active { border-color: #3fb950; color: #3fb950; }
  #now-playing { font-size: 0.8em; color: #8b949e; margin: 6px 0; min-height: 1em; }
  video { width: 100%; max-width: 860px; border-radius: 4px; margin-top: 4px;
          background: #000; border: 1px solid #21262d; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
         background: #3fb950; margin-right: 8px; vertical-align: middle; }
  .dot.waiting { background: #d29922; }
</style>
</head>
<body>
<h1>&#x1F916; RoboSub Control</h1>

<div class="card">
  <h2>&#x2022; Status</h2>
  <p style="margin-top:6px"><span class="dot" id="dot"></span><span id="status">Connecting...</span></p>
</div>

<div class="card">
  <h2>&#x2022; Mission Control</h2>
  <div class="buttons" style="margin-top:8px">
    <button class="btn-start"  onclick="ctrl('start')">&#x25B6; Start</button>
    <button class="btn-pause"  onclick="ctrl('pause')">&#x23F8; Pause</button>
    <button class="btn-resume" onclick="ctrl('resume')">&#x25B6; Resume</button>
    <button class="btn-reset"  onclick="ctrl('reset')">&#x21BA; Reset</button>
    <button class="btn-stop"   onclick="ctrl('quit')">&#x25A0; Stop</button>
  </div>
</div>

<div class="card">
  <h2>&#x2022; Recordings</h2>
  <div class="video-list" id="video-list"><em style="color:#8b949e;font-size:0.85em">Loading...</em></div>
  <p id="now-playing"></p>
  <video id="player" controls></video>
</div>

<script>
  let activeItem = null;

  function ctrl(cmd) {
    fetch('/api/control/' + cmd)
      .catch(e => console.warn('ctrl error', e));
  }

  function pollStatus() {
    fetch('/api/status')
      .then(r => r.json())
      .then(d => {
        const el = document.getElementById('status');
        const dot = document.getElementById('dot');
        const s = d.status.replace('|', ' \u2014 ');
        el.textContent = s;
        const waiting = s.startsWith('WAITING');
        el.className = waiting ? 'waiting' : '';
        dot.className = 'dot' + (waiting ? ' waiting' : '');
      })
      .catch(() => {
        document.getElementById('status').textContent = 'Disconnected';
      });
  }

  function loadVideos() {
    fetch('/api/videos')
      .then(r => r.json())
      .then(files => {
        const div = document.getElementById('video-list');
        if (files.length === 0) {
          div.innerHTML = '<em style="color:#8b949e;font-size:0.85em">No recordings yet.</em>';
          return;
        }
        div.innerHTML = '';
        files.forEach(f => {
          const item = document.createElement('div');
          item.className = 'video-item';
          item.textContent = f;
          item.dataset.file = f;
          item.onclick = () => {
            if (activeItem) activeItem.classList.remove('active');
            activeItem = item;
            item.classList.add('active');
            const player = document.getElementById('player');
            player.src = '/api/video/' + f;
            document.getElementById('now-playing').textContent = f;
            player.play();
          };

          const del = document.createElement('span');
          del.textContent = ' \u2715';
          del.style.cssText = 'color:#da3633;float:right;font-weight:bold;padding:0 4px;';
          del.title = 'Delete';
          del.onclick = (e) => {
            e.stopPropagation();
            if (!confirm('Delete ' + f + '?')) return;
            fetch('/api/video/' + f, {method: 'DELETE'})
              .then(() => { if (activeItem === item) {
                document.getElementById('player').src = '';
                document.getElementById('now-playing').textContent = '';
                activeItem = null;
              }
              loadVideos(); });
          };
          item.appendChild(del);
          div.appendChild(item);
        });
      });
  }

  setInterval(pollStatus, 1000);
  setInterval(loadVideos, 8000);
  pollStatus();
  loadVideos();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class WebNode(Node):

    def __init__(self):
        super().__init__('web_node')

        self.declare_parameter('port', 8080)
        self._port  = self.get_parameter('port').get_parameter_value().integer_value

        self._status = 'WAITING|Press Start to begin'

        self._ctrl_pub = self.create_publisher(String, '/sim/control', 10)
        self.create_subscription(String, '/sub/status', self._status_cb, 10)

        self._app = Flask(__name__)
        self._setup_routes()

        t = threading.Thread(target=self._run_flask, daemon=True)
        t.start()
        self.get_logger().info(f'Web UI available at http://localhost:{self._port}')

    def _status_cb(self, msg: String):
        self._status = msg.data

    def _publish(self, cmd: str):
        self._ctrl_pub.publish(String(data=cmd))

    def _setup_routes(self):
        app  = self._app
        node = self

        @app.route('/')
        def index():
            return render_template_string(HTML)

        @app.route('/api/status')
        def status():
            return jsonify({'status': node._status})

        @app.route('/api/control/<cmd>')
        def control(cmd):
            if cmd in ('start', 'pause', 'resume', 'reset', 'quit'):
                node._publish(cmd)
                return jsonify({'ok': True, 'cmd': cmd})
            return jsonify({'ok': False, 'error': 'unknown command'}), 400

        @app.route('/api/videos')
        def videos():
            d = RECORDINGS_DIR
            if not os.path.isdir(d):
                return jsonify([])
            files = sorted(
                (f for f in os.listdir(d) if f.endswith('.mp4')),
                reverse=True
            )
            return jsonify(files)

        @app.route('/api/video/<path:filename>', methods=['GET', 'DELETE'])
        def video(filename):
            if request.method == 'DELETE':
                path = os.path.join(RECORDINGS_DIR, os.path.basename(filename))
                if os.path.isfile(path):
                    os.remove(path)
                    return jsonify({'ok': True})
                return jsonify({'ok': False, 'error': 'not found'}), 404
            # send_from_directory handles Range requests so HTML5 seeking works
            return send_from_directory(RECORDINGS_DIR, filename,
                                       conditional=True)

    def _run_flask(self):
        import logging
        log = logging.getLogger('werkzeug')
        log.setLevel(logging.WARNING)   # suppress per-request noise
        self._app.run(host='0.0.0.0', port=self._port, debug=False,
                      threaded=True, use_reloader=False)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WebNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
