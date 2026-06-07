#!/usr/bin/env python3
"""Minimal PDDL patrol web UI.

Serves a single-page plan viewer on http://<host>:8080.
State is fed via JSON published on /pddl_patrol/ui_state.
"""

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Shared mutable state updated by the ROS subscriber
_ui_state = {
    "plan": [],
    "replanning": False,
    "current_wp": "",
    "visited": [],
}


class UIStateSubscriber(Node):
    def __init__(self):
        super().__init__('pddl_ui_server')
        self.create_subscription(
            String, '/pddl_patrol/ui_state', self._callback, 10)
        self.get_logger().info('UI state subscriber ready')

    def _callback(self, msg: String):
        global _ui_state
        try:
            data = json.loads(msg.data)
            _ui_state.update(data)
        except json.JSONDecodeError:
            pass


class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silence logs

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body.encode())

    def do_GET(self):
        if self.path == '/status':
            global _ui_state
            self._send(200, 'application/json', json.dumps(_ui_state))
            return

        # Serve the single-page HTML
        html = """<!DOCTYPE html>
<html lang=\"en\">
<head>
<meta charset=\"UTF-8\">
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">
<title>PDDL Patrol</title>
<style>
  body {
    font-family: system-ui, -apple-system, sans-serif;
    background: #f5f5f5;
    margin: 0;
    padding: 1rem;
    display: flex;
    justify-content: center;
  }
  .container {
    width: 100%;
    max-width: 480px;
  }
  h1 {
    font-size: 1.2rem;
    margin: 0 0 0.8rem;
    color: #333;
  }
  .replanning {
    display: none;
    position: fixed;
    top: 0; left: 0; right: 0; bottom: 0;
    background: rgba(245,245,245,0.92);
    align-items: center;
    justify-content: center;
    flex-direction: column;
    z-index: 100;
  }
  .replanning.active {
    display: flex;
  }
  .spinner {
    border: 4px solid #e0e0e0;
    border-top: 4px solid #4caf50;
    border-right: 4px solid #4caf50;
    border-radius: 50%;
    width: 48px;
    height: 48px;
    animation: spin 1s linear infinite;
    margin-bottom: 1rem;
  }
  @keyframes spin {
    0% { transform: rotate(0deg); }
    100% { transform: rotate(360deg); }
  }
  .thinking-text {
    font-size: 1.1rem;
    color: #555;
    font-weight: 600;
    letter-spacing: 0.05em;
  }
  .dots::after {
    content: '';
    animation: dots 1.5s steps(5, end) infinite;
  }
  @keyframes dots {
    0%   { content: ''; }
    25%  { content: '.'; }
    50%  { content: '..'; }
    75%  { content: '...'; }
    100% { content: ''; }
  }
  .step {
    display: flex;
    align-items: center;
    gap: 0.6rem;
    padding: 0.7rem 0.9rem;
    border-radius: 6px;
    margin-bottom: 0.5rem;
    background: #fff;
    box-shadow: 0 1px 2px rgba(0,0,0,0.05);
    transition: background 0.3s;
  }
  .step .box {
    width: 18px;
    height: 18px;
    border-radius: 4px;
    flex-shrink: 0;
    border: 2px solid #ddd;
  }
  .step.pending { opacity: 0.8; }
  .step.pending .box { background: #fff; border-color: #ccc; }
  .step.active { background: #e8f5e9; }
  .step.active .box { background: #4caf50; border-color: #4caf50; }
  .step.done { background: #eeeeee; opacity: 0.6; }
  .step.done .box { background: #9e9e9e; border-color: #9e9e9e; }
  .step .label {
    font-size: 0.95rem;
    color: #333;
  }
  .step.done .label { text-decoration: line-through; }
  .footer {
    font-size: 0.75rem;
    color: #888;
    text-align: center;
    margin-top: 0.8rem;
  }
</style>
</head>
<body>
<div class=\"container\">
  <h1>PDDL Patrol Plan</h1>
  <div id="replanning" class="replanning">
    <div class="spinner"></div>
    <div class="thinking-text">Replanning<span class="dots"></span></div>
  </div>
  <div id=\"plan\"></div>
  <div class=\"footer\">http://localhost:8080</div>
</div>
<script>
  async function fetchStatus() {
    try {
      const res = await fetch('/status');
      return await res.json();
    } catch (e) { return null; }
  }

  function render(state) {
    const planEl = document.getElementById('plan');
    const replanEl = document.getElementById('replanning');
    if (!state) return;

    if (state.replanning) {
      replanEl.classList.add('active');
    } else {
      replanEl.classList.remove('active');
    }

    planEl.innerHTML = '';
    if (!state.plan || state.plan.length === 0) {
      planEl.innerHTML = '<div style=\"text-align:center;color:#888;padding:1rem;\">No plan yet</div>';
      return;
    }
    for (const step of state.plan) {
      const div = document.createElement('div');
      div.className = 'step ' + (step.status || 'pending');
      div.innerHTML = '<div class=\"box\"></div><div class=\"label\">' + escapeHtml(step.action || '') + '</div>';
      planEl.appendChild(div);
    }
  }

  function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }

  async function tick() {
    const state = await fetchStatus();
    render(state);
  }

  setInterval(tick, 500);
  tick();
</script>
</body>
</html>"""
        self._send(200, 'text/html', html)


def run_http_server(port=8080):
    srv = ReusableHTTPServer(('', port), Handler)
    srv.serve_forever()


def main():
    rclpy.init()
    node = UIStateSubscriber()

    # Start HTTP server in a background thread
    t = threading.Thread(target=run_http_server, args=(8080,), daemon=True)
    t.start()
    node.get_logger().info('PDDL UI server running at http://0.0.0.0:8080')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
