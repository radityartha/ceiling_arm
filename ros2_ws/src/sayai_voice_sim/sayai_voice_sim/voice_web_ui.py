#!/usr/bin/env python3

import json
import os
import ssl
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def _ensure_cert(cert_dir: str):
    """Generate a self-signed cert in cert_dir if one doesn't exist."""
    os.makedirs(cert_dir, exist_ok=True)
    cert = os.path.join(cert_dir, "cert.pem")
    key  = os.path.join(cert_dir, "key.pem")
    if not os.path.exists(cert) or not os.path.exists(key):
        subprocess.run(
            [
                "openssl", "req", "-x509", "-newkey", "rsa:2048",
                "-keyout", key, "-out", cert,
                "-days", "3650", "-nodes",
                "-subj", "/CN=workcell-voice-ui",
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    return cert, key


HTML_PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Workcell Voice Control</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: system-ui, sans-serif; margin: 0; background: #101418; color: #edf2f7; }
    main { max-width: 600px; margin: 0 auto; padding: 24px; }
    h1 { font-size: 24px; margin: 0 0 20px; }
    section { border: 1px solid #2d3748; border-radius: 10px; padding: 16px; margin: 14px 0; background: #171d24; }
    h2 { font-size: 13px; text-transform: uppercase; letter-spacing: 1px; color: #94a3b8; margin: 0 0 12px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    button {
      font-size: 16px; font-weight: 600; padding: 18px 10px;
      border: 0; border-radius: 8px; color: #fff; cursor: pointer;
      background: #2563eb; width: 100%; transition: opacity .15s;
    }
    button:active { opacity: .75; }
    button.stop { background: #dc2626; grid-column: 1 / -1; font-size: 18px; padding: 20px; }
    .status-row { display: flex; gap: 8px; align-items: baseline; margin: 6px 0; }
    .label { font-size: 12px; color: #64748b; min-width: 80px; }
    .value { font-family: monospace; font-size: 14px; color: #a7f3d0; }
    .busy { color: #fbbf24; }
    input {
      width: 100%; font-size: 16px; padding: 12px;
      border-radius: 8px; border: 1px solid #334155;
      background: #0f172a; color: #fff; margin-top: 4px;
    }
    #micBtn {
      width: 100%; padding: 16px; font-size: 16px; font-weight: 600;
      border: 2px solid #334155; border-radius: 8px;
      background: #0f172a; color: #94a3b8; cursor: pointer;
      transition: all .2s;
    }
    #micBtn.listening {
      background: #1a2a1a; border-color: #22c55e; color: #22c55e;
      animation: pulse 1s infinite;
    }
    #micBtn.unsupported { opacity: .4; cursor: not-allowed; }
    #micResult { font-family: monospace; font-size: 13px; color: #a7f3d0; margin-top: 8px; min-height: 18px; }
    @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:.6; } }
  </style>
</head>
<body>
<main>
  <h1>Workcell Control</h1>

  <section>
    <h2>Status</h2>
    <div class="status-row"><span class="label">Transcript</span><span class="value" id="stTranscript">—</span></div>
    <div class="status-row"><span class="label">Action</span><span class="value" id="stAction">—</span></div>
    <div class="status-row"><span class="label">Active task</span><span class="value busy" id="stBusy">—</span></div>
  </section>

  <section>
    <h2>Tasks</h2>
    <div class="grid">
      <button onclick="task('open_curtain')">Open Curtain</button>
      <button onclick="task('close_curtain')">Close Curtain</button>
      <button onclick="task('bring_bag')">Bring Bag</button>
      <button onclick="task('bring_bottle')">Bring Bottle</button>
      <button onclick="task('unitree_collab')" style="grid-column:1/-1;background:#7c3aed;">Unitree Collab Handoff</button>
      <button class="stop" onclick="task('stop')">STOP</button>
    </div>
  </section>

  <section>
    <h2>Speak from browser</h2>
    <button id="micBtn" onclick="toggleMic()">🎙 Hold to speak</button>
    <div id="micResult"></div>
  </section>

  <section>
    <h2>Type command</h2>
    <form onsubmit="sendText(event)">
      <input id="textCmd" placeholder="e.g. open  /  bag  /  stop">
    </form>
  </section>
</main>
<script>
  // --- mic (Web Speech API — works in Chrome/Chromium) ---
  let recog = null;
  const micBtn = document.getElementById('micBtn');
  const micResult = document.getElementById('micResult');

  if ('webkitSpeechRecognition' in window || 'SpeechRecognition' in window) {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    recog = new SR();
    recog.lang = 'en-US';
    recog.interimResults = false;
    recog.maxAlternatives = 1;

    recog.onresult = (e) => {
      const text = e.results[0][0].transcript;
      micResult.textContent = '"' + text + '"';
      sendRawText(text);
    };
    recog.onend = () => {
      micBtn.classList.remove('listening');
      micBtn.textContent = '🎙 Hold to speak';
    };
    recog.onerror = (e) => {
      micResult.textContent = 'Error: ' + e.error;
      micBtn.classList.remove('listening');
      micBtn.textContent = '🎙 Hold to speak';
    };
  } else {
    micBtn.classList.add('unsupported');
    micBtn.textContent = '🎙 Not supported (use Chrome)';
  }

  function toggleMic() {
    if (!recog) return;
    if (micBtn.classList.contains('listening')) {
      recog.stop();
    } else {
      micResult.textContent = '';
      micBtn.classList.add('listening');
      micBtn.textContent = '🔴 Listening…';
      recog.start();
    }
  }

  async function sendRawText(text) {
    await fetch('/api/transcript', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({text}),
    });
    refresh();
  }

  // --- status polling ---
  async function refresh() {
    try {
      const data = await fetch('/api/status').then(r => r.json());
      document.getElementById('stTranscript').textContent = data.last_transcript || '—';
      document.getElementById('stAction').textContent    = data.last_action    || '—';
      document.getElementById('stBusy').textContent      = data.active_task    || '—';
    } catch (_) {}
  }
  async function task(name) {
    await fetch('/api/task/' + name, {method: 'POST'});
    refresh();
  }
  async function sendText(e) {
    e.preventDefault();
    const inp = document.getElementById('textCmd');
    await fetch('/api/transcript', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({text: inp.value}),
    });
    inp.value = '';
    refresh();
  }
  setInterval(refresh, 1000);
  refresh();
</script>
</body>
</html>
"""


class VoiceWebUi(Node):
    """Small local browser UI for voice-command monitoring and manual override."""

    def __init__(self):
        super().__init__("voice_web_ui")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("use_ssl", True)

        self._host = self.get_parameter("host").value
        self._port = int(self.get_parameter("port").value)
        self._use_ssl = self.get_parameter("use_ssl").value
        self._last_transcript = ""
        self._last_action = ""

        self._transcript_pub = self.create_publisher(String, "/voice/transcript", 10)
        self._transcript_sub = self.create_subscription(
            String,
            "/voice/transcript",
            self._transcript_cb,
            10,
        )
        self._task_clients = {
            "open_curtain":  self.create_client(Trigger, "/task/open_curtain"),
            "close_curtain": self.create_client(Trigger, "/task/close_curtain"),
            "bring_bag":     self.create_client(Trigger, "/task/bring_bag"),
            "bring_bottle":   self.create_client(Trigger, "/task/bring_bottle"),
            "unitree_collab": self.create_client(Trigger, "/task/unitree_collab"),
            "stop":           self.create_client(Trigger, "/task/stop"),
        }
        self._active_task = ""

        handler = self._make_handler()
        self._server = ThreadingHTTPServer((self._host, self._port), handler)

        if self._use_ssl:
            cert_dir = os.path.join(os.path.expanduser("~"), ".ros", "voice_web_ui_cert")
            cert, key = _ensure_cert(cert_dir)
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(cert, key)
            self._server.socket = ctx.wrap_socket(self._server.socket, server_side=True)
            scheme = "https"
        else:
            scheme = "http"

        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        self.get_logger().info(
            "Voice web UI ready: %s://%s:%d  (phone: %s://$(hostname -I | awk '{print $1}'):%d)"
            % (scheme, self._host, self._port, scheme, self._port)
        )

    def _transcript_cb(self, msg: String):
        self._last_transcript = msg.data

    def _publish_transcript(self, text: str):
        msg = String()
        msg.data = text
        self._transcript_pub.publish(msg)
        self._last_action = "published transcript: %s" % text

    def _call_task(self, task_name: str):
        client = self._task_clients.get(task_name)
        if client is None:
            self._last_action = "unknown task: %s" % task_name
            return
        if not client.wait_for_service(timeout_sec=0.5):
            self._last_action = "service unavailable: %s" % task_name
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f: self._on_task_response(task_name, f))
        self._last_action = "called: %s" % task_name

    def _on_task_response(self, task_name: str, future):
        try:
            result = future.result()
        except Exception:
            self._active_task = ""
            return
        if result.success:
            self._active_task = task_name if task_name != "stop" else ""
        else:
            self._last_action = result.message
            self._active_task = ""

    def _status(self):
        return {
            "last_transcript": self._last_transcript,
            "last_action": self._last_action,
            "active_task": self._active_task,
        }

    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    self._send(200, HTML_PAGE, "text/html; charset=utf-8")
                elif self.path == "/api/status":
                    self._send_json(200, node._status())
                else:
                    self._send_json(404, {"error": "not found"})

            def do_POST(self):
                if self.path.startswith("/api/task/"):
                    task_name = self.path.rsplit("/", 1)[-1]
                    node._call_task(task_name)
                    self._send_json(200, node._status())
                    return
                if self.path == "/api/transcript":
                    length = int(self.headers.get("Content-Length", "0"))
                    body = self.rfile.read(length).decode("utf-8") if length else "{}"
                    try:
                        payload = json.loads(body)
                    except json.JSONDecodeError:
                        self._send_json(400, {"error": "invalid json"})
                        return
                    node._publish_transcript(str(payload.get("text", "")))
                    self._send_json(200, node._status())
                    return
                self._send_json(404, {"error": "not found"})

            def log_message(self, fmt, *args):
                return

            def _send_json(self, status, payload):
                self._send(status, json.dumps(payload), "application/json")

            def _send(self, status, body, content_type):
                encoded = body.encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

        return Handler

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceWebUi()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
