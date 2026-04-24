#!/usr/bin/env python3
import csv
import io
import json
import os
import socket
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from flask import Flask, jsonify, request, send_file, Response

import depthdetect as depth_sensor
import actuator
import RPi.GPIO as GPIO
from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
                    BIAS_FILE, DATA_FILE, CONTROLLER_IP, CONTROLLER_PORT,
                    ACTUATOR_DUTY_CYCLE, CONTROL_DEADBAND_M,
                    TEST_SURFACE_DELAY_S, TEST_SURFACE_EXTEND_S)

app = Flask(__name__)

BASE_DIR  = os.path.dirname(os.path.abspath(__file__))
BIAS_PATH = os.path.join(BASE_DIR, BIAS_FILE)
DATA_PATH = os.path.join(BASE_DIR, DATA_FILE)

_mission_proc = None


# ── helpers ──────────────────────────────────────────────────────────────────

def _load_bias():
    if os.path.exists(BIAS_PATH):
        with open(BIAS_PATH) as f:
            return float(json.load(f).get('bias_m', 0.0))
    return 0.0


def _save_bias(value):
    with open(BIAS_PATH, 'w') as f:
        json.dump({'bias_m': value}, f)


def _mission_running():
    return _mission_proc is not None and _mission_proc.poll() is None


def _packet_count():
    if not os.path.exists(DATA_PATH):
        return 0
    with open(DATA_PATH) as f:
        return max(0, sum(1 for _ in f) - 1)  # minus header row


# ── UI ───────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    has_plot = os.path.exists(DATA_PATH) and _packet_count() >= 2
    plot_tag = (f'<img src="/plot?t={{t}}" id="plot" style="max-width:100%;border-radius:6px">'
                if has_plot else
                '<p style="color:#888">No plot yet — run a mission first.</p>')
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{CALL_SIGN} Float Control</title>
<style>
  body{{font-family:system-ui,sans-serif;margin:0;padding:1rem 1.5rem;background:#f4f6f8;color:#1a1a2e}}
  h1{{margin:0 0 1rem;font-size:1.4rem}}
  .grid{{display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-bottom:1rem}}
  .card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.1)}}
  .card h2{{margin:0 0 .6rem;font-size:.9rem;text-transform:uppercase;letter-spacing:.05em;color:#555}}
  .status-row{{display:flex;justify-content:space-between;margin:.25rem 0;font-size:.95rem}}
  .pill{{display:inline-block;padding:.15rem .6rem;border-radius:999px;font-size:.8rem;font-weight:600}}
  .running{{background:#d4edda;color:#155724}}
  .idle{{background:#e2e3e5;color:#383d41}}
  .online{{background:#d4edda;color:#155724}}
  .offline{{background:#f8d7da;color:#721c24}}
  .warn{{background:#fff3cd;color:#856404}}
  button{{cursor:pointer;border:none;border-radius:6px;padding:.5rem 1rem;font-size:.9rem;font-weight:600;margin:.25rem .25rem .25rem 0}}
  .btn-primary{{background:#0077b6;color:#fff}}
  .btn-warn{{background:#e07b00;color:#fff}}
  .btn-danger{{background:#c0392b;color:#fff}}
  .btn-neutral{{background:#555;color:#fff}}
  input[type=number]{{width:4rem;padding:.4rem;border:1px solid #ccc;border-radius:4px;font-size:.9rem}}
  #msg{{min-height:1.2rem;font-size:.85rem;color:#0077b6;margin:.5rem 0}}
  .plot-card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.1)}}
  a.dl{{font-size:.85rem;color:#0077b6}}
</style>
</head>
<body>
<h1>&#x1F4E1; {CALL_SIGN} Float Control &nbsp;<a href="/tuning" style="font-size:.8rem;font-weight:400;color:#0077b6">&#x1F9EA; Tuning</a></h1>

<div class="grid">
  <div class="card">
    <h2>Status</h2>
    <div class="status-row"><span>Network</span><span id="net-badge" class="pill idle">checking…</span>&nbsp;<span id="net-ip" style="font-size:.8rem;color:#888"></span></div>
    <div class="status-row"><span>Controller</span><span id="ctrl-badge" class="pill idle">checking…</span></div>
    <div class="status-row"><span>Mission</span><span id="mission-badge" class="pill idle">idle</span></div>
    <div class="status-row"><span>Bias</span><span id="bias-val">—</span></div>
    <div class="status-row"><span>Packets logged</span><span id="packets-val">—</span></div>
    <div class="status-row"><span>Depth</span><span id="depth-val">—</span></div>
    <div class="status-row"><span>Pressure</span><span id="pressure-val">—</span></div>
  </div>

  <div class="card">
    <h2>Actions</h2>
    <div id="msg"></div>
    <button class="btn-primary" onclick="api('POST','/calibrate')">&#x1F4CF; Calibrate bias</button>
    <button class="btn-primary" onclick="api('POST','/start')">&#x25B6; Start mission</button>
    <button class="btn-warn"    onclick="api('POST','/start?test=true')">&#x1F9EA; Test run</button>
    <br>
    <label>Extend <input type="number" id="ext-dur" value="10" min="1" max="60"> s</label>
    <button class="btn-neutral" onclick="api('POST','/extend?duration='+document.getElementById('ext-dur').value)">&#x2193; Extend</button>
    <br>
    <label>Retract <input type="number" id="ret-dur" value="10" min="1" max="60"> s</label>
    <button class="btn-neutral" onclick="api('POST','/retract?duration='+document.getElementById('ret-dur').value)">&#x2191; Retract</button>
    <br>
    <button class="btn-danger" onclick="api('POST','/stop')">&#x23F9; Stop actuator</button>
    <br><br>
    <a class="dl" href="/data" download>&#x2B73; Download CSV</a>
  </div>
</div>

<div class="plot-card">
  <h2 style="font-size:.9rem;text-transform:uppercase;letter-spacing:.05em;color:#555;margin:0 0 .75rem">Last Run Plot</h2>
  <div id="plot-container">{plot_tag}</div>
</div>

<script>
function msg(text, err) {{
  const el = document.getElementById('msg');
  el.textContent = text;
  el.style.color = err ? '#c0392b' : '#0077b6';
}}

function api(method, url) {{
  msg('...');
  fetch(url, {{method}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg(JSON.stringify(d));
      refreshPlot();
    }})
    .catch(e => msg(e.toString(), true));
}}

function refreshStatus() {{
  fetch('/status').then(r=>r.json()).then(d=>{{
    const badge = document.getElementById('mission-badge');
    badge.textContent = d.mission_running ? 'running' : 'idle';
    badge.className = 'pill ' + (d.mission_running ? 'running' : 'idle');
    document.getElementById('bias-val').textContent = d.bias_m.toFixed(4) + ' m';
    document.getElementById('packets-val').textContent = d.packets_logged;
  }});
  fetch('/depth').then(r=>r.json()).then(d=>{{
    if (d.depth_m !== undefined) {{
      document.getElementById('depth-val').textContent = d.depth_m.toFixed(3) + ' m';
      document.getElementById('pressure-val').textContent = d.pressure_kpa.toFixed(1) + ' kPa';
    }}
  }}).catch(()=>{{}});
}}

function refreshPlot() {{
  const img = document.getElementById('plot');
  if (img) {{
    img.src = '/plot?t=' + Date.now();
  }} else {{
    fetch('/status').then(r=>r.json()).then(d=>{{
      if (d.packets_logged >= 2) {{
        document.getElementById('plot-container').innerHTML =
          '<img src="/plot?t=' + Date.now() + '" id="plot" style="max-width:100%;border-radius:6px">';
      }}
    }});
  }}
}}

function refreshNetwork() {{
  fetch('/network').then(r=>r.json()).then(d=>{{
    const nb = document.getElementById('net-badge');
    nb.textContent = d.online ? 'online' : 'offline';
    nb.className   = 'pill ' + (d.online ? 'online' : 'offline');
    document.getElementById('net-ip').textContent = d.ip ? d.ip : '';

    const cb = document.getElementById('ctrl-badge');
    cb.textContent = d.controller_reachable ? 'reachable' : 'unreachable';
    cb.className   = 'pill ' + (d.controller_reachable ? 'online' : (d.online ? 'warn' : 'offline'));
  }}).catch(()=>{{}});
}}

refreshStatus();
refreshNetwork();
setInterval(refreshStatus, 5000);
setInterval(refreshNetwork, 10000);
setInterval(refreshPlot, 30000);
</script>
</body>
</html>"""
    return Response(html, mimetype='text/html')


# ── actuator endpoints ────────────────────────────────────────────────────────

@app.route('/extend', methods=['POST'])
def extend():
    if _mission_running():
        return jsonify({'error': 'Mission in progress'}), 409
    duration = float(request.args.get('duration', 20))
    actuator.extendActuator()
    time.sleep(duration)
    actuator.stopActuator()
    return jsonify({'status': 'done', 'duration': duration})


@app.route('/retract', methods=['POST'])
def retract():
    if _mission_running():
        return jsonify({'error': 'Mission in progress'}), 409
    duration = float(request.args.get('duration', 20))
    actuator.retractActuator()
    time.sleep(duration)
    actuator.stopActuator()
    return jsonify({'status': 'done', 'duration': duration})


@app.route('/stop', methods=['POST'])
def stop():
    actuator.stopActuator()
    return jsonify({'status': 'stopped'})


# ── sensor endpoints ──────────────────────────────────────────────────────────

@app.route('/depth')
def get_depth():
    depth_m, pressure_kpa = depth_sensor.readSensor()
    if depth_m is None:
        return jsonify({'error': 'Sensor read failed'}), 500
    bias = _load_bias()
    return jsonify({
        'raw_m':        round(depth_m, 4),
        'bias_m':       bias,
        'depth_m':      round(depth_m - bias, 4),
        'pressure_kpa': round(pressure_kpa, 1)
    })


# ── bias endpoints ────────────────────────────────────────────────────────────

@app.route('/bias', methods=['GET'])
def bias():
    # GET /bias          → return current bias
    # GET /bias?value=X  → set bias to X and return it
    if 'value' in request.args:
        v = float(request.args['value'])
        _save_bias(v)
        return jsonify({'bias_m': v})
    return jsonify({'bias_m': _load_bias()})


@app.route('/bias', methods=['POST'])
def set_bias_post():
    v = float(request.get_json().get('value', 0.0))
    _save_bias(v)
    return jsonify({'bias_m': v})


@app.route('/calibrate', methods=['POST'])
def calibrate():
    samples = []
    for _ in range(10):
        d, _ = depth_sensor.readSensor()
        if d is not None:
            samples.append(d)
        time.sleep(0.2)
    if not samples:
        return jsonify({'error': 'No sensor readings'}), 500
    bias = sum(samples) / len(samples)
    _save_bias(bias)
    return jsonify({'bias_m': round(bias, 4)})


# ── mission endpoints ─────────────────────────────────────────────────────────

@app.route('/start', methods=['POST'])
def start_mission():
    global _mission_proc
    if _mission_running():
        return jsonify({'error': 'Mission already running'}), 409
    a = request.args
    test = a.get('test', '').lower() in ('1', 'true', 'yes')
    script = os.path.join(BASE_DIR, 'depthadjust.py')
    cmd = [sys.executable, script]
    if test:                        cmd.append('--test')
    if 'duty'           in a:       cmd += ['--duty',           a['duty']]
    if 'deadband'       in a:       cmd += ['--deadband',       a['deadband']]
    if 'surface_delay'  in a:       cmd += ['--surface-delay',  a['surface_delay']]
    if 'surface_extend' in a:       cmd += ['--surface-extend', a['surface_extend']]
    _mission_proc = subprocess.Popen(cmd, cwd=BASE_DIR)
    return jsonify({'status': 'mission started', 'pid': _mission_proc.pid,
                    'test_mode': test, 'cmd': cmd[2:]})


@app.route('/status')
def status():
    return jsonify({
        'mission_running': _mission_running(),
        'bias_m':          _load_bias(),
        'packets_logged':  _packet_count()
    })


# ── data endpoints ────────────────────────────────────────────────────────────

@app.route('/data')
def get_data():
    if not os.path.exists(DATA_PATH):
        return jsonify({'error': 'No data yet'}), 404
    return send_file(DATA_PATH, mimetype='text/csv',
                     as_attachment=True, download_name='float_data.csv')


@app.route('/plot')
def get_plot():
    if not os.path.exists(DATA_PATH):
        return jsonify({'error': 'No data yet'}), 404

    times, depths = [], []
    with open(DATA_PATH) as f:
        for row in csv.DictReader(f):
            times.append(float(row['elapsed_s']))
            depths.append(float(row['depth_m']))

    if len(depths) < 2:
        return jsonify({'error': 'Not enough data points'}), 400

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(times, depths, 'b-o', markersize=4, linewidth=1.5)
    ax.invert_yaxis()
    ax.set_xlabel('Elapsed Time (s)')
    ax.set_ylabel('Depth (m)')
    ax.set_title(f'{CALL_SIGN} — Vertical Profile')
    ax.axhline(y=TARGET_BOTTOM_M, color='r', linestyle='--', alpha=0.6,
               label=f'Bottom target ({TARGET_BOTTOM_M} m)')
    ax.axhline(y=TARGET_SURFACE_M, color='g', linestyle='--', alpha=0.6,
               label=f'Surface target ({TARGET_SURFACE_M} m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    buf = io.BytesIO()
    fig.savefig(buf, format='jpeg', dpi=150)
    plt.close(fig)
    buf.seek(0)
    return send_file(buf, mimetype='image/jpeg')


# ── config / tuning ──────────────────────────────────────────────────────────

@app.route('/config')
def get_config():
    return jsonify({
        'duty_cycle':       ACTUATOR_DUTY_CYCLE,
        'deadband_m':       CONTROL_DEADBAND_M,
        'surface_delay_s':  TEST_SURFACE_DELAY_S,
        'surface_extend_s': TEST_SURFACE_EXTEND_S,
        'target_bottom_m':  TARGET_BOTTOM_M,
        'target_surface_m': TARGET_SURFACE_M,
        'tolerance_m':      TOLERANCE_M,
    })


@app.route('/tuning')
def tuning_page():
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{CALL_SIGN} — Tuning</title>
<style>
  body{{font-family:system-ui,sans-serif;margin:0;padding:1rem 1.5rem;background:#f4f6f8;color:#1a1a2e}}
  h1{{margin:0 0 .25rem;font-size:1.3rem}}
  .back{{font-size:.85rem;color:#0077b6;text-decoration:none}}
  .card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.1);margin:1rem 0}}
  h2{{margin:0 0 .75rem;font-size:.9rem;text-transform:uppercase;letter-spacing:.05em;color:#555}}
  .param{{margin:.6rem 0}}
  .param label{{display:block;font-size:.88rem;font-weight:600;margin-bottom:.2rem}}
  .param .desc{{font-size:.78rem;color:#666;margin-bottom:.3rem}}
  .row{{display:flex;align-items:center;gap:.6rem}}
  input[type=range]{{flex:1;accent-color:#0077b6}}
  input[type=number]{{width:5rem;padding:.35rem .5rem;border:1px solid #ccc;border-radius:4px;font-size:.9rem}}
  .val{{min-width:3.5rem;font-weight:600;font-family:monospace;font-size:.9rem}}
  .info-row{{display:flex;justify-content:space-between;font-size:.88rem;padding:.25rem 0;border-bottom:1px solid #f0f0f0}}
  .info-row:last-child{{border:none}}
  button{{cursor:pointer;border:none;border-radius:6px;padding:.55rem 1.1rem;font-size:.95rem;font-weight:700;margin:.3rem .3rem 0 0}}
  .btn-test{{background:#e07b00;color:#fff;font-size:1rem;padding:.6rem 1.4rem}}
  .btn-stop{{background:#c0392b;color:#fff}}
  .btn-neutral{{background:#555;color:#fff}}
  #msg{{min-height:1.1rem;font-size:.85rem;color:#0077b6;margin:.4rem 0}}
  .pill{{display:inline-block;padding:.15rem .55rem;border-radius:999px;font-size:.78rem;font-weight:700}}
  .running{{background:#cce5ff;color:#004085}}
  .idle{{background:#e2e3e5;color:#383d41}}
  #plot-img{{max-width:100%;border-radius:6px;margin-top:.5rem}}
</style>
</head>
<body>
<h1>&#x1F9EA; {CALL_SIGN} — Test &amp; Tuning</h1>
<a class="back" href="/">← Back to main UI</a>

<div class="card">
  <h2>Tunable Parameters</h2>

  <div class="param">
    <label>Actuator Duty Cycle</label>
    <div class="desc">Motor speed — lower = slower piston, less overshoot. Default: {ACTUATOR_DUTY_CYCLE}%</div>
    <div class="row">
      <input type="range" id="duty" min="20" max="100" step="5" value="{ACTUATOR_DUTY_CYCLE}"
             oninput="document.getElementById('duty-val').textContent=this.value+'%'">
      <span class="val" id="duty-val">{ACTUATOR_DUTY_CYCLE}%</span>
    </div>
  </div>

  <div class="param">
    <label>Control Deadband (m)</label>
    <div class="desc">Stop zone around target — wider = less hunting, more depth variation. Default: {CONTROL_DEADBAND_M} m</div>
    <div class="row">
      <input type="range" id="deadband" min="0.01" max="0.15" step="0.01" value="{CONTROL_DEADBAND_M}"
             oninput="document.getElementById('db-val').textContent=parseFloat(this.value).toFixed(2)+' m'">
      <span class="val" id="db-val">{CONTROL_DEADBAND_M} m</span>
    </div>
  </div>

  <div class="param">
    <label>Surface Delay (s)</label>
    <div class="desc">Seconds to wait at end of profiles before surfacing in test mode. Default: {TEST_SURFACE_DELAY_S} s</div>
    <div class="row">
      <input type="number" id="surface-delay" min="10" max="300" value="{TEST_SURFACE_DELAY_S}"> s
    </div>
  </div>

  <div class="param">
    <label>Surface Extend (s)</label>
    <div class="desc">Seconds to run extend motor when surfacing. Default: {TEST_SURFACE_EXTEND_S} s</div>
    <div class="row">
      <input type="number" id="surface-extend" min="10" max="60" value="{TEST_SURFACE_EXTEND_S}"> s
    </div>
  </div>
</div>

<div class="card">
  <h2>Fixed Competition Parameters</h2>
  <div class="info-row"><span>Bottom target</span><span>{TARGET_BOTTOM_M} m  (valid {TARGET_BOTTOM_M-TOLERANCE_M:.2f}–{TARGET_BOTTOM_M+TOLERANCE_M:.2f} m)</span></div>
  <div class="info-row"><span>Surface target</span><span>{TARGET_SURFACE_M} m  (valid {TARGET_SURFACE_M-TOLERANCE_M:.2f}–{TARGET_SURFACE_M+TOLERANCE_M:.2f} m)</span></div>
  <div class="info-row"><span>Hold time</span><span>30 s · 7 packets · 5 s interval</span></div>
  <div class="info-row"><span>Profiles</span><span>2</span></div>
</div>

<div class="card">
  <h2>Run</h2>
  <div id="msg"></div>
  <div style="margin-bottom:.5rem">Mission: <span id="mission-badge" class="pill idle">idle</span> &nbsp; Packets: <span id="packets">—</span></div>
  <button class="btn-test" onclick="startTest()">&#x1F9EA; Start Test Run</button>
  <button class="btn-stop" onclick="stopRun()">&#x23F9; Stop</button>
  <button class="btn-neutral" onclick="location.href='/plot'" style="float:right">&#x1F4C8; View plot</button>
</div>

<div id="plot-card" style="display:none" class="card">
  <h2>Last Run Plot</h2>
  <img id="plot-img" src="" alt="plot">
</div>

<script>
let _polling = null;
let _lastPackets = 0;

function msg(text, err) {{
  const el = document.getElementById('msg');
  el.textContent = text;
  el.style.color = err ? '#c0392b' : '#0077b6';
}}

function buildUrl() {{
  const duty    = document.getElementById('duty').value;
  const db      = document.getElementById('deadband').value;
  const delay   = document.getElementById('surface-delay').value;
  const extend  = document.getElementById('surface-extend').value;
  return `/start?test=true&duty=${{duty}}&deadband=${{db}}&surface_delay=${{delay}}&surface_extend=${{extend}}`;
}}

function startTest() {{
  msg('Starting test run…');
  fetch(buildUrl(), {{method:'POST'}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg('Running: ' + JSON.stringify(d.cmd));
      startPolling();
    }})
    .catch(e => msg(e.toString(), true));
}}

function stopRun() {{
  fetch('/stop', {{method:'POST'}}).then(() => msg('Stopped.'));
}}

function startPolling() {{
  if (_polling) clearInterval(_polling);
  _polling = setInterval(pollStatus, 3000);
  pollStatus();
}}

function pollStatus() {{
  fetch('/status').then(r=>r.json()).then(d=>{{
    const b = document.getElementById('mission-badge');
    b.textContent = d.mission_running ? 'running' : 'idle';
    b.className = 'pill ' + (d.mission_running ? 'running' : 'idle');
    document.getElementById('packets').textContent = d.packets_logged;
    if (!d.mission_running && _lastPackets !== d.packets_logged) {{
      _lastPackets = d.packets_logged;
      if (d.packets_logged >= 2) showPlot();
      if (_polling) {{ clearInterval(_polling); _polling = null; }}
    }}
  }});
}}

function showPlot() {{
  const card = document.getElementById('plot-card');
  card.style.display = '';
  document.getElementById('plot-img').src = '/plot?t=' + Date.now();
}}

pollStatus();
setInterval(pollStatus, 5000);
</script>
</body>
</html>"""
    return Response(html, mimetype='text/html')


# ── network status ───────────────────────────────────────────────────────────

@app.route('/network')
def network():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('192.168.3.1', 1))
        ip = s.getsockname()[0]
        s.close()
    except Exception:
        ip = None

    controller_ok = False
    try:
        import requests
        r = requests.get(f"http://{CONTROLLER_IP}:{CONTROLLER_PORT}/events",
                         timeout=2)
        controller_ok = r.status_code == 200
    except Exception:
        pass

    return jsonify({
        'ip':                 ip,
        'online':             ip is not None,
        'controller_ip':      CONTROLLER_IP,
        'controller_port':    CONTROLLER_PORT,
        'controller_reachable': controller_ok,
    })


# ── startup ───────────────────────────────────────────────────────────────────

GPIO.setwarnings(False)
actuator.setupActuator()
depth_sensor.initSensor()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
