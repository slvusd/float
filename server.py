#!/usr/bin/env python3
import csv
import glob
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
try:
    import RPi.GPIO as GPIO
    _HAS_GPIO = True
except ImportError:
    GPIO = None
    _HAS_GPIO = False

from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
                    BIAS_FILE, DATA_FILE, CONTROLLER_IP, CONTROLLER_PORT,
                    ACTUATOR_DUTY_CYCLE, CONTROL_DEADBAND_M,
                    TEST_SURFACE_DELAY_S, TEST_SURFACE_EXTEND_S,
                    SENSOR_DEPTH_OFFSET_M, FLOAT_HEIGHT_M, APPROACH_ZONE_M, MIN_DUTY_PCT)

app = Flask(__name__)

BASE_DIR     = os.path.dirname(os.path.abspath(__file__))
BIAS_PATH    = os.path.join(BASE_DIR, BIAS_FILE)
DATA_PATH    = os.path.join(BASE_DIR, DATA_FILE)
STATUS_PATH  = os.path.join(BASE_DIR, 'mission_status.json')

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


def _mission_stage():
    if os.path.exists(STATUS_PATH):
        try:
            with open(STATUS_PATH) as f:
                return json.load(f)
        except Exception:
            pass
    return {'stage': 'idle', 'time': ''}


def _packet_count():
    if not os.path.exists(DATA_PATH):
        return 0
    with open(DATA_PATH) as f:
        return max(0, sum(1 for _ in f) - 1)  # minus header row


def _nav(active=''):
    pages = [('/', 'Home'), ('/tuning', 'Tuning'), ('/runs', 'Runs'), ('/log', 'Log')]
    items = ''.join(
        '<a href="{}" style="text-decoration:none;margin-right:1.4rem;font-size:.88rem;'
        'color:{};{}">{}</a>'.format(
            href,
            '#fff' if href == active else '#99b',
            'font-weight:700;border-bottom:2px solid #4db8ff;padding-bottom:2px'
            if href == active else '',
            label
        ) for href, label in pages
    )
    return ('<nav style="background:#1a1a2e;padding:.55rem 1.5rem;'
            'margin:-1rem -1.5rem 1.2rem">' + items + '</nav>')


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
{_nav('/')}
<h1 style="margin-top:0">&#x1F4E1; {CALL_SIGN} Float</h1>

<div class="grid">
  <div class="card">
    <h2>Status</h2>
    <div class="status-row"><span>Network</span><span id="net-badge" class="pill idle">checking…</span>&nbsp;<span id="net-ip" style="font-size:.8rem;color:#888"></span></div>
    <div class="status-row"><span>Controller</span><span id="ctrl-badge" class="pill idle">checking…</span></div>
    <div class="status-row"><span>Mission</span><span id="mission-badge" class="pill idle">idle</span></div>
    <div class="status-row"><span>Stage</span><span id="stage-val" style="font-size:.82rem;color:#555">—</span></div>
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
    <button class="btn-danger" onclick="api('POST','/abort')" style="background:#7b0000">&#x1F6D1; Abort mission</button>
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
    const label = d.profiles_complete ? 'transmitting' : (d.mission_running ? 'running' : 'idle');
    const cls   = d.profiles_complete ? 'warn' : (d.mission_running ? 'running' : 'idle');
    badge.textContent = label;
    badge.className   = 'pill ' + cls;
    document.getElementById('stage-val').textContent =
      d.stage ? (d.stage.replace(/_/g,' ') + (d.stage_time ? '  ' + d.stage_time : '')) : '—';
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


# ── actuator GPIO management ──────────────────────────────────────────────────
# GPIO is held between requests (efficient) but released cleanly before the
# mission subprocess starts so depthadjust.py can claim the pins without conflict.

_gpio_ready = False


def _ensure_gpio():
    global _gpio_ready
    if not _HAS_GPIO:
        return
    if not _gpio_ready:
        GPIO.setwarnings(False)
        actuator.setupActuator()
        _gpio_ready = True


def _release_gpio():
    global _gpio_ready
    if not _HAS_GPIO:
        return
    if _gpio_ready:
        actuator.cleanupActuator()
        _gpio_ready = False


# ── actuator endpoints ────────────────────────────────────────────────────────

@app.route('/extend', methods=['POST'])
def extend():
    if _mission_running():
        return jsonify({'error': 'Mission in progress'}), 409
    duration = float(request.args.get('duration', 20))
    _ensure_gpio()
    actuator.extendActuator()
    time.sleep(duration)
    actuator.stopActuator()
    return jsonify({'status': 'done', 'duration': duration})


@app.route('/retract', methods=['POST'])
def retract():
    if _mission_running():
        return jsonify({'error': 'Mission in progress'}), 409
    duration = float(request.args.get('duration', 20))
    _ensure_gpio()
    actuator.retractActuator()
    time.sleep(duration)
    actuator.stopActuator()
    return jsonify({'status': 'done', 'duration': duration})


@app.route('/stop', methods=['POST'])
def stop():
    if _gpio_ready:
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
    if 'surface_extend'  in a:      cmd += ['--surface-extend',  a['surface_extend']]
    if 'target_bottom'   in a:      cmd += ['--target-bottom',   a['target_bottom']]
    if 'target_surface'  in a:      cmd += ['--target-surface',  a['target_surface']]
    if 'sensor_offset'   in a:      cmd += ['--sensor-offset',   a['sensor_offset']]
    if a.get('sim', '').lower() in ('1', 'true', 'yes'):
                                    cmd.append('--sim')
    if 'sim_rate'        in a:      cmd += ['--sim-rate',        a['sim_rate']]
    if 'approach_zone'   in a:      cmd += ['--approach-zone',   a['approach_zone']]
    if 'min_duty'        in a:      cmd += ['--min-duty',        a['min_duty']]
    if 'float_height'    in a:      cmd += ['--float-height',    a['float_height']]
    _release_gpio()   # let depthadjust.py claim the pins cleanly
    _mission_proc = subprocess.Popen(cmd, cwd=BASE_DIR)
    return jsonify({'status': 'mission started', 'pid': _mission_proc.pid,
                    'test_mode': test, 'cmd': cmd[2:]})


@app.route('/status')
def status():
    running = _mission_running()
    if not running:
        _ensure_gpio()
    stage = _mission_stage()
    return jsonify({
        'mission_running':    running,
        'stage':              stage.get('stage', 'idle'),
        'stage_time':         stage.get('time', ''),
        'profiles_complete':  stage.get('stage') in ('profiles_complete',
                                                      'surfacing', 'transmitting', 'done'),
        'bias_m':             _load_bias(),
        'packets_logged':     _packet_count(),
        'depth_m':            stage.get('depth_m'),
        'elapsed_s':          stage.get('elapsed_s'),
        'actuator':           stage.get('actuator', ''),
        'velocity_ms':        stage.get('velocity_ms'),
        'syringe_pct':        stage.get('syringe_pct'),
    })


@app.route('/abort', methods=['POST'])
def abort():
    global _mission_proc
    if _mission_proc and _mission_proc.poll() is None:
        _mission_proc.terminate()
        try:
            _mission_proc.wait(timeout=5)
        except Exception:
            _mission_proc.kill()
    _ensure_gpio()
    actuator.stopActuator()
    return jsonify({'status': 'aborted'})


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


# ── log / run history ────────────────────────────────────────────────────────

@app.route('/log')
def get_log():
    log_path = os.path.join(BASE_DIR, 'float.log')
    if not os.path.exists(log_path):
        return Response("No log file yet — mission hasn't run.\n", mimetype='text/plain')
    with open(log_path) as f:
        lines = f.readlines()
    return Response(''.join(lines[-200:]), mimetype='text/plain')


@app.route('/runs')
def list_runs():
    runs_dir = os.path.join(BASE_DIR, 'runs')
    runs = sorted(glob.glob(os.path.join(runs_dir, 'run_*.csv')), reverse=True) if os.path.exists(runs_dir) else []
    names = [os.path.basename(r) for r in runs]
    rows  = ''.join(
        f'<tr><td>{n}</td>'
        f'<td><a href="/runs/{n}" download>download</a></td>'
        f'<td><a href="/runs/{n}/plot">plot</a></td></tr>'
        for n in names
    ) or '<tr><td colspan="3" style="color:#888">No archived runs yet.</td></tr>'
    html = f"""<!DOCTYPE html><html><head><meta charset="utf-8">
<title>{CALL_SIGN} Run History</title>
<style>body{{font-family:system-ui,sans-serif;padding:1rem 1.5rem;background:#f4f6f8}}
h1{{font-size:1.2rem}}a{{color:#0077b6}}
table{{border-collapse:collapse;width:100%}}th,td{{padding:.4rem .8rem;border-bottom:1px solid #eee;text-align:left}}
th{{background:#f0f2f5;font-size:.82rem;text-transform:uppercase}}</style></head>
<body>{_nav('/runs')}<h1 style="margin-top:0">&#x1F4C1; {CALL_SIGN} Run History</h1>
<table><thead><tr><th>Run</th><th>CSV</th><th>Plot</th></tr></thead>
<tbody>{rows}</tbody></table></body></html>"""
    return Response(html, mimetype='text/html')


@app.route('/runs/<name>')
def download_run(name):
    if not name.endswith('.csv') or '/' in name:
        return jsonify({'error': 'invalid'}), 400
    path = os.path.join(BASE_DIR, 'runs', name)
    if not os.path.exists(path):
        return jsonify({'error': 'not found'}), 404
    return send_file(path, mimetype='text/csv', as_attachment=True, download_name=name)


@app.route('/runs/<name>/plot')
def run_plot(name):
    if not name.endswith('.csv') or '/' in name:
        return jsonify({'error': 'invalid'}), 400
    path = os.path.join(BASE_DIR, 'runs', name)
    if not os.path.exists(path):
        return jsonify({'error': 'not found'}), 404
    times, depths = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            times.append(float(row['elapsed_s']))
            depths.append(float(row['depth_m']))
    if len(depths) < 2:
        return jsonify({'error': 'not enough data'}), 400
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(times, depths, 'b-o', markersize=3.5, linewidth=1.5)
    ax.invert_yaxis()
    ax.set_xlabel('Elapsed Time (s)')
    ax.set_ylabel('Depth (m)')
    ax.set_title(f'{CALL_SIGN} — {name}', fontsize=12)
    ax.axhline(y=TARGET_BOTTOM_M,  color='r', linestyle='--', alpha=0.7, label=f'Bottom {TARGET_BOTTOM_M} m')
    ax.axhline(y=TARGET_SURFACE_M, color='g', linestyle='--', alpha=0.7, label=f'Surface {TARGET_SURFACE_M} m')
    ax.legend(); ax.grid(True, alpha=0.3); plt.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format='jpeg', dpi=150)
    plt.close(fig); buf.seek(0)
    return send_file(buf, mimetype='image/jpeg')


# ── config / tuning ──────────────────────────────────────────────────────────

@app.route('/config')
def get_config():
    return jsonify({
        'duty_cycle':         ACTUATOR_DUTY_CYCLE,
        'deadband_m':         CONTROL_DEADBAND_M,
        'surface_delay_s':    TEST_SURFACE_DELAY_S,
        'surface_extend_s':   TEST_SURFACE_EXTEND_S,
        'target_bottom_m':    TARGET_BOTTOM_M,
        'target_surface_m':   TARGET_SURFACE_M,
        'tolerance_m':        TOLERANCE_M,
        'sensor_offset_m':    SENSOR_DEPTH_OFFSET_M,
        'float_height_m':     FLOAT_HEIGHT_M,
        'approach_zone_m':    APPROACH_ZONE_M,
        'min_duty_pct':       MIN_DUTY_PCT,
        'sim_rate':           0.08,
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
  .warn{{background:#fff3cd;color:#856404}}
  #plot-img{{max-width:100%;border-radius:6px;margin-top:.5rem}}
</style>
</head>
<body>
{_nav('/tuning')}
<h1 style="margin-top:0">&#x1F9EA; {CALL_SIGN} — Test &amp; Tuning</h1>

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
    <label>Approach Zone (m)</label>
    <div class="desc">Start slowing when this far from target. Duty ramps from max down to min. Default: {APPROACH_ZONE_M} m</div>
    <div class="row">
      <input type="range" id="approach-zone" min="0.1" max="1.5" step="0.05" value="{APPROACH_ZONE_M}"
             oninput="document.getElementById('az-val').textContent=parseFloat(this.value).toFixed(2)+' m'">
      <span class="val" id="az-val">{APPROACH_ZONE_M:.2f} m</span>
    </div>
  </div>

  <div class="param">
    <label>Min Duty % (near target)</label>
    <div class="desc">Minimum speed when very close to target. Below 25% the motor may stall. Default: {MIN_DUTY_PCT}%</div>
    <div class="row">
      <input type="range" id="min-duty" min="15" max="60" step="5" value="{MIN_DUTY_PCT}"
             oninput="document.getElementById('md-val').textContent=this.value+'%'">
      <span class="val" id="md-val">{MIN_DUTY_PCT}%</span>
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
  <h2>Depth Settings</h2>

  <div class="param">
    <label>Float Body Height (m)</label>
    <div class="desc">Total height of the float body. Used for the ascent target:
    sensor must be deeper than the surface target by (height − sensor offset) so the
    <em>top</em> of the float reaches the competition depth. Measure once, set in config.py.</div>
    <div class="row">
      <input type="number" id="float-height" min="0" max="1.0" step="0.01"
             value="{FLOAT_HEIGHT_M:.3f}" oninput="updateTargets()"> m
    </div>
  </div>

  <div class="param">
    <label>Sensor Position Offset (m)</label>
    <div class="desc">Distance from pressure sensor to <em>bottom</em> of float.
    Descent sensor target = competition depth − offset. Measure once, set in config.py.</div>
    <div class="row">
      <input type="number" id="sensor-offset" min="-0.5" max="0.5" step="0.01"
             value="{SENSOR_DEPTH_OFFSET_M:.3f}" oninput="updateTargets()"> m
    </div>
  </div>

  <div class="param">
    <label>Target Bottom Depth (m) <span style="font-weight:400;color:#888;font-size:.8rem">— competition: {TARGET_BOTTOM_M} m</span></label>
    <div class="desc">Reduce for shallow pool tests. Sensor will target this minus the offset above.</div>
    <div class="row">
      <input type="range" id="target-bottom" min="0.5" max="3.0" step="0.1" value="{TARGET_BOTTOM_M}"
             oninput="updateTargets()">
      <span class="val" id="tb-val">{TARGET_BOTTOM_M:.2f} m</span>
    </div>
  </div>

  <div class="param">
    <label>Target Surface Depth (m) <span style="font-weight:400;color:#888;font-size:.8rem">— competition: {TARGET_SURFACE_M} m</span></label>
    <div class="desc">Adjust if testing at a different surface hold depth.</div>
    <div class="row">
      <input type="range" id="target-surface" min="0.1" max="1.0" step="0.05" value="{TARGET_SURFACE_M}"
             oninput="updateTargets()">
      <span class="val" id="ts-val">{TARGET_SURFACE_M:.2f} m</span>
    </div>
  </div>

  <div style="font-size:.82rem;background:#f8f8f8;border-radius:6px;padding:.5rem .75rem;margin-top:.4rem">
    Effective sensor targets: bottom <b id="eff-bottom">—</b> &nbsp;|&nbsp; surface <b id="eff-surface">—</b>
  </div>
</div>

<div class="card">
  <h2>Fixed (Competition Rules)</h2>
  <div class="info-row"><span>Valid window</span><span>±{TOLERANCE_M} m ({TOLERANCE_M*100:.0f} cm)</span></div>
  <div class="info-row"><span>Hold time</span><span>30 s · 7 packets · 5 s interval</span></div>
  <div class="info-row"><span>Profiles</span><span>2</span></div>
</div>

<div class="card">
  <h2>Simulation (dry run)</h2>
  <div class="desc" style="margin-bottom:.5rem">
    Simulates depth sensor — actuator still moves so the syringe runs dry.
    Use this to verify the UI, data logging, and control logic without water.
  </div>
  <div class="param">
    <label>Sim descent/ascent rate (m/s)</label>
    <div class="desc">Speed of virtual buoyancy change. 0.08 m/s = ~4 min full run. Try 0.30 on a Mac for a faster demo.</div>
    <div class="row">
      <input type="range" id="sim-rate" min="0.02" max="0.5" step="0.02" value="0.08"
             oninput="document.getElementById('sr-val').textContent=parseFloat(this.value).toFixed(2)+' m/s'">
      <span class="val" id="sr-val">0.08 m/s</span>
    </div>
  </div>
  <button class="btn-neutral" style="background:#5a3e85"
          onclick="startSim()">&#x1F52C; Sim Run (dry)</button>
  <button class="btn-stop" onclick="abortRun()">&#x1F6D1; Abort</button>
</div>

<div class="card">
  <h2>Real Run</h2>
  <div id="msg"></div>
  <div style="margin-bottom:.3rem">Mission: <span id="mission-badge" class="pill idle">idle</span> &nbsp; Packets: <span id="packets">—</span></div>
  <div style="font-size:.82rem;color:#555;margin-bottom:.5rem">Stage: <span id="stage-lbl">—</span></div>
  <button class="btn-test" onclick="startTest()">&#x1F9EA; Test Run (surfaces)</button>
  <button class="btn-stop" onclick="stopRun()">&#x23F9; Stop actuator</button>
  <button class="btn-stop" onclick="abortRun()" style="background:#7b0000">&#x1F6D1; Abort mission</button>
  <button class="btn-neutral" onclick="location.href='/plot'" style="float:right">&#x1F4C8; View plot</button>
</div>

<div id="live-card" style="display:none" class="card">
  <h2>Live View</h2>
  <div style="display:flex;gap:1.5rem;align-items:flex-start">
    <div style="flex:0 0 auto;position:relative;height:280px;width:96px">
      <div style="position:absolute;left:0;top:0;width:44px;height:280px;
                  background:linear-gradient(180deg,#e8f6ff,#b8d8f0);
                  border:2px solid #8aabb8;border-radius:5px;overflow:hidden">
        <div id="g-surf-zone" style="position:absolute;width:100%;background:rgba(39,174,96,.22)"></div>
        <div id="g-bot-zone"  style="position:absolute;width:100%;background:rgba(192,57,43,.18)"></div>
        <div id="g-float"     style="position:absolute;left:2px;width:36px;height:10px;
                                     border-radius:3px;background:#0077b6;
                                     transition:top .5s ease,background .3s ease"></div>
      </div>
      <div style="position:absolute;left:50px;top:-6px;font-size:.65rem;color:#888">0 m</div>
      <div id="g-lbl-surf" style="position:absolute;left:50px;font-size:.65rem;color:#27ae60;font-weight:700;white-space:nowrap"></div>
      <div id="g-lbl-bot"  style="position:absolute;left:50px;font-size:.65rem;color:#c0392b;font-weight:700;white-space:nowrap"></div>
      <div style="position:absolute;left:50px;bottom:-6px;font-size:.65rem;color:#888">3 m</div>
    </div>
    <div>
      <div id="l-depth" style="font-size:2.2rem;font-weight:700;font-family:monospace;color:#0077b6;line-height:1.1">—</div>
      <div style="font-size:.75rem;color:#888;margin-bottom:.7rem">metres depth</div>
      <table style="font-size:.86rem;border-collapse:collapse">
        <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Elapsed</td><td id="l-elapsed" style="font-family:monospace;font-weight:600">—</td></tr>
        <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Stage</td>  <td id="l-stage"    style="font-weight:600">—</td></tr>
        <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Velocity</td><td id="l-velocity" style="font-family:monospace;font-weight:600">—</td></tr>
        <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Motor</td>  <td id="l-actuator" style="font-weight:600">—</td></tr>
        <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Packets</td><td id="l-packets"  style="font-weight:600">—</td></tr>
      </table>
      <div id="l-syringe-section" style="display:none;margin-top:.6rem;min-width:180px">
        <div style="font-size:.75rem;color:#888;margin-bottom:.2rem">
          Syringe &nbsp;<b id="l-syringe-lbl"></b>
        </div>
        <div style="position:relative;height:10px;background:#e0e0e0;border-radius:5px;overflow:hidden">
          <div id="l-syringe-fill" style="position:absolute;left:0;top:0;height:100%;width:50%;
               border-radius:5px;transition:width .5s ease,background .3s ease"></div>
        </div>
        <div style="display:flex;justify-content:space-between;font-size:.6rem;color:#aaa;margin-top:.1rem">
          <span>empty ↑</span><span>neutral</span><span>full ↓</span>
        </div>
      </div>
    </div>
  </div>
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

const _GH = 280, _GM = 3.1;   // gauge height px, max depth m
function _d2y(d) {{ return Math.max(0, Math.min(_GH, d / _GM * _GH)); }}

function initGauge() {{
  const tb  = parseFloat(document.getElementById('target-bottom').value)  || {TARGET_BOTTOM_M};
  const ts  = parseFloat(document.getElementById('target-surface').value) || {TARGET_SURFACE_M};
  const tol = 0.33;
  const bz  = document.getElementById('g-bot-zone');
  if (!bz) return;
  bz.style.top    = _d2y(tb - tol) + 'px';
  bz.style.height = (_d2y(tb + tol) - _d2y(tb - tol)) + 'px';
  const bl = document.getElementById('g-lbl-bot');
  bl.style.top = (_d2y(tb) - 7) + 'px';
  bl.textContent = tb.toFixed(1) + ' m';
  const sz = document.getElementById('g-surf-zone');
  sz.style.top    = _d2y(ts - tol) + 'px';
  sz.style.height = (_d2y(ts + tol) - _d2y(ts - tol)) + 'px';
  const sl = document.getElementById('g-lbl-surf');
  sl.style.top = (_d2y(ts) - 7) + 'px';
  sl.textContent = ts.toFixed(1) + ' m';
}}

function updateTargets() {{
  const tb  = parseFloat(document.getElementById('target-bottom').value);
  const ts  = parseFloat(document.getElementById('target-surface').value);
  const off = parseFloat(document.getElementById('sensor-offset').value) || 0;
  const fh  = parseFloat(document.getElementById('float-height').value)  || 0;
  document.getElementById('tb-val').textContent = tb.toFixed(2) + ' m';
  document.getElementById('ts-val').textContent = ts.toFixed(2) + ' m';
  document.getElementById('eff-bottom').textContent  = (tb - off).toFixed(2) + ' m';
  document.getElementById('eff-surface').textContent = (ts + fh - off).toFixed(2) + ' m';
  initGauge();
}}

function buildUrl(extra='') {{
  const duty    = document.getElementById('duty').value;
  const db      = document.getElementById('deadband').value;
  const delay   = document.getElementById('surface-delay').value;
  const extend  = document.getElementById('surface-extend').value;
  const tb      = document.getElementById('target-bottom').value;
  const ts      = document.getElementById('target-surface').value;
  const off     = document.getElementById('sensor-offset').value;
  const az      = document.getElementById('approach-zone').value;
  const md      = document.getElementById('min-duty').value;
  const fh      = document.getElementById('float-height').value;
  return `/start?test=true&duty=${{duty}}&deadband=${{db}}&surface_delay=${{delay}}&surface_extend=${{extend}}&target_bottom=${{tb}}&target_surface=${{ts}}&sensor_offset=${{off}}&approach_zone=${{az}}&min_duty=${{md}}&float_height=${{fh}}${{extra}}`;
}}

let _liveInterval = null;

function startLive() {{
  document.getElementById('live-card').style.display = '';
  initGauge();
  if (_liveInterval) clearInterval(_liveInterval);
  _liveInterval = setInterval(updateLive, 1000);
  updateLive();
}}

function updateLive() {{
  fetch('/status').then(r=>r.json()).then(d=>{{
    // mission badge + stage (Real Run card)
    const b = document.getElementById('mission-badge');
    const label = d.profiles_complete ? 'transmitting' : (d.mission_running ? 'running' : 'idle');
    const cls   = d.profiles_complete ? 'warn' : (d.mission_running ? 'running' : 'idle');
    b.textContent = label; b.className = 'pill ' + cls;
    document.getElementById('packets').textContent = d.packets_logged;
    document.getElementById('stage-lbl').textContent =
      d.stage ? d.stage.replace(/_/g,' ') + (d.stage_time ? '  ' + d.stage_time : '') : '—';

    // live panel stats
    document.getElementById('l-depth').textContent   = d.depth_m != null ? (+d.depth_m).toFixed(2) : '—';
    document.getElementById('l-elapsed').textContent = d.elapsed_s != null ? (+d.elapsed_s).toFixed(0) + ' s' : '—';
    document.getElementById('l-stage').textContent   = d.stage ? d.stage.replace(/_/g,' ') : '—';
    document.getElementById('l-packets').textContent = d.packets_logged;
    const vel = d.velocity_ms;
    document.getElementById('l-velocity').textContent =
      vel != null ? (vel >= 0 ? '+' : '') + (+vel).toFixed(3) + ' m/s' : '—';
    const act = document.getElementById('l-actuator');
    const astr = d.actuator || '—';
    act.textContent = astr;
    act.style.color = astr.includes('↓') ? '#c0392b' :
                      astr.includes('↑') ? '#27ae60' : '#555';

    if (d.syringe_pct != null) {{
      const sp = +d.syringe_pct;
      const ss = document.getElementById('l-syringe-section');
      if (ss) ss.style.display = '';
      document.getElementById('l-syringe-fill').style.width      = sp + '%';
      document.getElementById('l-syringe-fill').style.background =
        sp > 60 ? '#c0392b' : sp < 40 ? '#27ae60' : '#e8a000';
      document.getElementById('l-syringe-lbl').textContent =
        sp.toFixed(0) + '%  ' + (sp > 60 ? '(heavy ↓)' : sp < 40 ? '(light ↑)' : '(neutral)');
    }}

    // gauge float marker
    if (d.depth_m != null) {{
      const m = document.getElementById('g-float');
      const markerY = Math.max(2, Math.min(_GH - 12, _d2y(+d.depth_m) - 5));
      m.style.top        = markerY + 'px';
      const v = d.velocity_ms || 0;
      m.style.background = v >  0.005 ? '#c0392b' :   // sinking
                           v < -0.005 ? '#27ae60' :    // rising
                           '#0077b6';                   // holding
    }}

    if (!d.mission_running && !d.profiles_complete) {{
      if (_liveInterval) {{ clearInterval(_liveInterval); _liveInterval = null; }}
      if (d.packets_logged >= 2) showPlot();
    }}
  }}).catch(()=>{{}});
}}

function startSim() {{
  const rate = document.getElementById('sim-rate').value;
  msg('Starting sim run…');
  fetch(buildUrl(`&sim=true&sim_rate=${{rate}}`), {{method:'POST'}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg('Sim running at ' + rate + ' m/s');
      startPolling(); startLive();
    }})
    .catch(e => msg(e.toString(), true));
}}

updateTargets();

function startTest() {{
  msg('Starting test run…');
  fetch(buildUrl(), {{method:'POST'}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg('Running: ' + JSON.stringify(d.cmd));
      startPolling(); startLive();
    }})
    .catch(e => msg(e.toString(), true));
}}

function stopRun() {{
  fetch('/stop', {{method:'POST'}}).then(() => msg('Stopped.'));
}}

function abortRun() {{
  fetch('/abort', {{method:'POST'}})
    .then(r => r.json())
    .then(d => msg(d.status || 'aborted'))
    .catch(e => msg(e.toString(), true));
}}

function startPolling() {{
  if (_polling) clearInterval(_polling);
  _polling = setInterval(pollStatus, 3000);
  pollStatus();
}}

function pollStatus() {{
  fetch('/status').then(r=>r.json()).then(d=>{{
    const b = document.getElementById('mission-badge');
    const label = d.profiles_complete ? 'transmitting' : (d.mission_running ? 'running' : 'idle');
    const cls   = d.profiles_complete ? 'warn' : (d.mission_running ? 'running' : 'idle');
    b.textContent = label;
    b.className = 'pill ' + cls;
    document.getElementById('packets').textContent = d.packets_logged;
    document.getElementById('stage-lbl').textContent =
      d.stage ? d.stage.replace(/_/g,' ') + (d.stage_time ? '  ' + d.stage_time : '') : '—';
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

if _HAS_GPIO:
    GPIO.setwarnings(False)
# Actuator GPIO is NOT held at startup — claimed per-request so the
# mission subprocess (depthadjust.py) can always claim the pins cleanly.
if not depth_sensor.initSensor():
    print("WARNING: depth sensor not available at startup — will retry on first /depth request")

if __name__ == '__main__':
    import os as _os
    _port = int(_os.environ.get('FLOAT_PORT', 5000))
    app.run(host='0.0.0.0', port=_port, debug=False)
