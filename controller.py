#!/usr/bin/env python3
import csv
import hashlib
import io
import json
import os
from datetime import datetime

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import requests as req
from flask import Flask, Response, jsonify, request, send_file

from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
                    FLOAT_IP, FLOAT_PORT, CONTROLLER_PORT)

app = Flask(__name__)

BASE_DIR     = os.path.dirname(os.path.abspath(__file__))
DATA_PATH    = os.path.join(BASE_DIR, 'received_data.csv')
HASH_PATH    = os.path.join(BASE_DIR, 'received_data.hash')
EVENTS_PATH  = os.path.join(BASE_DIR, 'events.json')
FLOAT_BASE   = f"http://{FLOAT_IP}:{FLOAT_PORT}"


# ── event tracking ────────────────────────────────────────────────────────────

def _now():
    return datetime.now().strftime('%H:%M:%S')


def _load_events():
    if os.path.exists(EVENTS_PATH):
        with open(EVENTS_PATH) as f:
            return json.load(f)
    return {}


def _save_events(ev):
    with open(EVENTS_PATH, 'w') as f:
        json.dump(ev, f, indent=2)


def _record_mission_start():
    _save_events({'mission_started_at': _now(), 'receive_count': 0})


def _record_receive(is_new, packets, hsh):
    ev = _load_events()
    ev['receive_count'] = ev.get('receive_count', 0) + 1
    ev['last_receive_at'] = _now()
    ev['data_id'] = hsh
    ev['packets'] = packets
    if is_new:
        ev['first_data_at'] = ev.get('first_data_at') or _now()
    _save_events(ev)


# ── data helpers ──────────────────────────────────────────────────────────────

def _data_hash(text):
    return hashlib.md5(text.encode()).hexdigest()


def _stored_hash():
    if os.path.exists(HASH_PATH):
        with open(HASH_PATH) as f:
            return f.read().strip()
    return None


def _save_data(text):
    """Write CSV and update hash. Returns (packets, hash_short, is_new)."""
    incoming = _data_hash(text)
    is_new   = incoming != _stored_hash()
    if is_new:
        with open(DATA_PATH, 'w') as f:
            f.write(text)
        with open(HASH_PATH, 'w') as f:
            f.write(incoming)
    packets = max(0, text.count('\n') - 1)
    return packets, incoming[:8], is_new


def _packet_count():
    if not os.path.exists(DATA_PATH):
        return 0
    with open(DATA_PATH) as f:
        return max(0, sum(1 for _ in f) - 1)


def _make_plot():
    times, depths = [], []
    with open(DATA_PATH) as f:
        for row in csv.DictReader(f):
            times.append(float(row['elapsed_s']))
            depths.append(float(row['depth_m']))
    if len(depths) < 2:
        return None
    fig, ax = plt.subplots(figsize=(11, 6))
    ax.plot(times, depths, 'b-o', markersize=3.5, linewidth=1.5, label='Measured depth')
    ax.invert_yaxis()
    ax.set_ylim(max(depths) + 0.3, min(min(depths) - 0.3, -0.15))
    ax.set_xlabel('Elapsed Time (s)', fontsize=12)
    ax.set_ylabel('Depth (m)', fontsize=12)
    ax.set_title(f'{CALL_SIGN} — Vertical Profile', fontsize=13, fontweight='bold')
    ax.axhline(y=TARGET_BOTTOM_M, color='r', linestyle='--', linewidth=1.2, alpha=0.8,
               label=f'Bottom {TARGET_BOTTOM_M} m  (valid {TARGET_BOTTOM_M-TOLERANCE_M:.2f}–{TARGET_BOTTOM_M+TOLERANCE_M:.2f} m)')
    ax.axhline(y=TARGET_SURFACE_M, color='g', linestyle='--', linewidth=1.2, alpha=0.8,
               label=f'Surface {TARGET_SURFACE_M} m  (valid {TARGET_SURFACE_M-TOLERANCE_M:.2f}–{TARGET_SURFACE_M+TOLERANCE_M:.2f} m)')
    ax.fill_between([min(times), max(times)],
                    TARGET_BOTTOM_M - TOLERANCE_M, TARGET_BOTTOM_M + TOLERANCE_M,
                    color='red', alpha=0.06)
    ax.fill_between([min(times), max(times)],
                    TARGET_SURFACE_M - TOLERANCE_M, TARGET_SURFACE_M + TOLERANCE_M,
                    color='green', alpha=0.06)
    ax.legend(fontsize=9, loc='lower right')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    buf = io.BytesIO()
    fig.savefig(buf, format='jpeg', dpi=150)
    plt.close(fig)
    buf.seek(0)
    return buf


def _nav(active=''):
    pages = [('/', 'Home'), ('/tuning', 'Tuning'), ('/comp', 'Competition'),
             ('/float/runs', 'Float Runs'), ('/float/log', 'Float Log')]
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


# ── UI ────────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{CALL_SIGN} Controller</title>
<style>
  *{{box-sizing:border-box}}
  body{{font-family:system-ui,sans-serif;margin:0;padding:1rem 1.5rem;background:#f0f2f5;color:#1a1a2e}}
  h1{{margin:0 0 1rem;font-size:1.4rem}}
  h2{{margin:0 0 .6rem;font-size:.85rem;text-transform:uppercase;letter-spacing:.06em;color:#555;font-weight:600}}
  .grid2{{display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-bottom:1rem}}
  .card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.08);margin-bottom:1rem}}
  .row{{display:flex;justify-content:space-between;align-items:center;padding:.2rem 0;font-size:.92rem;border-bottom:1px solid #f0f0f0}}
  .row:last-child{{border-bottom:none}}
  .label{{color:#555}}
  .pill{{display:inline-block;padding:.15rem .6rem;border-radius:999px;font-size:.78rem;font-weight:700}}
  .ok{{background:#d4edda;color:#155724}}
  .err{{background:#f8d7da;color:#721c24}}
  .idle{{background:#e2e3e5;color:#383d41}}
  .running{{background:#cce5ff;color:#004085}}
  .warn{{background:#fff3cd;color:#856404}}
  button{{cursor:pointer;border:none;border-radius:6px;padding:.45rem .9rem;font-size:.88rem;font-weight:600;margin:.2rem .2rem .2rem 0;transition:opacity .15s}}
  button:active{{opacity:.7}}
  .btn-primary{{background:#0077b6;color:#fff}}
  .btn-warn{{background:#e07b00;color:#fff}}
  .btn-danger{{background:#c0392b;color:#fff}}
  .btn-neutral{{background:#555;color:#fff}}
  #msg{{min-height:1.1rem;font-size:.82rem;color:#0077b6;margin:.3rem 0}}
  a.dl{{font-size:.85rem;color:#0077b6;margin-right:.75rem;text-decoration:none}}
  a.dl:hover{{text-decoration:underline}}
  .tl-row{{display:flex;gap:.75rem;align-items:baseline;padding:.3rem 0;border-bottom:1px solid #f4f4f4;font-size:.92rem}}
  .tl-row:last-child{{border-bottom:none}}
  .tl-label{{color:#555;min-width:11rem}}
  .tl-val{{font-weight:600;font-family:monospace}}
  .tl-note{{color:#888;font-size:.8rem;margin-left:.25rem}}
  #plot-img{{max-width:100%;border-radius:6px;display:block}}
  table{{width:100%;border-collapse:collapse;font-size:.82rem}}
  th{{background:#f0f2f5;padding:.4rem .6rem;text-align:left;font-weight:600;font-size:.78rem;text-transform:uppercase;letter-spacing:.04em;color:#444;position:sticky;top:0}}
  td{{padding:.35rem .6rem;border-bottom:1px solid #f0f0f0;font-family:monospace}}
  tr:last-child td{{border-bottom:none}}
  .scroll{{max-height:320px;overflow-y:auto;border:1px solid #e8e8e8;border-radius:6px}}
  .no-data{{color:#888;font-style:italic;padding:.5rem 0}}
</style>
</head>
<body>
{_nav('/')}
<h1 style="margin-top:0">&#x1F9FF; {CALL_SIGN} Controller</h1>

<div class="grid2">
  <div class="card">
    <h2>Float Live Status</h2>
    <div class="row"><span class="label">Connection</span><span id="float-conn" class="pill idle">checking…</span></div>
    <div class="row"><span class="label">Mission</span><span id="float-mission" class="pill idle">—</span></div>
    <div class="row"><span class="label">Packets (on float)</span><span id="float-packets">—</span></div>
    <div class="row"><span class="label">Depth</span><span id="float-depth">—</span></div>
    <div class="row"><span class="label">Pressure</span><span id="float-pressure">—</span></div>
    <div class="row"><span class="label">Bias</span><span id="float-bias">—</span></div>
    <br><a href="{FLOAT_BASE}" target="_blank" style="font-size:.82rem;color:#555">&#x2197; Float UI direct</a>
  </div>

  <div class="card">
    <h2>Float Controls (proxied)</h2>
    <div id="msg"></div>
    <button class="btn-primary" onclick="floatCmd('POST','/calibrate')">&#x1F4CF; Calibrate bias</button>
    <button class="btn-primary" onclick="floatCmd('POST','/start')">&#x25B6; Start mission</button>
    <button class="btn-warn"    onclick="floatCmd('POST','/start?test=true')">&#x1F9EA; Test run</button>
    <button class="btn-danger"  onclick="floatCmd('POST','/stop')">&#x23F9; Stop actuator</button>
    <br>
    <button class="btn-neutral" onclick="floatCmd('POST','/extend?duration=10')">&#x2193; Extend 10s</button>
    <button class="btn-neutral" onclick="floatCmd('POST','/retract?duration=10')">&#x2191; Retract 10s</button>
    <br><br>
    <button class="btn-warn" onclick="ctrlCmd('POST','/fetch')">&#x2B07; Fetch CSV from float</button>
    &nbsp;<a class="dl" href="/data" download>&#x2B73; Download CSV</a>
  </div>
</div>

<div class="card">
  <h2>Mission Summary</h2>
  <div class="tl-row"><span class="tl-label">&#x25B6; Mission started</span><span class="tl-val" id="ev-start">—</span></div>
  <div class="tl-row"><span class="tl-label">&#x2705; First data received</span><span class="tl-val" id="ev-first"><span class="pill warn">waiting…</span></span><span class="tl-note" id="ev-lag"></span></div>
  <div class="tl-row"><span class="tl-label">&#x1F4E1; Last transmission</span><span class="tl-val" id="ev-last">—</span><span class="tl-note" id="ev-count"></span></div>
  <div class="tl-row"><span class="tl-label">&#x1F4E6; Packets here</span><span class="tl-val" id="ev-packets">—</span></div>
  <div class="tl-row"><span class="tl-label">&#x1F511; Data ID</span><span class="tl-val" id="ev-id" style="font-size:.78rem;color:#888">—</span></div>
</div>

<div class="card" id="plot-card">
  <h2>Depth Profile Plot &nbsp;<a href="/plot" target="_blank" style="font-size:.78rem;font-weight:400;color:#0077b6">open full size</a></h2>
  <div id="plot-container"><p class="no-data">No data yet.</p></div>
</div>

<div class="card">
  <h2>Raw Data</h2>
  <div class="scroll">
    <table>
      <thead><tr><th>#</th><th>Elapsed (s)</th><th>Depth (m)</th><th>Pressure (kPa)</th><th>Packet</th></tr></thead>
      <tbody id="data-table"><tr><td colspan="5" style="color:#888;font-style:italic;padding:.5rem">No data yet.</td></tr></tbody>
    </table>
  </div>
</div>

<script>
let _lastDataId = null;

function msg(text, err) {{
  const el = document.getElementById('msg');
  el.textContent = text;
  el.style.color = err ? '#c0392b' : '#0077b6';
}}

function floatCmd(method, path) {{
  msg('sending…');
  fetch('/float' + path, {{method}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg(JSON.stringify(d));
      // record mission start client-side label update happens via /events poll
    }})
    .catch(e => msg(e.toString(), true));
}}

function ctrlCmd(method, url) {{
  msg('sending…');
  fetch(url, {{method}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg(JSON.stringify(d));
    }})
    .catch(e => msg(e.toString(), true));
}}

function refreshFloatStatus() {{
  fetch('/float/status')
    .then(r => r.json())
    .then(d => {{
      document.getElementById('float-conn').textContent = 'reachable';
      document.getElementById('float-conn').className   = 'pill ok';
      const mb = document.getElementById('float-mission');
      mb.textContent = d.mission_running ? 'running' : 'idle';
      mb.className   = 'pill ' + (d.mission_running ? 'running' : 'idle');
      document.getElementById('float-bias').textContent    = d.bias_m.toFixed(4) + ' m';
      document.getElementById('float-packets').textContent = d.packets_logged;
    }})
    .catch(() => {{
      document.getElementById('float-conn').textContent = 'unreachable';
      document.getElementById('float-conn').className   = 'pill err';
    }});

  fetch('/float/depth')
    .then(r => r.json())
    .then(d => {{
      if (d.depth_m !== undefined) {{
        document.getElementById('float-depth').textContent    = d.depth_m.toFixed(3) + ' m';
        document.getElementById('float-pressure').textContent = d.pressure_kpa.toFixed(1) + ' kPa';
      }}
    }})
    .catch(() => {{}});
}}

function refreshSummary() {{
  fetch('/events')
    .then(r => r.json())
    .then(ev => {{
      document.getElementById('ev-start').textContent   = ev.mission_started_at ?? '—';
      document.getElementById('ev-last').textContent    = ev.last_receive_at    ?? '—';
      document.getElementById('ev-packets').textContent = ev.packets            ?? '—';
      document.getElementById('ev-id').textContent      = ev.data_id            ?? '—';

      if (ev.receive_count)
        document.getElementById('ev-count').textContent = ev.receive_count + ' transmission(s)';

      if (ev.first_data_at) {{
        document.getElementById('ev-first').innerHTML = '<span style="font-weight:600;font-family:monospace">' + ev.first_data_at + '</span>';
        if (ev.mission_started_at && ev.first_data_at) {{
          document.getElementById('ev-lag').textContent = '(' + ev.lag_note + ')';
        }}
      }}

      // Only reload plot and table when data ID changes
      if (ev.data_id && ev.data_id !== _lastDataId) {{
        _lastDataId = ev.data_id;
        refreshPlot();
        refreshTable();
      }}
    }})
    .catch(() => {{}});
}}

function refreshPlot() {{
  const container = document.getElementById('plot-container');
  const img = document.createElement('img');
  img.id    = 'plot-img';
  img.src   = '/plot?t=' + Date.now();
  img.onerror = () => {{ container.innerHTML = '<p class="no-data">Plot not available yet.</p>'; }};
  container.innerHTML = '';
  container.appendChild(img);
}}

function refreshTable() {{
  fetch('/rawdata')
    .then(r => r.json())
    .then(rows => {{
      const tbody = document.getElementById('data-table');
      if (!rows.length) {{
        tbody.innerHTML = '<tr><td colspan="5" style="color:#888;font-style:italic;padding:.5rem">No data yet.</td></tr>';
        return;
      }}
      tbody.innerHTML = rows.map((r, i) =>
        `<tr>
          <td>${{i+1}}</td>
          <td>${{parseFloat(r.elapsed_s).toFixed(1)}}</td>
          <td>${{parseFloat(r.depth_m).toFixed(4)}}</td>
          <td>${{parseFloat(r.pressure_kpa).toFixed(1)}}</td>
          <td>${{r.packet}}</td>
        </tr>`
      ).join('');
    }})
    .catch(() => {{}});
}}

refreshFloatStatus();
refreshSummary();
setInterval(refreshFloatStatus, 5000);
setInterval(refreshSummary, 5000);
</script>
</body>
</html>"""
    return Response(html, mimetype='text/html')


# ── tuning page ──────────────────────────────────────────────────────────────

@app.route('/tuning')
def tuning_page():
    # Load current config from float; fall back to defaults if unreachable
    cfg = {}
    try:
        r = req.get(f"{FLOAT_BASE}/config", timeout=3)
        if r.status_code == 200:
            cfg = r.json()
    except Exception:
        pass

    duty         = cfg.get('duty_cycle',         80)
    db           = cfg.get('deadband_m',         0.03)
    delay        = cfg.get('surface_delay_s',    60)
    extend       = cfg.get('surface_extend_s',   30)
    bot          = cfg.get('target_bottom_m',    TARGET_BOTTOM_M)
    surf         = cfg.get('target_surface_m',   TARGET_SURFACE_M)
    tol          = cfg.get('tolerance_m',        TOLERANCE_M)
    offset       = cfg.get('sensor_offset_m',    0.0)
    approach_zone = cfg.get('approach_zone_m',   0.50)
    min_duty     = cfg.get('min_duty_pct',       30)
    float_ok = bool(cfg)

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{CALL_SIGN} — Tuning</title>
<style>
  body{{font-family:system-ui,sans-serif;margin:0;padding:1rem 1.5rem;background:#f0f2f5;color:#1a1a2e}}
  h1{{margin:0 0 .25rem;font-size:1.3rem}}
  .back{{font-size:.85rem;color:#0077b6;text-decoration:none}}
  .card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.08);margin:1rem 0}}
  h2{{margin:0 0 .75rem;font-size:.9rem;text-transform:uppercase;letter-spacing:.05em;color:#555}}
  .param{{margin:.7rem 0}}
  .param label{{display:block;font-size:.9rem;font-weight:600;margin-bottom:.15rem}}
  .desc{{font-size:.78rem;color:#666;margin-bottom:.3rem}}
  .row{{display:flex;align-items:center;gap:.6rem}}
  input[type=range]{{flex:1;accent-color:#0077b6}}
  input[type=number]{{width:5rem;padding:.35rem .5rem;border:1px solid #ccc;border-radius:4px;font-size:.9rem}}
  .val{{min-width:4rem;font-weight:700;font-family:monospace;font-size:.9rem;color:#0077b6}}
  .info-row{{display:flex;justify-content:space-between;font-size:.88rem;padding:.25rem 0;border-bottom:1px solid #f0f0f0}}
  .info-row:last-child{{border:none}}
  .status-row{{display:flex;justify-content:space-between;align-items:center;padding:.2rem 0;font-size:.92rem}}
  button{{cursor:pointer;border:none;border-radius:6px;padding:.5rem 1rem;font-size:.9rem;font-weight:700;margin:.25rem .25rem 0 0}}
  .btn-test{{background:#e07b00;color:#fff;font-size:1rem;padding:.65rem 1.5rem}}
  .btn-stop{{background:#c0392b;color:#fff}}
  .btn-neutral{{background:#555;color:#fff}}
  #msg{{min-height:1.1rem;font-size:.82rem;color:#0077b6;margin:.4rem 0}}
  .pill{{display:inline-block;padding:.15rem .55rem;border-radius:999px;font-size:.78rem;font-weight:700}}
  .running{{background:#cce5ff;color:#004085}}
  .idle{{background:#e2e3e5;color:#383d41}}
  .warn{{background:#fff3cd;color:#856404}}
  .ok{{background:#d4edda;color:#155724}}
  .err{{background:#f8d7da;color:#721c24}}
  .grid2{{display:grid;grid-template-columns:1fr 1fr;gap:1rem}}
  #plot-img{{max-width:100%;border-radius:6px;display:block;margin-top:.5rem}}
  table{{width:100%;border-collapse:collapse;font-size:.82rem}}
  th{{background:#f0f2f5;padding:.35rem .6rem;text-align:left;font-weight:600;font-size:.78rem;text-transform:uppercase;color:#444;position:sticky;top:0}}
  td{{padding:.3rem .6rem;border-bottom:1px solid #f0f0f0;font-family:monospace}}
  .scroll{{max-height:260px;overflow-y:auto;border:1px solid #e8e8e8;border-radius:6px;margin-top:.5rem}}
</style>
</head>
<body>
{_nav('/tuning')}
<h1 style="margin-top:0">&#x1F9EA; {CALL_SIGN} — Test &amp; Tuning</h1>

{'<p style="color:#c0392b;font-size:.85rem">⚠ Float unreachable — showing default values. Parameters will be applied when run starts.</p>' if not float_ok else '<p style="color:#155724;font-size:.85rem">✓ Loaded current config from float.</p>'}

<div class="grid2">

<div>
  <div class="card">
    <h2>Tunable Parameters</h2>

    <div class="param">
      <label>Actuator Duty Cycle</label>
      <div class="desc">Motor speed. Lower = slower piston, less overshoot.</div>
      <div class="row">
        <input type="range" id="duty" min="20" max="100" step="5" value="{duty}"
               oninput="document.getElementById('duty-val').textContent=this.value+'%'">
        <span class="val" id="duty-val">{duty}%</span>
      </div>
    </div>

    <div class="param">
      <label>Control Deadband (m)</label>
      <div class="desc">Stop zone around target. Wider = less hunting, more drift.</div>
      <div class="row">
        <input type="range" id="deadband" min="0.01" max="0.15" step="0.01" value="{db}"
               oninput="document.getElementById('db-val').textContent=parseFloat(this.value).toFixed(2)+' m'">
        <span class="val" id="db-val">{db:.2f} m</span>
      </div>
    </div>

    <div class="param">
      <label>Approach Zone (m)</label>
      <div class="desc">Start slowing when this far from target. Duty ramps from max down to min to reduce overshoot.</div>
      <div class="row">
        <input type="range" id="approach-zone" min="0.1" max="1.5" step="0.05" value="{approach_zone}"
               oninput="document.getElementById('az-val').textContent=parseFloat(this.value).toFixed(2)+' m'">
        <span class="val" id="az-val">{approach_zone:.2f} m</span>
      </div>
    </div>

    <div class="param">
      <label>Min Duty % (near target)</label>
      <div class="desc">Minimum speed when very close to target. Below 25% the motor may stall under pressure.</div>
      <div class="row">
        <input type="range" id="min-duty" min="15" max="60" step="5" value="{min_duty}"
               oninput="document.getElementById('md-val').textContent=this.value+'%'">
        <span class="val" id="md-val">{min_duty}%</span>
      </div>
    </div>

    <div class="param">
      <label>Surface Delay (s)</label>
      <div class="desc">Wait after profiles before surfacing.</div>
      <div class="row"><input type="number" id="surface-delay" min="10" max="300" value="{delay}"> s</div>
    </div>

    <div class="param">
      <label>Surface Extend (s)</label>
      <div class="desc">How long to run extend motor when surfacing.</div>
      <div class="row"><input type="number" id="surface-extend" min="10" max="60" value="{extend}"> s</div>
    </div>
  </div>

  <div class="card">
    <h2>Depth Settings</h2>

    <div class="param">
      <label>Sensor Position Offset (m)</label>
      <div class="desc">Distance from sensor to float's bottom reference face. Sensor target = competition target − offset.</div>
      <div class="row">
        <input type="number" id="sensor-offset" min="-0.5" max="0.5" step="0.01" value="{offset:.3f}"
               oninput="updateTargets()"> m
      </div>
    </div>

    <div class="param">
      <label>Target Bottom (m) <span style="font-weight:400;color:#888;font-size:.78rem">competition: {bot} m</span></label>
      <div class="desc">Reduce for shallow pool tests.</div>
      <div class="row">
        <input type="range" id="target-bottom" min="0.5" max="3.0" step="0.1" value="{bot}"
               oninput="updateTargets()">
        <span class="val" id="tb-val">{bot:.2f} m</span>
      </div>
    </div>

    <div class="param">
      <label>Target Surface (m) <span style="font-weight:400;color:#888;font-size:.78rem">competition: {surf} m</span></label>
      <div class="row">
        <input type="range" id="target-surface" min="0.1" max="1.0" step="0.05" value="{surf}"
               oninput="updateTargets()">
        <span class="val" id="ts-val">{surf:.2f} m</span>
      </div>
    </div>

    <div style="font-size:.82rem;background:#f8f8f8;border-radius:6px;padding:.5rem .75rem;margin-top:.4rem">
      Effective sensor targets: bottom <b id="eff-bottom">—</b> &nbsp;|&nbsp; surface <b id="eff-surface">—</b>
    </div>
  </div>

  <div class="card">
    <h2>Fixed (Competition Rules)</h2>
    <div class="info-row"><span>Valid window</span><span>±{tol} m</span></div>
    <div class="info-row"><span>Hold / packets</span><span>30 s · 7 pkts · 5 s</span></div>
    <div class="info-row"><span>Profiles</span><span>2</span></div>
  </div>

  <div class="card">
    <h2>Simulation (dry run)</h2>
    <div class="desc" style="margin-bottom:.5rem">
      Mocks the depth sensor — actuator GPIO still fires so the syringe moves dry.
      Verify UI, logging, and control logic without a pool.
    </div>
    <div class="param">
      <label>Sim rate (m/s)</label>
      <div class="row">
        <input type="range" id="sim-rate" min="0.02" max="0.5" step="0.02" value="0.08"
               oninput="document.getElementById('sr-val').textContent=parseFloat(this.value).toFixed(2)+' m/s'">
        <span class="val" id="sr-val">0.08 m/s</span>
      </div>
    </div>
    <button style="background:#5a3e85;color:#fff;border:none;border-radius:6px;padding:.5rem 1rem;font-size:.9rem;font-weight:700;cursor:pointer"
            onclick="startSim()">&#x1F52C; Sim Run (dry)</button>
    <button class="btn-stop" onclick="abortRun()">&#x1F6D1; Abort</button>
  </div>

  <div class="card">
    <h2>Run</h2>
    <div id="msg"></div>
    <div class="status-row">Float <span id="float-conn" class="pill {'ok' if float_ok else 'err'}">&nbsp;</span></div>
    <div class="status-row">Mission <span id="mission-badge" class="pill idle">idle</span></div>
    <div class="status-row" style="font-size:.82rem;color:#555">Stage <span id="stage-lbl">—</span></div>
    <div class="status-row">Packets (float) <span id="float-pkts">—</span></div>
    <div class="status-row">Received here <span id="ctrl-pkts">—</span></div>
    <br>
    <button class="btn-test" onclick="startTest()">&#x1F9EA; Start Test Run</button>
    <button class="btn-stop" onclick="stopRun()">&#x23F9; Stop</button>
    <button class="btn-stop" onclick="abortRun()" style="background:#7b0000">&#x1F6D1; Abort</button>
  </div>
</div>

<div>
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
          <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Motor</td>  <td id="l-actuator" style="font-weight:600">—</td></tr>
          <tr><td style="color:#666;padding:.15rem .7rem .15rem 0">Packets</td><td id="l-packets"  style="font-weight:600">—</td></tr>
        </table>
      </div>
    </div>
  </div>

  <div class="card" id="plot-card">
    <h2>Plot &nbsp;<a href="/plot" target="_blank" style="font-size:.78rem;font-weight:400;color:#0077b6">full size</a></h2>
    <div id="plot-container"><p style="color:#888;font-size:.85rem">Run a test to see plot here.</p></div>
  </div>

  <div class="card">
    <h2>Raw Data</h2>
    <div class="scroll">
      <table>
        <thead><tr><th>#</th><th>Elapsed</th><th>Depth (m)</th><th>Pressure (kPa)</th><th>Packet</th></tr></thead>
        <tbody id="data-table"><tr><td colspan="5" style="color:#888;padding:.5rem;font-style:italic">No data yet.</td></tr></tbody>
      </table>
    </div>
  </div>
</div>

</div><!-- grid2 -->

<script>
let _polling = null;
let _lastDataId = null;

function msg(t, err) {{
  const el = document.getElementById('msg');
  el.textContent = t;
  el.style.color = err ? '#c0392b' : '#0077b6';
}}

const _GH = 280, _GM = 3.1;
function _d2y(d) {{ return Math.max(0, Math.min(_GH, d / _GM * _GH)); }}

function initGauge() {{
  const tb  = parseFloat(document.getElementById('target-bottom').value)  || {bot};
  const ts  = parseFloat(document.getElementById('target-surface').value) || {surf};
  const tol = 0.33;
  const bz = document.getElementById('g-bot-zone');  if (!bz) return;
  bz.style.top = _d2y(tb-tol)+'px'; bz.style.height = (_d2y(tb+tol)-_d2y(tb-tol))+'px';
  const bl = document.getElementById('g-lbl-bot');
  bl.style.top = (_d2y(tb)-7)+'px'; bl.textContent = tb.toFixed(1)+' m';
  const sz = document.getElementById('g-surf-zone');
  sz.style.top = _d2y(ts-tol)+'px'; sz.style.height = (_d2y(ts+tol)-_d2y(ts-tol))+'px';
  const sl = document.getElementById('g-lbl-surf');
  sl.style.top = (_d2y(ts)-7)+'px'; sl.textContent = ts.toFixed(1)+' m';
}}

function updateTargets() {{
  const tb  = parseFloat(document.getElementById('target-bottom').value);
  const ts  = parseFloat(document.getElementById('target-surface').value);
  const off = parseFloat(document.getElementById('sensor-offset').value) || 0;
  document.getElementById('tb-val').textContent = tb.toFixed(2) + ' m';
  document.getElementById('ts-val').textContent = ts.toFixed(2) + ' m';
  document.getElementById('eff-bottom').textContent  = (tb - off).toFixed(2) + ' m';
  document.getElementById('eff-surface').textContent = (ts - off).toFixed(2) + ' m';
  initGauge();
}}

function buildCmd(extra='') {{
  const duty   = document.getElementById('duty').value;
  const db     = document.getElementById('deadband').value;
  const delay  = document.getElementById('surface-delay').value;
  const ext    = document.getElementById('surface-extend').value;
  const tb     = document.getElementById('target-bottom').value;
  const ts     = document.getElementById('target-surface').value;
  const off    = document.getElementById('sensor-offset').value;
  const az     = document.getElementById('approach-zone').value;
  const md     = document.getElementById('min-duty').value;
  return `/float/start?test=true&duty=${{duty}}&deadband=${{db}}&surface_delay=${{delay}}&surface_extend=${{ext}}&target_bottom=${{tb}}&target_surface=${{ts}}&sensor_offset=${{off}}&approach_zone=${{az}}&min_duty=${{md}}${{extra}}`;
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
  fetch('/float/status').then(r=>r.json()).then(d=>{{
    document.getElementById('float-conn').textContent = 'reachable';
    document.getElementById('float-conn').className = 'pill ok';
    const b = document.getElementById('mission-badge');
    const label = d.profiles_complete ? 'transmitting' : (d.mission_running ? 'running' : 'idle');
    const cls   = d.profiles_complete ? 'warn' : (d.mission_running ? 'running' : 'idle');
    b.textContent = label; b.className = 'pill ' + cls;
    document.getElementById('stage-lbl').textContent =
      d.stage ? d.stage.replace(/_/g,' ') + (d.stage_time ? '  '+d.stage_time : '') : '—';
    document.getElementById('float-pkts').textContent = d.packets_logged;

    document.getElementById('l-depth').textContent   = d.depth_m != null ? (+d.depth_m).toFixed(2) : '—';
    document.getElementById('l-elapsed').textContent = d.elapsed_s != null ? (+d.elapsed_s).toFixed(0)+' s' : '—';
    document.getElementById('l-stage').textContent   = d.stage ? d.stage.replace(/_/g,' ') : '—';
    document.getElementById('l-packets').textContent = d.packets_logged;
    const act = document.getElementById('l-actuator');
    const astr = d.actuator || '—';
    act.textContent = astr;
    act.style.color = astr.includes('retract') ? '#c0392b' :
                      astr.includes('extend')  ? '#27ae60' : '#555';
    if (d.depth_m != null) {{
      const m = document.getElementById('g-float');
      m.style.top        = (_d2y(+d.depth_m) - 5) + 'px';
      m.style.background = astr.includes('retract') ? '#c0392b' :
                           astr.includes('extend')  ? '#27ae60' : '#0077b6';
    }}
    if (!d.mission_running && !d.profiles_complete) {{
      if (_liveInterval) {{ clearInterval(_liveInterval); _liveInterval = null; }}
    }}
  }}).catch(() => {{
    document.getElementById('float-conn').textContent = 'unreachable';
    document.getElementById('float-conn').className = 'pill err';
  }});
  fetch('/events').then(r=>r.json()).then(ev=>{{
    document.getElementById('ctrl-pkts').textContent = ev.packets ?? '—';
    if (ev.data_id && ev.data_id !== _lastDataId) {{
      _lastDataId = ev.data_id; refreshPlot(); refreshTable();
    }}
  }}).catch(()=>{{}});
}}

function startSim() {{
  const rate = document.getElementById('sim-rate').value;
  msg('Starting sim run…');
  fetch(buildCmd(`&sim=true&sim_rate=${{rate}}`), {{method:'POST'}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg('Sim running at ' + rate + ' m/s');
      startLive();
    }})
    .catch(e => msg(e.toString(), true));
}}

function startTest() {{
  msg('Sending start command…');
  fetch(buildCmd(), {{method:'POST'}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg('Running — ' + (d.cmd ? d.cmd.join(' ') : 'ok'));
      startLive();
    }})
    .catch(e => msg(e.toString(), true));
}}

function stopRun() {{
  fetch('/float/stop', {{method:'POST'}}).then(() => msg('Stop sent.'));
}}

function abortRun() {{
  fetch('/float/abort', {{method:'POST'}})
    .then(r => r.json())
    .then(d => msg(d.status || d.error || 'aborted'))
    .catch(e => msg(e.toString(), true));
}}

function pollAll() {{
  fetch('/float/status')
    .then(r => r.json())
    .then(d => {{
      document.getElementById('float-conn').textContent = 'reachable';
      document.getElementById('float-conn').className = 'pill ok';
      const b = document.getElementById('mission-badge');
      const label = d.profiles_complete ? 'transmitting' : (d.mission_running ? 'running' : 'idle');
      const cls   = d.profiles_complete ? 'warn' : (d.mission_running ? 'running' : 'idle');
      b.textContent = label;
      b.className = 'pill ' + cls;
      document.getElementById('stage-lbl').textContent =
        d.stage ? d.stage.replace(/_/g,' ') + (d.stage_time ? '  ' + d.stage_time : '') : '—';
      document.getElementById('float-pkts').textContent = d.packets_logged;
    }})
    .catch(() => {{
      document.getElementById('float-conn').textContent = 'unreachable';
      document.getElementById('float-conn').className = 'pill err';
    }});

  fetch('/events')
    .then(r => r.json())
    .then(ev => {{
      document.getElementById('ctrl-pkts').textContent = ev.packets ?? '—';
      if (ev.data_id && ev.data_id !== _lastDataId) {{
        _lastDataId = ev.data_id;
        refreshPlot();
        refreshTable();
      }}
    }})
    .catch(() => {{}});
}}

function refreshPlot() {{
  const c = document.getElementById('plot-container');
  const img = document.createElement('img');
  img.id  = 'plot-img';
  img.src = '/plot?t=' + Date.now();
  img.onerror = () => {{ c.innerHTML = '<p style="color:#888">Plot not ready yet.</p>'; }};
  c.innerHTML = '';
  c.appendChild(img);
}}

function refreshTable() {{
  fetch('/rawdata')
    .then(r => r.json())
    .then(rows => {{
      if (!rows.length) return;
      document.getElementById('data-table').innerHTML = rows.map((r,i) =>
        `<tr><td>${{i+1}}</td><td>${{parseFloat(r.elapsed_s).toFixed(1)}}</td><td>${{parseFloat(r.depth_m).toFixed(4)}}</td><td>${{parseFloat(r.pressure_kpa).toFixed(1)}}</td><td>${{r.packet}}</td></tr>`
      ).join('');
    }})
    .catch(() => {{}});
}}

updateTargets();
// Use updateLive for all status polling (also handles gauge when running)
_polling = setInterval(updateLive, 4000);
updateLive();
</script>
</body>
</html>"""
    return Response(html, mimetype='text/html')


# ── competition page ─────────────────────────────────────────────────────────

@app.route('/comp')
def comp():
    """Clean results page for the judges — no infrastructure details."""
    packets = []
    if os.path.exists(DATA_PATH):
        with open(DATA_PATH) as f:
            packets = list(csv.DictReader(f))

    n        = len(packets)
    has_plot = n >= 2

    rows = ''.join(
        f'<tr><td>{i+1}</td><td>{p["elapsed_s"]} s</td>'
        f'<td>{float(p["depth_m"]):.2f} m</td>'
        f'<td>{float(p["pressure_kpa"]):.1f} kPa</td>'
        f'<td>{p["packet"]}</td></tr>'
        for i, p in enumerate(packets)
    ) or '<tr><td colspan="5">No data yet.</td></tr>'

    plot_section = (
        f'<img src="/plot?t={int(__import__("time").time())}" alt="Depth profile" '
        f'style="max-width:100%;border:1px solid #ccc;border-radius:4px">'
        if has_plot else
        '<p style="color:#888">Run a mission to generate the plot.</p>'
    )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Team {CALL_SIGN} — Float Results</title>
<style>
  body      {{ font-family: Arial, sans-serif; max-width: 960px; margin: 20px auto;
               padding: 0 20px; color: #222; }}
  h1        {{ color: #003366; border-bottom: 3px solid #003366; padding-bottom: 8px; }}
  h2        {{ color: #0055aa; margin-top: 1.8rem; }}
  .mission  {{ background: #f0f4ff; border-left: 4px solid #003366;
               padding: 12px 16px; margin: 16px 0; border-radius: 0 4px 4px 0; }}
  .mission li {{ margin: 4px 0; }}
  table     {{ border-collapse: collapse; width: 100%; font-size: 13px; margin-top: .5rem; }}
  th        {{ background: #003366; color: white; padding: 8px 12px; text-align: left; }}
  td        {{ padding: 5px 12px; border-bottom: 1px solid #ddd; font-family: monospace; }}
  tr:nth-child(even) {{ background: #f8f8f8; }}
  .btn      {{ display: inline-block; background: #003366; color: white;
               padding: 10px 22px; text-decoration: none; border-radius: 4px;
               margin: 8px 4px 8px 0; font-size: 14px; }}
  .tools    {{ font-size: 12px; color: #888; margin-top: 2rem;
               border-top: 1px solid #eee; padding-top: .8rem; }}
</style>
</head>
<body>
<h1>Team {CALL_SIGN} — Autonomous Float</h1>
<p style="color:#555;margin-top:-.5rem">MATE ROV Competition</p>

<div class="mission">
  <strong>What our float did:</strong>
  <ul>
    <li>Descended to 2.5 m and held depth for 30 seconds, logging data every 5 seconds</li>
    <li>Ascended to 0.4 m and held depth for 30 seconds, logging data every 5 seconds</li>
    <li>Repeated both profiles a second time</li>
    <li>Waited passively for ROV recovery — did not surface on its own</li>
    <li>Transmitted all data to our shore station after recovery</li>
  </ul>
</div>

<h2>Depth Profile</h2>
{plot_section}

<h2>Data Log &nbsp;<span style="font-weight:400;font-size:.9rem;color:#555">({n} packets)</span></h2>
<a class="btn" href="/data" download>&#x2B73; Download CSV</a>
<table>
  <thead>
    <tr><th>#</th><th>Elapsed</th><th>Depth</th><th>Pressure</th><th>Data Packet</th></tr>
  </thead>
  <tbody>{rows}</tbody>
</table>

<p class="tools"><a href="/">Team tools &#x2192;</a></p>
</body>
</html>"""
    return Response(html, mimetype='text/html')


# ── proxy ─────────────────────────────────────────────────────────────────────

@app.route('/float', methods=['GET', 'POST'])
@app.route('/float/', methods=['GET', 'POST'])
@app.route('/float/<path:path>', methods=['GET', 'POST'])
def proxy(path=''):
    if path == 'start' and request.method == 'POST':
        _record_mission_start()
        print(f"[proxy] mission start command sent at {_now()}", flush=True)
    url = f"{FLOAT_BASE}/{path}"
    try:
        r = req.request(
            method=request.method,
            url=url,
            params=request.args,
            data=request.get_data(),
            headers={k: v for k, v in request.headers if k.lower() != 'host'},
            timeout=30,
            allow_redirects=False,
        )
        return Response(r.content, status=r.status_code,
                        content_type=r.headers.get('Content-Type', 'application/json'))
    except Exception as e:
        return jsonify({'error': f'Float unreachable: {e}'}), 503


# ── data receive ──────────────────────────────────────────────────────────────

@app.route('/receive', methods=['POST'])
def receive():
    if 'file' in request.files:
        data = request.files['file'].read().decode()
    else:
        data = request.get_data(as_text=True)
    packets, hsh, is_new = _save_data(data)
    _record_receive(is_new, packets, hsh)
    if is_new:
        print(f"[receive] NEW data from {request.remote_addr} — "
              f"{packets} packets  id={hsh}", flush=True)
        status = 'received'
    else:
        print(f"[receive] heartbeat from {request.remote_addr} — "
              f"{packets} packets, no change  id={hsh}", flush=True)
        status = 'heartbeat'
    return jsonify({'status': status, 'packets': packets, 'id': hsh})


@app.route('/fetch', methods=['POST'])
def fetch():
    try:
        r = req.get(f"{FLOAT_BASE}/data", timeout=10)
        if r.status_code != 200:
            return jsonify({'error': 'Float returned no data'}), 502
        packets, hsh, is_new = _save_data(r.text)
        _record_receive(is_new, packets, hsh)
        if is_new:
            print(f"[fetch] NEW data pulled from float — "
                  f"{packets} packets  id={hsh}", flush=True)
        else:
            print(f"[fetch] pulled from float — no change  id={hsh}", flush=True)
        return jsonify({'status': 'fetched', 'packets': packets,
                        'id': hsh, 'new': is_new})
    except Exception as e:
        print(f"[fetch] failed: {e}", flush=True)
        return jsonify({'error': str(e)}), 503


# ── data serve ────────────────────────────────────────────────────────────────

@app.route('/events')
def events():
    ev = _load_events()
    # Compute a human-readable lag note if we have both timestamps
    if ev.get('mission_started_at') and ev.get('first_data_at'):
        try:
            fmt   = '%H:%M:%S'
            start = datetime.strptime(ev['mission_started_at'], fmt)
            first = datetime.strptime(ev['first_data_at'], fmt)
            secs  = int((first - start).total_seconds())
            m, s  = divmod(abs(secs), 60)
            ev['lag_note'] = f"{m}m {s}s after mission start"
        except Exception:
            ev['lag_note'] = ''
    return jsonify(ev)


@app.route('/rawdata')
def rawdata():
    if not os.path.exists(DATA_PATH):
        return jsonify([])
    rows = []
    with open(DATA_PATH) as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return jsonify(rows)


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
    buf = _make_plot()
    if buf is None:
        return jsonify({'error': 'Not enough data points'}), 400
    return send_file(buf, mimetype='image/jpeg')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=CONTROLLER_PORT, debug=False)
