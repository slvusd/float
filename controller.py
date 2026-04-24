#!/usr/bin/env python3
import csv
import io
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import requests as req
from flask import Flask, Response, jsonify, request, send_file

from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M, TOLERANCE_M,
                    FLOAT_IP, FLOAT_PORT, CONTROLLER_PORT)

app = Flask(__name__)

BASE_DIR   = os.path.dirname(os.path.abspath(__file__))
DATA_PATH  = os.path.join(BASE_DIR, 'received_data.csv')
FLOAT_BASE = f"http://{FLOAT_IP}:{FLOAT_PORT}"


# ── helpers ───────────────────────────────────────────────────────────────────

def _float_status():
    try:
        r = req.get(f"{FLOAT_BASE}/status", timeout=2)
        return r.json()
    except Exception:
        return None


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


# ── UI ────────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    packets   = _packet_count()
    has_plot  = packets >= 2
    float_url = FLOAT_BASE
    plot_tag  = (f'<img src="/plot?t={{t}}" id="plot" style="max-width:100%;border-radius:6px">'
                 if has_plot else
                 '<p style="color:#888">No data yet — fetch from float or wait for transmission.</p>')

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{CALL_SIGN} Controller</title>
<style>
  body{{font-family:system-ui,sans-serif;margin:0;padding:1rem 1.5rem;background:#f0f2f5;color:#1a1a2e}}
  h1{{margin:0 0 1rem;font-size:1.4rem}}
  .grid{{display:grid;grid-template-columns:1fr 1fr;gap:1rem;margin-bottom:1rem}}
  .card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.1)}}
  .card h2{{margin:0 0 .6rem;font-size:.9rem;text-transform:uppercase;letter-spacing:.05em;color:#555}}
  .row{{display:flex;justify-content:space-between;margin:.25rem 0;font-size:.95rem}}
  .pill{{display:inline-block;padding:.15rem .6rem;border-radius:999px;font-size:.8rem;font-weight:600}}
  .ok{{background:#d4edda;color:#155724}}
  .err{{background:#f8d7da;color:#721c24}}
  .idle{{background:#e2e3e5;color:#383d41}}
  .running{{background:#cce5ff;color:#004085}}
  button{{cursor:pointer;border:none;border-radius:6px;padding:.5rem 1rem;font-size:.9rem;font-weight:600;margin:.25rem .25rem .25rem 0}}
  .btn-primary{{background:#0077b6;color:#fff}}
  .btn-warn{{background:#e07b00;color:#fff}}
  .btn-danger{{background:#c0392b;color:#fff}}
  .btn-neutral{{background:#555;color:#fff}}
  #msg{{min-height:1.2rem;font-size:.85rem;color:#0077b6;margin:.4rem 0}}
  .plot-card{{background:#fff;border-radius:8px;padding:1rem;box-shadow:0 1px 4px rgba(0,0,0,.1)}}
  a.dl{{font-size:.85rem;color:#0077b6;margin-right:.75rem}}
  a.ext{{font-size:.85rem;color:#555}}
</style>
</head>
<body>
<h1>&#x1F9FF; {CALL_SIGN} Controller &nbsp;<small style="font-weight:400;font-size:.9rem;color:#555">port {CONTROLLER_PORT}</small></h1>

<div class="grid">

  <div class="card">
    <h2>Float Status</h2>
    <div class="row"><span>Connection</span><span id="float-conn" class="pill idle">checking...</span></div>
    <div class="row"><span>Mission</span><span id="float-mission" class="pill idle">—</span></div>
    <div class="row"><span>Bias</span><span id="float-bias">—</span></div>
    <div class="row"><span>Packets (float)</span><span id="float-packets">—</span></div>
    <div class="row"><span>Depth</span><span id="float-depth">—</span></div>
    <br>
    <a class="ext" href="{float_url}" target="_blank">&#x2197; Open float UI directly</a>
  </div>

  <div class="card">
    <h2>Float Controls (proxied)</h2>
    <div id="msg"></div>
    <button class="btn-primary" onclick="floatApi('POST','/calibrate')">&#x1F4CF; Calibrate</button>
    <button class="btn-primary" onclick="floatApi('POST','/start')">&#x25B6; Start mission</button>
    <button class="btn-danger"  onclick="floatApi('POST','/stop')">&#x23F9; Stop actuator</button>
    <br><br>
    <button class="btn-neutral" onclick="floatApi('POST','/extend?duration=10')">&#x2193; Extend 10s</button>
    <button class="btn-neutral" onclick="floatApi('POST','/retract?duration=10')">&#x2191; Retract 10s</button>
  </div>

</div>

<div class="card" style="margin-bottom:1rem">
  <h2>Data</h2>
  <div class="row"><span>Packets received here</span><span id="local-packets">{packets}</span></div>
  <br>
  <button class="btn-warn" onclick="api('POST','/fetch')">&#x2B07; Fetch CSV from float</button>
  <br><br>
  <a class="dl" href="/data" download>&#x2B73; Download CSV</a>
  <a class="dl" href="/plot" target="_blank">&#x1F4C8; Open plot</a>
</div>

<div class="plot-card">
  <h2 style="font-size:.9rem;text-transform:uppercase;letter-spacing:.05em;color:#555;margin:0 0 .75rem">Plot</h2>
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
      document.getElementById('local-packets').textContent = d.packets ?? '—';
      refreshPlot();
    }})
    .catch(e => msg(e.toString(), true));
}}

function floatApi(method, path) {{
  msg('...');
  fetch('/float' + path, {{method}})
    .then(r => r.json())
    .then(d => {{
      if (d.error) {{ msg(d.error, true); return; }}
      msg(JSON.stringify(d));
    }})
    .catch(e => msg(e.toString(), true));
}}

function refreshStatus() {{
  fetch('/float/status')
    .then(r => r.json())
    .then(d => {{
      document.getElementById('float-conn').textContent    = 'reachable';
      document.getElementById('float-conn').className      = 'pill ok';
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
      if (d.depth_m !== undefined)
        document.getElementById('float-depth').textContent =
          d.depth_m.toFixed(3) + ' m  (' + d.pressure_kpa.toFixed(1) + ' kPa)';
    }})
    .catch(() => {{}});
}}

function refreshPlot() {{
  const img = document.getElementById('plot');
  if (img) {{
    img.src = '/plot?t=' + Date.now();
  }} else {{
    fetch('/status' + '').then(() => {{
      fetch('/fetch').then(() => {{}}); // no-op check
    }}).catch(() => {{}});
  }}
}}

// bootstrap
refreshStatus();
setInterval(refreshStatus, 5000);
setInterval(refreshPlot, 30000);
</script>
</body>
</html>"""
    return Response(html, mimetype='text/html')


# ── proxy ─────────────────────────────────────────────────────────────────────

@app.route('/float', methods=['GET', 'POST'])
@app.route('/float/', methods=['GET', 'POST'])
@app.route('/float/<path:path>', methods=['GET', 'POST'])
def proxy(path=''):
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
    """Float pushes CSV here after recovery."""
    if 'file' in request.files:
        data = request.files['file'].read().decode()
    else:
        data = request.get_data(as_text=True)
    with open(DATA_PATH, 'w') as f:
        f.write(data)
    packets = max(0, data.count('\n') - 1)
    print(f"[receive] data from {request.remote_addr} — {packets} packets stored", flush=True)
    return jsonify({'status': 'received', 'packets': packets})


@app.route('/fetch', methods=['POST'])
def fetch():
    """Controller pulls CSV from float on demand."""
    try:
        r = req.get(f"{FLOAT_BASE}/data", timeout=10)
        if r.status_code != 200:
            return jsonify({'error': 'Float returned no data'}), 502
        with open(DATA_PATH, 'wb') as f:
            f.write(r.content)
        packets = max(0, r.text.count('\n') - 1)
        print(f"[fetch] pulled {packets} packets from float", flush=True)
        return jsonify({'status': 'fetched', 'packets': packets})
    except Exception as e:
        print(f"[fetch] failed: {e}", flush=True)
        return jsonify({'error': str(e)}), 503


# ── data serve ────────────────────────────────────────────────────────────────

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
