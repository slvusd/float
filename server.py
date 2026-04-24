#!/usr/bin/env python3
import csv
import io
import json
import os
import subprocess
import sys
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from flask import Flask, jsonify, request, send_file

import depthdetect as depth_sensor
import actuator
import RPi.GPIO as GPIO
from config import (CALL_SIGN, TARGET_BOTTOM_M, TARGET_SURFACE_M,
                    BIAS_FILE, DATA_FILE)

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
    script = os.path.join(BASE_DIR, 'depthadjust.py')
    _mission_proc = subprocess.Popen([sys.executable, script], cwd=BASE_DIR)
    return jsonify({'status': 'mission started', 'pid': _mission_proc.pid})


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
    fig.savefig(buf, format='jpeg', dpi=150, quality=90)
    plt.close(fig)
    buf.seek(0)
    return send_file(buf, mimetype='image/jpeg')


# ── startup ───────────────────────────────────────────────────────────────────

GPIO.setwarnings(False)
actuator.setupActuator()
depth_sensor.initSensor()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
