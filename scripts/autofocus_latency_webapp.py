#!/usr/bin/env python3
"""
# 6/17/26
# latency webapp to track how long it takes for DOF stage to 
# react to a focus command based on random distances & set
# distances.
"""
from __future__ import annotations

import can
import sys
import time
import argparse
from dataclasses import dataclass
import struct
import random
import io
import base64
import numpy as np
import cv2

from flask import Flask, request, jsonify

app = Flask(__name__)

_bus = None

# --- CV mode state ---
_cv_frame: np.ndarray | None = None          # current (possibly shifted) frame
_cv_shift_px: int = 0                         # pixels shifted
_cv_surface: dict | None = None               # last SurfaceResult as dict
_cv_ref: np.ndarray | None = None             # reference cornea image (DC-cleaned)
_cv_bg: np.ndarray | None = None              # oversized background canvas
_cv_max_shift: int = 200                       # max shift range in pixels
_cv_baseline_median_y: float = 0.0             # median_y of un-shifted frame
_focus_row: int = 150
_dz_mm_per_row: float = 0.004593


# --- CV one-time init (called from main()) ---
def _init_cv(sample: str = "cornea_1"):
    """Load reference image (sample = 'cornea_1'..'cornea_4'), apply DC fix,
    build background canvas. Mirrors cornea_focus/generate_sim.py. Also detects
    the baseline median_y."""
    import cv2
    global _cv_ref, _cv_bg, _cv_max_shift, _cv_baseline_median_y, _cv_frame, _cv_shift_px, _cv_surface
    from pathlib import Path as _P

    fname = f"{sample}.npy" if not sample.endswith(".npy") else sample
    ref = np.load(str(_P(__file__).resolve().parent.parent / "data" / "samples" / fname)).astype(np.float32)
    H, W = ref.shape

    # Suppress DC artifact in top rows (match generate_sim.py)
    DC_ROWS, SB = 4, (5, 25)
    clean = ref[SB[0]:SB[1]]
    cm, cs = float(clean.mean()), float(clean.std())
    rng = np.random.default_rng(1)
    ref[:DC_ROWS] = np.clip(rng.normal(cm, cs, size=(DC_ROWS, W)), 0.0, None).astype(ref.dtype)

    _cv_ref = ref
    _cv_max_shift = 200

    # Build oversized background canvas (match generate_sim.py)
    STRIP = 20
    top_s, bot_s = ref[:STRIP], ref[-STRIP:]
    tm, ts = float(top_s.mean()), float(top_s.std())
    bm, bs = float(bot_s.mean()), float(bot_s.std())
    TH = H + 2 * _cv_max_shift
    blend = np.linspace(0.0, 1.0, TH)[:, None]
    mp = (1.0 - blend) * tm + blend * bm
    sp = (1.0 - blend) * ts + blend * bs
    rng2 = np.random.default_rng(0)
    _cv_bg = np.clip(rng2.normal(0.0, 1.0, size=(TH, W)) * sp + mp, 0.0, None).astype(ref.dtype)

    # Baseline detection on un-shifted frame (for shift sanity check)
    from cornea_focus.surface import detect
    from cornea_focus.config import DetectorConfig
    det_cfg = DetectorConfig(mask_top_rows=10, blur_sigma=3, peak_prominence=10, smoothing_window=11)
    try:
        _cv_baseline_median_y = float(detect(ref, det_cfg).median_y)
    except Exception:
        _cv_baseline_median_y = float(_focus_row)
    _cv_frame = ref.copy()  # show unshifted frame immediately
    _cv_shift_px = 0
    _cv_surface = None
    print(f"[cv] loaded {sample}  baseline median_y={_cv_baseline_median_y:.1f}  focus_row={_focus_row}")


# website code

# --- CV rendering helper (inline matplotlib, no pyplot) ---
def _render_frame_to_png(frame: np.ndarray, surface_y=None, focus_row=None,
                          median_y=None, top_y=None, bottom_y=None,
                          valid: bool = True) -> str:
    """Render an OCT frame + surface trace + bounding box → base64 PNG."""
    import matplotlib
    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    h, w = frame.shape
    fmin, fmax = float(frame.min()), float(frame.max())
    img = ((frame - fmin) / (fmax - fmin + 1e-9) * 255).astype(np.uint8)

    dpi = 80
    fig, ax = plt.subplots(figsize=(w/dpi*1.2, h/dpi), dpi=dpi)
    ax.imshow(img, cmap="gray", aspect="auto", extent=[0, w, h, 0])
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)

    if surface_y is not None and len(surface_y) > 0:
        sy = np.asarray(surface_y, dtype=float)
        xs = np.arange(w) + 0.5
        # lime surface trace (matches play_sim_with_dof.py)
        ax.plot(xs, sy, color="lime", linewidth=1.5)
        # bounding box around detected cornea span
        if top_y is not None and bottom_y is not None and valid:
            col0 = xs[0]; col1 = xs[-1]
            rect = plt.Rectangle((col0, top_y), col1 - col0, bottom_y - top_y,
                                  linewidth=2, edgecolor="cyan",
                                  facecolor="none", linestyle="-")
            ax.add_patch(rect)
            ax.text(5, max(2, top_y - 8),
                    f"box: rows {top_y:.0f}-{bottom_y:.0f}",
                    color="cyan", fontsize=8, family="monospace",
                    bbox=dict(facecolor="black", alpha=0.7))

    if focus_row is not None:
        ax.axhline(focus_row, color="white", linestyle="--", linewidth=1)

    if median_y is not None:
        ax.axhline(median_y, color="lime", linewidth=1.2)
        err = median_y - focus_row if focus_row else 0
        ax.text(5, focus_row + 5 if focus_row else 10,
                f"err={err:.1f} px ({err * _dz_mm_per_row * 1000:.1f} µm)",
                color="white", fontsize=9, family="monospace",
                bbox=dict(facecolor="black", alpha=0.7))
        if not valid:
            ax.text(w/2, h/2, "INVALID", color="red", fontsize=20,
                    ha="center", va="center", weight="bold",
                    bbox=dict(facecolor="black", alpha=0.7))

    ax.axis("off")
    fig.tight_layout(pad=0)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0, facecolor="black")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode()


# live image API
# accept base64 PNG 
@app.route("/focus_live", methods=["POST"])
def focus_live():
    t_arrival_ns = time.perf_counter_ns()
    global _bus, _cv_surface
    if _bus is None:
        return jsonify({"status": "error", "message": "no stage (--no-stage mode)"}), 503
    data = request.get_json(silent=True) or {}

    # accept either "img" or "image_b64" as the image key
    img_b64 = data.get("img") or data.get("image_b64")
    if not img_b64:
        return jsonify({"status": "error", "message": "missing img/image_b64"}), 400
    vel = data.get("velocity_mm_s", 10.0)
    acc = data.get("acceleration_mm_s2", 100.0)
    direction = data.get("direction", 0)
    
    # header removal(?)
    if "," in img_b64:
        img_b64 = img_b64.split(",", 1)[1]
    
    img_bytes = base64.b64decode(img_b64)
    
    img_buffer = np.frombuffer(img_bytes, dtype=np.uint8)
    
    img = cv2.imdecode(img_buffer, cv2.IMREAD_GRAYSCALE)

    if img is None:
        return jsonify({
            "status": "error",
            "message": "Image could not be decoded",
        }), 400

    img = img.astype(np.float32)
    
    if img is None:
        raise ValueError("Failed to decode image")
    
    
    # recycled from /FOCUS
    
    # -----
    
    dof_init.set_motion_params(_bus, vel_mm_s = vel, acc_mm_s2 = acc)
    
    cv_us = 0.0
    overlay_b64 = None
    if img is not None:
        # CV mode: measure detection time, then use detected error
        from cornea_focus.surface import detect
        from cornea_focus.config import DetectorConfig
        det_cfg = DetectorConfig(mask_top_rows=10, blur_sigma=3,
                                 peak_prominence=10, smoothing_window=11)
        t0 = time.perf_counter_ns()
        res = detect(img, det_cfg)
        t1 = time.perf_counter_ns()
        cv_us = (t1 - t0) / 1000.0
        _cv_surface = {
            "median_y": round(float(res.median_y), 2),
            "top_y": round(float(res.top_y), 2),
            "bottom_y": round(float(res.bottom_y), 2),
            "surface_y": np.round(res.surface_y, 1).tolist(),
            "valid": res.valid,
        }
        # Render the CV overlay (surface trace + bounding box + error text)
        # on the incoming snapshot frame so callers can display it.
        overlay_b64 = _render_frame_to_png(
            img, surface_y=res.surface_y, focus_row=_focus_row,
            median_y=res.median_y, top_y=res.top_y, bottom_y=res.bottom_y,
            valid=res.valid,
        )
        if not res.valid:
            return jsonify({
                "status": "error",
                "message": "Focus detection was invalid",
                "overlay_b64": overlay_b64,
            }), 422

        # CV error already has a sign via (median_y - focus_row). Stage
        # moves OPPOSITE the error (control.py sign convention):
        #   positive error (cornea below focus) → stage moves up (negative)
        cv_error_um = (res.median_y - _focus_row) * _dz_mm_per_row * 1000.0
        error_um_actual = abs(round(cv_error_um, 1))
        direction = -1 if cv_error_um > 0 else 1
        report_extra = {
            "cv_detected_median_y": round(float(res.median_y), 1),
            "cv_focus_row": _focus_row,
            "cv_error_px": round(float(res.median_y - _focus_row), 1),
            "cv_baseline_median_y": round(_cv_baseline_median_y, 1),
            "cv_shift_px": _cv_shift_px,
            "cv_detected_shift_px": round(float(res.median_y - _cv_baseline_median_y), 1),
        }
    else:
        error_um_actual = error_um

    home_counts = dof_init.get_pos_counts(_bus)

    
    
    target_counts = home_counts + direction * int(error_um_actual * 200)

    vel_mode = data.get("vel_mode", "classical")
    poll_vel = (vel_mode == "polled")
    result, trace, vel_polled = run_trial(_bus, target_counts, home_counts, poll_velocity=poll_vel)
    report = build_report(result, error_um_actual, direction, home_counts, target_counts,
                      trace[-1][1] if trace else home_counts, trace,
                      vel_mode=vel_mode, vel_polled=vel_polled)
    if cv_us > 0:
        report["cv_ms"] = round(cv_us / 1000.0, 2)
        report.update(report_extra)
    if overlay_b64 is not None:
        report["overlay_b64"] = overlay_b64
    report["t_arrival_ns"] = t_arrival_ns
    return jsonify(report)

    # ----
    





@app.route("/cv_frame")
def cv_frame():
    """Return current CV frame as base64 PNG with overlay."""
    if _cv_frame is None:
        return jsonify({"error": "no frame loaded. POST /shift_image first."}), 400
    img_b64 = _render_frame_to_png(
        _cv_frame,
        surface_y=_cv_surface.get("surface_y") if _cv_surface else None,
        focus_row=_focus_row,
        median_y=_cv_surface.get("median_y") if _cv_surface else None,
        top_y=_cv_surface.get("top_y") if _cv_surface else None,
        bottom_y=_cv_surface.get("bottom_y") if _cv_surface else None,
        valid=_cv_surface.get("valid", False) if _cv_surface else False,
    )
    return jsonify({
        "img_b64": img_b64,
        "shift_px": _cv_shift_px,
        "median_y": _cv_surface.get("median_y") if _cv_surface else None,
        "error_um": round((_cv_surface["median_y"] - _focus_row) * _dz_mm_per_row * 1000, 1) if _cv_surface and _cv_surface.get("valid") else None,
    })


@app.route("/set_cv_sample", methods=["POST"])
def set_cv_sample():
    """Load a different cornea sample."""
    global _cv_frame, _cv_shift_px, _cv_surface
    data = request.get_json() or {}
    sample = data.get("sample", "cornea_1")
    if sample not in ("cornea_1", "cornea_2", "cornea_3", "cornea_4"):
        return jsonify({"error": f"invalid sample {sample}"}), 400
    _init_cv(sample)
    return jsonify({"sample": sample, "baseline_median_y": _cv_baseline_median_y})


@app.route("/shift_image", methods=["POST"])
def shift_image():
    """Shift cornea image using warpAffine + background fill (mirrors generate_sim.py)."""
    import cv2
    global _cv_frame, _cv_shift_px, _cv_surface

    if _cv_ref is None or _cv_bg is None:
        _init_cv()

    data = request.get_json() or {}
    mode = data.get("mode", "random")
    px = data.get("shift_px")

    H, W = _cv_ref.shape
    margin = 40

    if mode == "predefined" and px is not None:
        shift = max(-_focus_row + margin, min(_cv_max_shift, int(px)))
    else:
        # random shift: use benchmark distances converted to pixel equivalents
        candidates = [int(d * 1000 / 4.593) for d in [0.05, 0.1, 0.25, 0.5, 1.0]]  # mm → px
        candidates = [c for c in candidates if margin <= c <= _cv_max_shift]
        candidates += [80, 50, 30]  # fallback pixel candidates
        shift = random.choice(candidates) if candidates else _cv_max_shift

    # warpAffine shift (exactly like generate_sim.py)
    M = np.float32([[1, 0, 0], [0, 1, shift]])
    shifted = cv2.warpAffine(_cv_ref, M, (W, H),
                             borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    # Fill exposed rows with background (match generate_sim.py)
    bg_top = _cv_max_shift - shift
    bg_patch = _cv_bg[bg_top:bg_top + H]
    if shift > 0:
        shifted[:shift] = bg_patch[:shift]
    elif shift < 0:
        shifted[H + shift:] = bg_patch[H + shift:]

    _cv_frame = shifted
    _cv_shift_px = shift

    # NOTE: NO detection here. Detection happens inside /focus's timed window
    # so cv_ms is a real "from click to usable error" measurement.
    _cv_surface = None

    return jsonify({"shift_px": shift})

@app.route("/focus", methods=["POST"])
def focus():
    global _bus, _cv_surface
    if _bus is None:
        return jsonify({"status": "error", "message": "no stage (--no-stage mode)"}), 503
    t_arrival_ns = time.perf_counter_ns()
    data = request.get_json(silent=True) or {}
    
    focus_mode = data.get("focus_mode", "simple")   
    mode = data.get("mode", "predefined")
    error_um = data.get("error_um", 100.0)
    vel = data.get("velocity_mm_s", 10.0)
    acc = data.get("acceleration_mm_s2", 100.0)
    direction = data.get("direction", 0)
    
    dof_init.set_motion_params(_bus, vel_mm_s = vel, acc_mm_s2 = acc)
    
    cv_us = 0.0
    if focus_mode == "cv" and _cv_frame is not None:
        # CV mode will measure detection time, then use detected error
        from cornea_focus.surface import detect
        from cornea_focus.config import DetectorConfig
        det_cfg = DetectorConfig(mask_top_rows=10, blur_sigma=3,
                                 peak_prominence=10, smoothing_window=11)
        t0 = time.perf_counter_ns()
        res = detect(_cv_frame, det_cfg)
        t1 = time.perf_counter_ns()
        cv_us = (t1 - t0) / 1000.0
        _cv_surface = {
            "median_y": round(float(res.median_y), 2),
            "top_y": round(float(res.top_y), 2),
            "bottom_y": round(float(res.bottom_y), 2),
            "surface_y": np.round(res.surface_y, 1).tolist(),
            "valid": res.valid,
        }
        if res.valid:

            cv_error_um = (res.median_y - _focus_row) * _dz_mm_per_row * 1000.0
            error_um_actual = abs(round(cv_error_um, 1))
            direction = -1 if cv_error_um > 0 else 1
            report_extra = {
                "cv_detected_median_y": round(float(res.median_y), 1),
                "cv_focus_row": _focus_row,
                "cv_error_px": round(float(res.median_y - _focus_row), 1),
                "cv_baseline_median_y": round(_cv_baseline_median_y, 1),
                "cv_shift_px": _cv_shift_px,
                "cv_detected_shift_px": round(float(res.median_y - _cv_baseline_median_y), 1),
            }
        else:
            error_um_actual = error_um
            report_extra = {}
    else:
        error_um_actual = error_um

    home_counts = dof_init.get_pos_counts(_bus)
    # Note: in CV mode we already set error_um_actual + direction above.
    # In simple mode, generate_error picks the direction.
    if focus_mode != "cv":
        error_um_actual, direction = generate_error(mode, error_um_actual, direction, home_counts)
    
    target_counts = home_counts + direction * int(error_um_actual * 200)

    vel_mode = data.get("vel_mode", "classical")
    poll_vel = (vel_mode == "polled")
    result, trace, vel_polled = run_trial(_bus, target_counts, home_counts, poll_velocity=poll_vel)
    report = build_report(result, error_um_actual, direction, home_counts, target_counts,
                      trace[-1][1] if trace else home_counts, trace,
                      vel_mode=vel_mode, vel_polled=vel_polled)
    if cv_us > 0:
        report["cv_ms"] = round(cv_us / 1000.0, 2)
        report.update(report_extra)
    report["t_arrival_ns"] = t_arrival_ns
    return jsonify(report)

@app.route("/")
def index():
    return """<!DOCTYPE html>
<html>
<head>
<style>
body{background:#000;color:#0f0;font-family:monospace;margin:0;padding:10px}
#main{display:flex;gap:20px}
#left{flex:1;min-width:400px}
#right{flex:1;min-width:350px}
#cv-img{max-width:100%;border:1px solid #0f0;display:none}
#cv-error{display:none;font-size:1.2em;margin:5px 0}
#error-row{display:block}
#shift-row{display:none}
input{background:#111;color:#0f0;border:1px solid #0f0;padding:4px;margin:2px;font-family:monospace}
button{background:#0a0a0a;color:#0f0;border:2px solid #0f0;padding:8px 16px;font-family:monospace;cursor:pointer}
button:hover{background:#0f0;color:#000}
select{background:#111;color:#0f0;border:1px solid #0f0;padding:4px;font-family:monospace}
</style>
</head>
<body>
<h1>AUTOFOCUS</h1>
<div id="main">
<div id="left">
<label>Mode:</label>
<select id="focus-mode" onchange="toggleMode()">
<option value="simple">Simple</option><option value="cv">Computer Vision</option>
</select><br><br>
<label>Velocity (mm/s):</label><br>
<input type="number" id="vel" value="10"><br>
<label>Acceleration (mm/s²):</label><br>
<input type="number" id="acc" value="100"><br>
<label>Vel source:</label>
<select id="vel-mode">
<option value="classical">Classical (Δp/Δt)</option>
<option value="polled">Polled (Juno 0xAD)</option>
</select><br>
<div id="error-row">
<label>Error (µm):</label><br>
<input type="number" id="error" value="100"><br>
<label><input type="checkbox" id="randomize"> Randomize</label><br>
</div>
<div id="shift-row">
<label>Sample:</label>
<select id="cv-sample" onchange="setCVSample()">
<option value="cornea_1">cornea_1</option>
<option value="cornea_2">cornea_2</option>
<option value="cornea_3">cornea_3</option>
<option value="cornea_4">cornea_4</option>
</select><br>
<label>Shift:</label>
<select id="shift-mode"><option value="random">Random</option><option value="predefined">Predefined</option></select>
<input type="number" id="shift-px" value="50" style="width:70px"> px
<button onclick="shiftImage()">Shift Image</button>
<div id="cv-error"></div>
</div>
<br>
<div id="total-ms" style="font-size:3em;border:2px solid #0f0;padding:10px;margin:10px 0;display:inline-block">-- ms</div>
<div id="cv-latency" style="font-size:1.2em;margin:5px 0;display:none"></div>
<br>
<div id="vel-p25" style="font-size:1.5em;border:1px solid #0f0;padding:5px;margin:3px;display:inline-block">p25: --</div>
<div id="vel-p50" style="font-size:1.5em;border:1px solid #0f0;padding:5px;margin:3px;display:inline-block">p50: --</div>
<div id="vel-p75" style="font-size:1.5em;border:1px solid #0f0;padding:5px;margin:3px;display:inline-block">p75: --</div>
<div id="vel-peak" style="font-size:1.5em;border:1px solid #0f0;padding:5px;margin:3px;display:inline-block">peak: --</div>
<br>
<canvas id="vel-canvas" width="600" height="200" style="border:1px solid #0f0;margin:10px 0;"></canvas>
<br>
<button onclick="focusClick()" style="font-size:2em;padding:20px;">FOCUS</button>
<pre id="report"></pre>
</div>
<div id="right">
<img id="cv-img" src="" alt="OCT frame">
</div>
</div>
<script>
function toggleMode() {
    let cv = document.getElementById("focus-mode").value === "cv";
    document.getElementById("error-row").style.display = cv ? "none" : "block";
    document.getElementById("shift-row").style.display = cv ? "block" : "none";
    document.getElementById("cv-img").style.display = cv ? "block" : "none";
    document.getElementById("cv-latency").style.display = cv ? "block" : "none";
    if (cv) refreshCVFrame();
}
async function setCVSample() {
    let sample = document.getElementById("cv-sample").value;
    await fetch("/set_cv_sample", {
        method:"POST", headers:{"Content-Type":"application/json"},
        body: JSON.stringify({sample: sample})
    });
    document.getElementById("cv-error").textContent = "Sample changed. Click FOCUS to detect.";
    refreshCVFrame();
}
async function shiftImage() {
    let sm = document.getElementById("shift-mode").value;
    let px = sm === "predefined" ? parseInt(document.getElementById("shift-px").value) : null;
    let resp = await fetch("/shift_image", {
        method:"POST", headers:{"Content-Type":"application/json"},
        body: JSON.stringify({mode: sm, shift_px: px})
    });
    let data = await resp.json();
    document.getElementById("cv-error").textContent =
        "Shifted " + data.shift_px + " px. Click FOCUS to detect.";
    // Show raw frame only — no overlay until FOCUS.
    refreshCVFrame();
}
async function refreshCVFrame() {
    let resp = await fetch("/cv_frame?t=" + Date.now());
    let data = await resp.json();
    if (data.img_b64) {
        document.getElementById("cv-img").src = "data:image/png;base64," + data.img_b64;
    }
    if (data.error_um != null) {
        document.getElementById("cv-error").textContent =
            "Detected error: " + data.error_um.toFixed(1) + " µm" +
            (data.valid === false ? " (INVALID)" : "");
    }
}
async function focusClick() {
    let focusMode = document.getElementById("focus-mode").value;
    let req = {
        focus_mode: focusMode,
        mode: document.getElementById("randomize").checked ? "random" : "predefined",
        error_um: parseFloat(document.getElementById("error").value),
        velocity_mm_s: parseFloat(document.getElementById("vel").value),
        acceleration_mm_s2: parseFloat(document.getElementById("acc").value),
        vel_mode: document.getElementById("vel-mode").value,
        direction: 0
    };
    let resp = await fetch("/focus", {
        method:"POST", headers:{"Content-Type":"application/json"},
        body: JSON.stringify(req)
    });
    let data = await resp.json();
    document.getElementById("report").textContent = JSON.stringify(data, null, 2);
    document.getElementById("total-ms").textContent = (data.phases_us.total_us / 1000).toFixed(1) + " ms";
    if (data.cv_ms) {
        document.getElementById("cv-latency").textContent =
            "CV detection: " + data.cv_ms.toFixed(2) + " ms";
    }
    document.getElementById("vel-p25").textContent = "p25: " + data.velocity_mm_s.p25 + " mm/s";
    document.getElementById("vel-p50").textContent = "p50: " + data.velocity_mm_s.p50 + " mm/s";
    document.getElementById("vel-p75").textContent = "p75: " + data.velocity_mm_s.p75 + " mm/s";
    document.getElementById("vel-peak").textContent = "peak: " + data.velocity_mm_s.peak + " mm/s";
    let rawTrace = drawGraph(data.vel_trace);
    if (rawTrace && rawTrace.length > 0) {
        // Stats from raw trace, filtering out dead-time/settle zeroes
        let moving = rawTrace.map(p => p[1]).filter(v => v >= 0.5).sort((a,b) => a-b);
        if (moving.length === 0) moving = rawTrace.map(p => p[1]).sort((a,b) => a-b);
        let [p25, p50, p75, peak] = percentiles(moving);
        document.getElementById("vel-p25").textContent = "p25: " + p25.toFixed(1) + " mm/s";
        document.getElementById("vel-p50").textContent = "p50: " + p50.toFixed(1) + " mm/s";
        document.getElementById("vel-p75").textContent = "p75: " + p75.toFixed(1) + " mm/s";
        document.getElementById("vel-peak").textContent = "peak: " + peak.toFixed(1) + " mm/s";
    }
    if (focusMode === "cv") refreshCVFrame();
}
function drawGraph(trace) {
    let canvas = document.getElementById("vel-canvas");
    let ctx = canvas.getContext("2d");
    let w = canvas.width, h = canvas.height;
    if (!trace || trace.length < 2) { ctx.clearRect(0,0,w,h); return []; }
    // Causal 5-point moving average
    let smoothed = [];
    for (let i = 0; i < trace.length; i++) {
        let sum = 0, count = 0;
        for (let j = Math.max(0, i - 4); j <= i; j++) { sum += trace[j][1]; count++; }
        smoothed.push([trace[i][0], sum / count]);
    }
    trace = smoothed;
    canvas._trace = trace;

    // Find min/max for signed velocity — zero line always centered
    let tMin = trace[0][0], tMax = trace[trace.length-1][0];
    let vMin = 0, vMax = 0;
    for (let p of trace) {
        if (p[1] > vMax) vMax = p[1];
        if (p[1] < vMin) vMin = p[1];
    }
    let absMax = Math.max(vMax, -vMin) * 1.1 || 1;
    vMin = -absMax; vMax = absMax;
    let zeroY = h - 25;  // y=0 maps to the horizontal axis
    if (vMin < 0) zeroY = (h - 25) - (0 - vMin) / (vMax - vMin) * (h - 40);

    function redraw(mx, my) {
        ctx.clearRect(0, 0, w, h);
        // axes
        ctx.strokeStyle = "#0f0"; ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(40, 10); ctx.lineTo(40, h-25); ctx.lineTo(w-10, h-25);
        ctx.stroke();
        // zero line
        ctx.strokeStyle = "#333"; ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(40, zeroY); ctx.lineTo(w-10, zeroY); ctx.stroke();
        ctx.setLineDash([]);
        // labels
        ctx.font = "10px monospace"; ctx.fillStyle = "#0f0";
        ctx.fillText("0", 30, zeroY + 4);
        ctx.fillText(vMax.toFixed(0) + " mm/s", 2, 15);
        if (vMin < -0.1) ctx.fillText(vMin.toFixed(0) + " mm/s", 2, h-28);
        ctx.fillText(tMin.toFixed(0) + " ms", 40, h-5);
        ctx.fillText(tMax.toFixed(0) + " ms", w-40, h-5);

        // Velocity curve — green for forward, red for reverse
        ctx.lineWidth = 2;
        for (let pass = 0; pass < 2; pass++) {
            ctx.strokeStyle = pass === 0 ? "#0f0" : "#f44";
            ctx.beginPath();
            let started = false;
            for (let i = 0; i < trace.length; i++) {
                let isPos = trace[i][1] >= 0;
                if ((pass === 0 && !isPos) || (pass === 1 && isPos)) continue;
                let x = 40 + (trace[i][0] - tMin) / (tMax - tMin) * (w - 60);
                let y = (h - 25) - (trace[i][1] - vMin) / (vMax - vMin) * (h - 40);
                if (!started) { ctx.moveTo(x, y); started = true; }
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }
        // hover crosshair
        if (mx >= 40 && mx <= w-10 && my >= 10 && my <= h-25) {
            let idx = Math.round((mx - 40) / (w - 60) * (trace.length - 1));
            idx = Math.max(0, Math.min(trace.length-1, idx));
            let px = 40 + (trace[idx][0] - tMin) / (tMax - tMin) * (w - 60);
            let py = (h - 25) - (trace[idx][1] - vMin) / (vMax - vMin) * (h - 40);
            ctx.strokeStyle = "#fff"; ctx.lineWidth = 1;
            ctx.beginPath(); ctx.arc(px, py, 4, 0, Math.PI*2); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(px, 10); ctx.lineTo(px, h-25); ctx.stroke();
            let tip = trace[idx][0].toFixed(1) + " ms, " + trace[idx][1].toFixed(1) + " mm/s";
            let tw = ctx.measureText(tip).width + 8;
            let tx = px + 10, ty = py - 12;
            if (tx + tw > w-10) tx = px - tw - 10;
            if (ty < 10) ty = py + 15;
            ctx.fillStyle = "#000"; ctx.fillRect(tx-2, ty-10, tw, 18);
            ctx.fillStyle = "#0f0"; ctx.fillText(tip, tx, ty+2);
        }
    }
    canvas.onmousemove = function(e) {
        let rect = canvas.getBoundingClientRect();
        let scaleX = w / rect.width;
        let scaleY = h / rect.height;
        let mx = (e.clientX - rect.left) * scaleX;
        let my = (e.clientY - rect.top) * scaleY;
        redraw(mx, my);
    };
    canvas.onmouseleave = function() { redraw(-1,-1); };
    redraw(-1, -1);
    return trace;
}
function percentiles(sorted) {
    let n = sorted.length;
    if (n === 0) return [0,0,0,0];
    return [sorted[Math.floor(n*0.25)], sorted[Math.floor(n*0.5)],
            sorted[Math.floor(n*0.75)], sorted[n-1]];
}
</script>
</body>
</html>"""


# dof init
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from scripts import dof_init

# detection constants (mirror latency/dof_latency_bench.py)
NOISE_COUNTS = 3             # ±15 nm — move detected
ENGAGE_BAND_COUNTS = 50      # ±250 nm — near target
SETTLE_BAND_COUNTS = 3       # ±15 nm — arrival window
SETTLE_HOLD_S = 0.005        # must stay in band 5 ms before "complete"
TIMEOUT_S = 0.5              # per-move safety timeout

DISTANCES_UM = [10, 25, 50, 100, 200]

# user end
@dataclass
class FocusRequest:
    mode: "random" | "predefined"
    error_um: float | None
    velocity_mm_s: float
    acceleration_mm_s2: float
    direction: 1 | -1 | 0

# stage end
@dataclass
class FocusResponse:   
    error_um: float
    direction: int
    target_mm: float
    home_mm: float
    final_mm: float
    final_error_nm: float
    events: TrialEvents
    phases_us: TrialPhases
    
    status: "complete" | "timeout" | "error"
    message: str

# events comms
@dataclass
class TrialEvents:
    t_cmd_ns: int
    t_react_ns: int
    t_engage_ns: int
    t_complete_ns: int
    
@dataclass
class TrialResult:
    t_cmd_ns: int | None = None
    t_react_ns: int | None = None
    t_engage_ns: int | None = None
    t_complete_ns: int | None = None
    
# phase comms
@dataclass
class TrialPhases:
    receive_us: float
    execute_us: float
    finish_us: float
    total_us: float
    
def run_trial(bus, target_counts, home_counts, poll_velocity=False):
    t_cmd = time.perf_counter_ns()
    
    dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", target_counts))
    dof_init.sr(bus, dof_init.OP_UPDATE)

    
    # event variables init
    t_react = None
    t_engage = None
    t_complete = None
    in_settle_band_since = None
    trace = []
    vel_polled = [] if poll_velocity else None
    
    while True:
        curr_position = dof_init.get_pos_counts(bus)
        t_curr = time.perf_counter_ns()

        # Optionally poll Juno velocity alongside position
        if poll_velocity:
            v_juno = dof_init.get_velocity_mm_s(bus) / 65536
            vel_polled.append((t_curr - t_cmd, v_juno))
        
        
        if(t_react == None and abs(curr_position - home_counts) > NOISE_COUNTS):
            t_react = t_curr
            
        if(t_engage == None and t_react != None
           and abs(curr_position - target_counts) <= ENGAGE_BAND_COUNTS):
            t_engage = t_curr
            
        # trace logic here
        # used for velocity calculations
        trace.append((t_curr - t_cmd, curr_position))
            
        # within three counts of target
        if abs(curr_position - target_counts) <= SETTLE_BAND_COUNTS:
            
            # starting timer to see if we meet threshold
            if in_settle_band_since is None:
                in_settle_band_since = t_curr       # first time in band
            
            # met threshold
            elif (t_curr - in_settle_band_since) / 1e9 >= SETTLE_HOLD_S:
                t_complete = t_curr
                break
        # fell out of timer, try again
        else:
            in_settle_band_since = None 
            
        # timeout safety
        if(t_curr - t_cmd) / 1e9 > TIMEOUT_S:
            break
        
    result = TrialResult()
    result.t_cmd_ns = t_cmd
    result.t_react_ns = t_react
    result.t_engage_ns = t_engage
    result.t_complete_ns = t_complete
    return result, trace, vel_polled

def generate_error(mode, error_um, direction, current_counts):
    if(mode == "random"):
        ran_range = len(DISTANCES_UM)
        num = random.randint(0, ran_range - 1)
        error = DISTANCES_UM[num]
        
    else:
        error = error_um
        
    if(direction == 0):
        direction = random.choice([-1, 1])
    
    return error, direction
    
    
def build_report(result, error_um, direction, home_counts, target_counts,
                 final_counts, trace, vel_mode="classical",
                 vel_polled=None):
    target_mm = target_counts / dof_init.COUNTS_PER_MM
    home_mm = home_counts / dof_init.COUNTS_PER_MM
    final_mm = final_counts / dof_init.COUNTS_PER_MM
    final_error_nm = abs(final_counts - target_counts) * 5

    receive_us = 0.0
    execute_us = 0.0
    finish_us = 0.0
    total_us = 0.0

    if result.t_react_ns is not None and result.t_cmd_ns is not None:
        receive_us = (result.t_react_ns - result.t_cmd_ns) / 1000.0
    if result.t_engage_ns is not None and result.t_react_ns is not None:
        execute_us = (result.t_engage_ns - result.t_react_ns) / 1000.0
    if result.t_complete_ns is not None and result.t_engage_ns is not None:
        finish_us = (result.t_complete_ns - result.t_engage_ns) / 1000.0
    if result.t_complete_ns is not None and result.t_cmd_ns is not None:
        total_us = (result.t_complete_ns - result.t_cmd_ns) / 1000.0

    # status + message
    if result.t_complete_ns is not None:
        status = "complete"
        message = ""
    else:
        status = "timeout"
        message = "trial timed out"

    # velocity from trace (motion phase only: t_react → t_engage)
    velocities = []
    if result.t_react_ns is not None and result.t_engage_ns is not None:
        motion_start = result.t_react_ns - result.t_cmd_ns
        motion_end = result.t_engage_ns - result.t_cmd_ns
        for i in range(1, len(trace)):
            t_rel = trace[i][0]
            if t_rel < motion_start or t_rel > motion_end:
                continue
            dt_ns = trace[i][0] - trace[i-1][0]
            dp_counts = abs(trace[i][1] - trace[i-1][1])
            if dt_ns > 0:
                velocities.append((dp_counts / dof_init.COUNTS_PER_MM) / (dt_ns / 1e9))
                
    
    
    velocities.sort()
    n = len(velocities)
    vel_25 = velocities[n // 4] if n > 0 else 0 # default to zero if the velocities list is empty
    vel_50 = velocities[n // 2] if n > 0 else 0
    vel_75 = velocities[3 * n // 4] if n > 0 else 0
    vel_peak = max(velocities) if velocities else 0

    vel_trace = []
    vel_trace.append([0.0, 0.0])

    if vel_mode == "polled" and vel_polled and len(vel_polled) >= 2:
        for t_rel, v in vel_polled:
            vel_trace.append([round(t_rel / 1e6, 3), round(v, 2)])
        if result.t_complete_ns is not None:
            t_comp_ms = (result.t_complete_ns - result.t_cmd_ns) / 1e6
            vel_trace.append([round(t_comp_ms, 3), 0.0])
    else:
        # classical: Δposition/Δtime from trace
        if result.t_react_ns is not None:
            t_react_ms = (result.t_react_ns - result.t_cmd_ns) / 1e6
            vel_trace.append([round(t_react_ms, 3), 0.0])
        if len(trace) >= 2:
            for i in range(1, len(trace)):
                dt_ns = trace[i][0] - trace[i-1][0]
                dp_counts = abs(trace[i][1] - trace[i-1][1])
                if dt_ns > 0:
                    t_ms = trace[i][0] / 1e6
                    v = (dp_counts / dof_init.COUNTS_PER_MM) / (dt_ns / 1e9)
                    vel_trace.append([round(t_ms, 3), round(v, 2)])
        if result.t_complete_ns is not None:
            t_comp_ms = (result.t_complete_ns - result.t_cmd_ns) / 1e6
            vel_trace.append([round(t_comp_ms, 3), 0.0])

    return {
        "error_um": error_um,
        "direction": direction,
        "target_mm": target_mm,
        "home_mm": home_mm,
        "final_mm": final_mm,
        "final_error_nm": final_error_nm,
        "events": {
            "t_cmd_ns": result.t_cmd_ns,
            "t_react_ns": result.t_react_ns,
            "t_engage_ns": result.t_engage_ns,
            "t_complete_ns": result.t_complete_ns,
        },
        "phases_us": {
            "receive_us": receive_us,
            "execute_us": execute_us,
            "finish_us": finish_us,
            "total_us": total_us,
        },
        # flat ms fields for LabVIEW — no nesting, no computation
        "receive_ms": round(receive_us / 1000.0, 2),
        "execute_ms": round(execute_us / 1000.0, 2),
        "finish_ms": round(finish_us / 1000.0, 2),
        "total_ms": round(total_us / 1000.0, 2),
        # flat us fields for LabVIEW (top-level, no nesting)
        "receive_us": round(receive_us, 2),
        "execute_us": round(execute_us, 2),
        "finish_us": round(finish_us, 2),
        "total_us": round(total_us, 2),
        # flat velocity percentiles for LabVIEW (top-level)
        "vel_p25": round(vel_25, 2),
        "vel_p50": round(vel_50, 2),
        "vel_p75": round(vel_75, 2),
        "vel_peak": round(vel_peak, 2),
        "velocity_mm_s": {
            "p25": round(vel_25, 2),
            "p50": round(vel_50, 2),
            "p75": round(vel_75, 2),
            "peak": round(vel_peak, 2),
        },
        "vel_trace": vel_trace,
        "status": status,
        "message": message,
    }
    
    
    
    
    


def main():
    parser = argparse.ArgumentParser(
        description="autofocus latency webapp"
    )
    
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host addy to bind to"
    )
    
    parser.add_argument(
        "--port",
        type=int,
        default=5000,
        help="Port to listen on"
    )
    
    parser.add_argument(
        "--channel",
        type=str,
        default="can0",
        help="DOF comms channel via CAN"
    )
    
    parser.add_argument(
        "--self-test",
        default=False,
        action="store_true",
        help="self test moving 50um to confirm expected outcomes"
    )
    
    parser.add_argument(
        "--latency-test",
        default=False,
        action="store_true",
        help="self test for latency in movement"
    )

    parser.add_argument(
        "--trial",
        type=int,
        default=None,
        metavar="UM",
        help="run one trial of N um and print the full report (Phase 2 checkpoint)"
    )

    parser.add_argument(
        "--no-stage",
        default=False,
        action="store_true",
        help="start Flask without connecting to the DOF stage (for UI dev)"
    )
        
    args = parser.parse_args()

    print(f"host: {args.host}")
    print(f"port: {args.port}")

    cli_mode = args.self_test or args.latency_test or args.trial is not None

    if args.no_stage:
        print("[no-stage] skipping CAN init — server only, no hardware")
        bus = None
    else:
        bus = can.interface.Bus(channel=args.channel, interface="socketcan",
                                bitrate=1_000_000)
        try:
            dof_init.init_drive(bus)
        except Exception as e:
            print(f"Error initializing stage: {e}")
            bus.shutdown() if bus else None
            raise SystemExit(1)
    try:

        if cli_mode:
            # self test here
            if args.self_test:  
                dof_init.set_motion_params(bus, vel_mm_s=125.0, acc_mm_s2=6000.0)
                
                home_pos = dof_init.get_pos_counts(bus)
                print(f"Pre-Check")
                print(f"Home Position: {home_pos}")
                
                # move target 50um
                target = (4000 * 200) + home_pos
                
                dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", target))
                dof_init.sr(bus, dof_init.OP_UPDATE)
                
                time.sleep(0.2)
                
                arrival = dof_init.get_pos_counts(bus)
                print(f"Arrived at: {arrival}")
                
                dof_init.sr(bus, dof_init.OP_SET_POSITION, struct.pack(">i", home_pos))
                dof_init.sr(bus, dof_init.OP_UPDATE)
                
                time.sleep(0.2)
                
                homed = dof_init.get_pos_counts(bus)
                print(f"Homed at: {homed}")
                
                if(abs(homed - home_pos) <= 3):
                    print("PRE CHECK PASSED")
                else:
                    print("PRE CHECK FAILED")
            
            if args.latency_test:
                dof_init.set_motion_params(bus, vel_mm_s=125.0, acc_mm_s2=6000.0)
                home_pos = dof_init.get_pos_counts(bus)
                print(f"Home position: {home_pos}")

                target = (4000 * 200) + home_pos
                print(f"MOVING {target / 200000:.3f} MM")

                result, _, _ = run_trial(bus, target, home_pos)
                if result.t_complete_ns is not None:
                    print(f"  react:  {result.t_react_ns - result.t_cmd_ns if result.t_react_ns else 0} ns")
                    print(f"  engage: {result.t_engage_ns - result.t_react_ns if result.t_engage_ns and result.t_react_ns else 0} ns")
                    print(f"  settle: {result.t_complete_ns - result.t_engage_ns if result.t_engage_ns else 0} ns")
                    print(f"  total:  {result.t_complete_ns - result.t_cmd_ns} ns")
                else:
                    print("  TRIAL TIMED OUT")

                # move back home
                result2, _, _ = run_trial(bus, home_pos, target)
                if result2.t_complete_ns is not None:
                    print(f"  return total: {result2.t_complete_ns - result2.t_cmd_ns} ns")
                else:
                    print("  RETURN TIMED OUT")

            if args.trial is not None:
                dof_init.set_motion_params(bus, vel_mm_s=125.0, acc_mm_s2=6000.0)
                home_pos = dof_init.get_pos_counts(bus)
                target = home_pos + int(args.trial * 200)
                print(f"[trial] {args.trial}um move: home={home_pos}  target={target}")
                result, _, _ = run_trial(bus, target, home_pos)
                if result.t_complete_ns is not None:
                    print(f"  react_us:  {(result.t_react_ns - result.t_cmd_ns) / 1000:.1f}")
                    print(f"  motion_us: {(result.t_engage_ns - result.t_react_ns) / 1000:.1f}")
                    print(f"  settle_us: {(result.t_complete_ns - result.t_engage_ns) / 1000:.1f}")
                    print(f"  total_us:  {(result.t_complete_ns - result.t_cmd_ns) / 1000:.1f}")
                else:
                    print("  TIMEOUT")
                # return home
                _, _, _ = run_trial(bus, home_pos, target)

        else:
            global _bus
            _bus = bus
            _init_cv()
            app.run(host=args.host, port=args.port)

        
    except KeyboardInterrupt:
        print("\n[bench] interrupted by user")
        
    except Exception as e:
        print("Error initializing stage")
        print(f"Error: {e}")
                
    finally:
        try:
            if bus is not None:
                bus.shutdown()
        except Exception:
            pass
    
    

if __name__ == "__main__":
    main()






