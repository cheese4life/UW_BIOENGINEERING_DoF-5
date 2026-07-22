import time

from dataclasses import dataclass
import base64
from flask import Flask, request, jsonify, send_file

import numpy as np
import cv2

import threading
import io

import matplotlib
matplotlib.use("Agg", force=True)
import argparse
import sys
from pathlib import Path
import json 
import csv
import urllib.request
import urllib.error


app = Flask(__name__)

_lock = threading.Lock()

# dict
_state = {
    "playing": False,
    "fps": 30.0,
    "frame_idx": 0,
    "shift_px": 0.0,
    "sim_dir": None,       # Path
    "master_fps": 400,
    "n_frames": 0,
    "current_frame": None, # np.ndarray
    "all_shifts": None,    # np.ndarray
    "_t_start": 0.0,
    "_t_at_pause": 0.0,
    "last_focus_json": None,   # latency + error JSON from last /snapshot run
    "last_overlay_png": None,  # CV overlay PNG bytes from last /snapshot run
}

# cooldown timer to debounce LabVIEW button-hold (seconds)
_last_focus_time = 0.0
_FOCUS_COOLDOWN = 1.0

# autofocus webapp endpoint that runs CV detection + stage move + latency
WEBAPP_FOCUS_URL = "http://127.0.0.1:5000/focus_live"

# make sibling packages (cornea_focus, scripts) importable for local CV overlay
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# CV overlay geometry (mirror scripts/autofocus_latency_webapp.py)
_FOCUS_ROW = 150
_DZ_MM_PER_ROW = 0.004593


def _render_overlay_png(frame, surface_y=None, focus_row=None,
                        median_y=None, top_y=None, bottom_y=None,
                        valid=True) -> bytes:
    """Render an OCT frame + surface trace + bounding box + error text to raw
    PNG bytes. Mirrors _render_frame_to_png in the autofocus webapp so the
    overlay looks identical. Always returns valid PNG bytes."""
    import matplotlib.pyplot as plt

    h, w = frame.shape
    fmin, fmax = float(frame.min()), float(frame.max())
    img = ((frame - fmin) / (fmax - fmin + 1e-9) * 255).astype(np.uint8)

    dpi = 80
    fig, ax = plt.subplots(figsize=(w / dpi * 1.2, h / dpi), dpi=dpi)
    ax.imshow(img, cmap="gray", aspect="auto", extent=[0, w, h, 0])
    ax.set_xlim(0, w)
    ax.set_ylim(h, 0)

    if surface_y is not None and len(surface_y) > 0:
        sy = np.asarray(surface_y, dtype=float)
        xs = np.arange(w) + 0.5
        ax.plot(xs, sy, color="lime", linewidth=1.5)
        if top_y is not None and bottom_y is not None and valid:
            col0, col1 = xs[0], xs[-1]
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
                f"err={err:.1f} px ({err * _DZ_MM_PER_ROW * 1000:.1f} um)",
                color="white", fontsize=9, family="monospace",
                bbox=dict(facecolor="black", alpha=0.7))

    if not valid:
        ax.text(w / 2, h / 2, "INVALID", color="red", fontsize=20,
                ha="center", va="center", weight="bold",
                bbox=dict(facecolor="black", alpha=0.7))

    ax.axis("off")
    fig.tight_layout(pad=0)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight",
                pad_inches=0, facecolor="black")
    plt.close(fig)
    buf.seek(0)
    return buf.getvalue()


def _detect_and_render_overlay(frame) -> bytes:
    """Run CV surface detection on the current frame and render the overlay
    locally. ALWAYS returns valid PNG bytes: on any failure it still renders a
    frame carrying the INVALID marker, and as a last resort encodes the raw
    frame with cv2.imencode so the caller never receives non-PNG bytes."""
    try:
        from cornea_focus.surface import detect
        from cornea_focus.config import DetectorConfig
        det_cfg = DetectorConfig(mask_top_rows=10, blur_sigma=3,
                                 peak_prominence=10, smoothing_window=11)
        res = detect(frame.astype(np.float32), det_cfg)
        return _render_overlay_png(
            frame, surface_y=res.surface_y, focus_row=_FOCUS_ROW,
            median_y=res.median_y, top_y=res.top_y, bottom_y=res.bottom_y,
            valid=res.valid,
        )
    except Exception as exc:
        print("overlay detection failed:", repr(exc))
        try:
            return _render_overlay_png(frame, focus_row=_FOCUS_ROW, valid=False)
        except Exception as exc2:
            print("overlay render failed:", repr(exc2))
            fmin, fmax = float(frame.min()), float(frame.max())
            img8 = ((frame - fmin) / (fmax - fmin + 1e-9) * 255).astype(np.uint8)
            ok, enc = cv2.imencode(".png", img8)
            return enc.tobytes() if ok else b""


def _call_focus_webapp(img_b64: str) -> dict:
    """POST the snapshot frame to the autofocus webapp's /focus_live, which runs
    CV surface detection, moves the DOF stage, and measures latency. Returns the
    latency + error JSON dict (any overlay_b64 is discarded; sim_server renders
    its own overlay locally)."""
    payload = json.dumps({
        "img": img_b64,
        "velocity_mm_s": 125.0,
        "acceleration_mm_s2": 6000.0,
    }).encode("utf-8")
    req = urllib.request.Request(
        WEBAPP_FOCUS_URL, data=payload,
        headers={"Content-Type": "application/json"}, method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            body = resp.read().decode("utf-8")
    except urllib.error.HTTPError as e:
        # e.g. 422 invalid detection still carries a JSON body
        body = e.read().decode("utf-8")
    except Exception as exc:
        return {"status": "error", "message": f"focus_live failed: {exc!r}"}

    try:
        result = json.loads(body)
    except Exception:
        return {"status": "error", "message": "focus_live returned non-JSON"}

    result.pop("overlay_b64", None)  # overlay is produced locally instead
    return result


def _run_autofocus(img_b64: str, frame):
    """Run the full autofocus for one /snapshot: get the latency JSON from the
    webapp and render the CV overlay locally. Returns (json_dict, png_bytes)."""
    focus_json = _call_focus_webapp(img_b64)
    overlay_png = _detect_and_render_overlay(frame)
    return focus_json, overlay_png


def load_sim(sim_dir: Path) -> None:
    with _lock:
        # reading config
        with open(sim_dir / "config.json") as f:
            cfg = json.load(f)
        _state["master_fps"] = cfg["master_fps"]
        _state["sim_dir"] = sim_dir
        
        # read manifest for shift metadata
        shifts = []
        with open(sim_dir / "manifest.csv") as f:
            for row in csv.DictReader(f):
                shifts.append(float(row["shift_px"]))
        _state["all_shifts"] = np.array(shifts, dtype=np.float32)
        _state["n_frames"] = len(shifts)
        
        # pre loading first frame
        _state["frame_idx"] = 0
        _state["current_frame"] = np.load(str(sim_dir / "frame_000000.npy"))
        
@app.route("/")
def index():
    return """<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Simulation Engine</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#000;color:#fff;font-family:monospace;padding:16px}
h1{font-size:1.2em;margin-bottom:10px}
#oct{max-width:500px;border:1px solid #444;display:block;margin-bottom:8px}
.row{margin:4px 0}
input,button{background:#111;color:#fff;border:1px solid #555;padding:5px 10px;font-family:monospace;font-size:0.85em;cursor:pointer}
button:hover{background:#333}
input[type=range]{vertical-align:middle;cursor:pointer}
input[type=range]#scrubber{width:100%;max-width:500px;margin:4px 0}
#info{color:#aaa;font-size:0.8em;margin-top:6px}
</style></head><body>
<h1>SIMULATION ENGINE</h1>
<img id="oct" src="" alt="OCT preview">
<div class="row">
<input type="range" id="scrubber" min="0" max="100" value="0" oninput="seekScrubber(this.value)">
</div>
<div class="row">
<button id="btn-play" onclick="togglePlay()">▶</button>
<button onclick="stepFrame(-1)">◀◀</button>
<button onclick="stepFrame(1)">▶▶</button>
<span style="margin-left:12px">FPS: <span id="fps-val">30</span></span>
</div>
<div class="row">
<input type="range" id="fps-slider" min="10" max="400" value="30" oninput="setFPS(this.value)" style="max-width:200px">
</div>
<div id="info"></div>
<script>
let state={playing:false,fps:30,frame_idx:0,n_frames:0,master_fps:400};
function refresh(){
  fetch('/live_frame?t='+Date.now()).then(r=>r.json()).then(d=>{
    if(d.img_b64) document.getElementById('oct').src='data:image/png;base64,'+d.img_b64;
    let i=d.frame_idx,n=state.n_frames||1;
    document.getElementById('scrubber').max=n-1;
    document.getElementById('scrubber').value=i;
    document.getElementById('info').textContent=
      'Frame '+i+'/'+n+' | shift='+d.shift_px.toFixed(2)+'px | t='+d.time_s.toFixed(3)+'s';
  });
  fetch('/state?t='+Date.now()).then(r=>r.json()).then(s=>{
    state=s;
    document.getElementById('btn-play').textContent=s.playing?'⏸':'▶';
    document.getElementById('fps-val').textContent=s.fps;
    document.getElementById('fps-slider').value=s.fps;
    let n=s.n_frames||1;
    document.getElementById('scrubber').max=n-1;
    document.getElementById('scrubber').value=s.frame_idx;
  });
}
function togglePlay(){
  fetch('/set_params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({playing:!state.playing})});
}
function setFPS(v){
  document.getElementById('fps-val').textContent=v;
  fetch('/set_params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({fps:parseFloat(v)})});
}
function seekScrubber(val){
  fetch('/set_params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({seek_frame:parseInt(val)})});
}
function stepFrame(delta){
  let target=state.frame_idx+delta;
  if(target<0)target=0;
  if(target>=state.n_frames)target=state.n_frames-1;
  fetch('/set_params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({seek_frame:target})});
}
setInterval(refresh,200);
</script></body></html>"""
        
@app.route("/live_frame", methods=["GET"])
def live_frame():
    with _lock:
        frame = _state["current_frame"]
        idx = _state["frame_idx"]
        shift_px = _state["shift_px"]
        mfps = _state["master_fps"]
        
    if frame is None:
        return jsonify({"error":"No simulation loaded"}), 400
    
    t_sim = idx / mfps
    
    #frame renderer, png to base64
    
    import matplotlib.pyplot as plt
    
    h, w = frame.shape
    fmin, fmax = float(frame.min()), float(frame.max())
    img_u8 = ((frame - fmin) / (fmax-fmin + 1e-9) * 255).astype(np.uint8)
    
    fig, ax = plt.subplots(figsize=(w/80*1.2, h/80), dpi = 80)
    
    ax.imshow(img_u8, cmap = "gray", aspect = "auto")
    ax.axis("off")
    fig.tight_layout(pad=0)
    
    buf = io.BytesIO()
    
    fig.savefig(buf, format="png", bbox_inches="tight", pad_inches=0, facecolor="black")
    plt.close(fig)
    buf.seek(0)
    img_b64 = base64.b64encode(buf.read()).decode()
    
    
    return jsonify({
        "img_b64": img_b64,
        "frame_idx": idx,
        "time_s": round(t_sim, 6),
        "shift_px": round(shift_px, 4),
    })


@app.route("/live_frame_png", methods=["GET"])
def live_frame_png():
    with _lock:
        frame = _state["current_frame"]
        idx = _state["frame_idx"]

    if frame is None:
        return jsonify({"error": "No simulation loaded"}), 400

    import matplotlib.pyplot as plt

    h, w = frame.shape
    fmin, fmax = float(frame.min()), float(frame.max())
    img_u8 = ((frame - fmin) / (fmax - fmin + 1e-9) * 255).astype(np.uint8)

    fig, ax = plt.subplots(figsize=(w / 80 * 1.2, h / 80), dpi=80)
    ax.imshow(img_u8, cmap="gray", aspect="auto")
    ax.axis("off")
    fig.tight_layout(pad=0)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight",
                pad_inches=0, facecolor="black")
    plt.close(fig)
    buf.seek(0)

    return send_file(buf, mimetype="image/png")


@app.route("/state", methods=["GET"])
def state():
    with _lock:
        return jsonify({
            "playing": _state["playing"],
            "fps": _state["fps"],
            "frame_idx": _state["frame_idx"],
            "n_frames": _state["n_frames"],
            "master_fps": _state["master_fps"],
            "time_s": round(_state["frame_idx"] / _state["master_fps"], 6),
        })


@app.route("/snapshot", methods=["GET", "POST"])
def snapshot():
    """LabVIEW FOCUS → returns raw PNG for snapshot display. Also internally
    triggers the webapp's /focus_live on localhost. 1.0s cooldown."""
    global _last_focus_time

    now = time.monotonic()
    should_trigger_focus = now - _last_focus_time >= _FOCUS_COOLDOWN
    if should_trigger_focus:
        _last_focus_time = now

    with _lock:
        frame = _state["current_frame"]
        idx = _state["frame_idx"]
        shift_px = _state["shift_px"]
        mfps = _state["master_fps"]

    if frame is None:
        return jsonify({"error": "No simulation loaded"}), 400

    t_sim = idx / mfps

    import matplotlib.pyplot as plt

    h, w = frame.shape
    fmin, fmax = float(frame.min()), float(frame.max())
    img_u8 = ((frame - fmin) / (fmax - fmin + 1e-9) * 255).astype(np.uint8)

    fig, ax = plt.subplots(figsize=(w / 80 * 1.2, h / 80), dpi=80)
    ax.imshow(img_u8, cmap="gray", aspect="auto")
    ax.axis("off")
    fig.tight_layout(pad=0)

    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight",
                pad_inches=0, facecolor="black")
    plt.close(fig)
    buf.seek(0)

    # png_bytes must exist on every request (always return a valid PNG)
    buf.seek(0)
    png_bytes = buf.getvalue()

    # cooldown affects only autofocus trigger, never the returned snapshot.
    # When triggered we synchronously run the autofocus webapp's CV detection
    # + stage move, then cache the latency JSON and CV overlay PNG so LabVIEW
    # can fetch them from /snapshot_json and /snapshot_overlay. Both are
    # committed together under one lock so they always describe the same run.
    if should_trigger_focus:
        img_b64 = base64.b64encode(png_bytes).decode("ascii")
        focus_json, overlay_png = _run_autofocus(img_b64, frame)
        with _lock:
            _state["last_focus_json"] = focus_json
            _state["last_overlay_png"] = overlay_png

    response = send_file(
        io.BytesIO(png_bytes),
        mimetype="image/png",
        download_name="focus_snapshot.png",
    )
    response.headers["X-Frame-Index"] = str(idx)
    response.headers["X-Time-S"] = str(round(t_sim, 6))
    response.headers["X-Shift-Px"] = str(round(shift_px, 4))
    return response


@app.route("/snapshot_json", methods=["GET"])
def snapshot_json():
    """Return the latency + error-correction JSON from the most recent
    /snapshot autofocus run."""
    with _lock:
        data = _state.get("last_focus_json")
    if data is None:
        return jsonify({
            "status": "pending",
            "message": "no autofocus run yet; call /snapshot first",
        }), 200
    return jsonify(data)


@app.route("/snapshot_overlay", methods=["GET"])
def snapshot_overlay():
    """Return the CV overlay PNG (surface trace + bounding box + error text)
    from the most recent /snapshot autofocus run."""
    with _lock:
        png = _state.get("last_overlay_png")
    if not png:
        return jsonify({
            "status": "pending",
            "message": "no overlay yet; call /snapshot first",
        }), 404
    return send_file(io.BytesIO(png), mimetype="image/png",
                     download_name="focus_overlay.png")



@app.route("/set_params", methods=["POST"])
def set_params():
    data = request.get_json(silent=True) or {}
    
    
    with _lock:
        if "fps" in data:
            _state["fps"] = float(data["fps"])
        if "playing" in data:
            new_playing = bool(data["playing"])
            if new_playing and not _state["playing"]:
                # pause to play
                _state["_t_at_pause"] = _state["frame_idx"] / _state["master_fps"]
                _state["_t_start"] = time.monotonic()
            elif not new_playing and _state["playing"]:
                # play to pause
                elapsed = time.monotonic() - _state["_t_start"]
                _state["_t_at_pause"] += elapsed
            _state["playing"] = new_playing
        if "seek_frame" in data:
            # Jump to absolute frame index (pauses playback)
            seek_idx = int(data["seek_frame"])
            seek_idx = max(0, min(_state["n_frames"] - 1, seek_idx))
            _state["playing"] = False
            _state["_t_at_pause"] = seek_idx / _state["master_fps"]
            _state["_t_start"] = time.monotonic()
            fname = _state["sim_dir"] / f"frame_{seek_idx:06d}.npy"
            if fname.exists():
                _state["current_frame"] = np.load(str(fname))
                _state["frame_idx"] = seek_idx
                _state["shift_px"] = float(_state["all_shifts"][seek_idx])
    
    return jsonify({
        "ok": True
    })    
    
    

    
    
# timer thread somewhere over here

def _timer_loop():
    while True:
        with _lock:
            playing = _state["playing"]
            fps = _state["fps"]
            sim_dir = _state["sim_dir"]
            mfps = _state["master_fps"]
            n_frames = _state["n_frames"]
            
            if not playing or sim_dir is None or n_frames == 0:
                time.sleep(0.05)
                continue
            
            elapsed_real = time.monotonic() - _state["_t_start"]
            t_sim = _state["_t_at_pause"] + elapsed_real
            frame_idx = int(t_sim * mfps)
            
            if frame_idx >= n_frames:
                _state["playing"] = False
                continue

        with _lock:
            if frame_idx != _state["frame_idx"]:
                fname = sim_dir / f"frame_{frame_idx:06d}.npy"
                _state["current_frame"] = np.load(str(fname))
                _state["frame_idx"] = frame_idx
                _state["shift_px"] = float(_state["all_shifts"][frame_idx])
        
        
        time.sleep(0.005)
        
        
def main():
    t = threading.Thread(target=_timer_loop, daemon = True)
    t.start()
    
    parser = argparse.ArgumentParser(description="Patient Simulation Server")
    parser.add_argument("--sim-dir", type=str, default=None,
                        help="Path to pre-generated patient_sim directory")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Host to bind to")
    parser.add_argument("--port", type=int, default=5002,
                        help="Port to listen on")
    args = parser.parse_args()
    
    if args.sim_dir:
        load_sim(Path(args.sim_dir))
        print(f"Loaded {_state['n_frames']} frames from {args.sim_dir}")
        
    app.run(host=args.host, port=args.port)
    



if __name__ == "__main__" : main()
    
    
    
    
                
            
        
    
        
        




    
            
    