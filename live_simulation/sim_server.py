import time

from dataclasses import dataclass
import base64
from flask import Flask, request, jsonify

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
    "_t_at_pause": 0.0
}

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
    
    
    
    
                
            
        
    
        
        




    
            
    