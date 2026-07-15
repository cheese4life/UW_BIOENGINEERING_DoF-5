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
    
    
@app.route("/state", methods=("GET"))
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
    

@app.route("/set_params", methods=("POST"))
def set_params():
    data = request.get_json(silent=True) or {}
    
    
    
    
    
# timer thread somewhere over here

    
            
    