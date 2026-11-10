#!/usr/bin/env python3
"""Run real TUM RGB-D A/B ablations against the backed-up pre-change executable.
Usage: python3 tools/validate_dynamic_switches.py [dataset] [frames]
"""
import csv
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs/dynamic-switch"
DATA = Path(sys.argv[1] if len(sys.argv) > 1 else "/home/aaa/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz")
COUNT = int(sys.argv[2]) if len(sys.argv) > 2 else 100
BASE = (ROOT / "config/TUM3-instance-association.yaml").read_text()

def setting(text, key, value):
    pattern = r"(?m)^" + re.escape(key) + r":.*$"
    line = key + ": " + str(value)
    return re.sub(pattern, line, text) if re.search(pattern, text) else text + "\n" + line + "\n"

def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))

summary = {}
for name, enable, mask_only, count, resize in [
    ("baseline", 0, 1, COUNT, False),
    ("A", 0, 1, COUNT, False),
    ("B", 1, 1, COUNT, False),
    ("missing", None, None, 5, False),
    ("reserved", 1, 0, 5, False),
    ("resized", 1, 1, 5, True),
]:
    folder = OUT / name
    folder.mkdir(parents=True, exist_ok=True)
    debug = folder / "debug"
    # Fresh output prevents stale logs from being mistaken for evidence.
    assert not debug.exists(), str(debug) + " already exists; preserve it before rerunning"
    text = BASE
    for key in ("Dynamic.Enable", "Dynamic.MaskOnly"):
        text = re.sub(r"(?m)^" + re.escape(key) + r":.*\n?", "", text)
    values = {
        "Run.Viewer": 0,
        # Deliberately conflicting legacy switches prove master precedence.
        "SemanticMask.Enabled": 0 if name == "baseline" else 1,
        "SemanticMask.MaskingEnabled": 1,
        "SemanticMask.AssociationEnabled": 0 if name == "baseline" else 1,
        "SemanticMask.AssociationDebug": 0 if name == "baseline" else 1,
        "SemanticMask.Debug": 1,
        "SemanticMask.OutputDirectory": '"' + str(debug) + '"',
    }
    if enable is not None:
        values.update({"Dynamic.Enable": enable, "Dynamic.MaskOnly": mask_only})
    if not enable:
        # Disabled and missing-key modes must run without loading any ONNX file.
        values["SemanticMask.Model"] = '"/nonexistent/dynamic-switch-test.onnx"'
    if resize:
        values.update({"Camera.newWidth": 480, "Camera.newHeight": 360})
    for key, value in values.items():
        text = setting(text, key, value)
    config = folder / "settings.yaml"
    config.write_text(text)
    binary = ROOT / "Examples/RGB-D/rgbd_tum"
    env = os.environ.copy()
    env.pop("DISPLAY", None)
    if name == "baseline":
        binary = OUT / "pre-change/rgbd_tum"
        env["LD_LIBRARY_PATH"] = str(OUT / "pre-change") + ":" + env.get("LD_LIBRARY_PATH", "")
    argv = [str(binary), str(ROOT / "Vocabulary/ORBvoc.txt"), str(config), str(DATA), str(DATA / "associate.txt"), str(folder), str(count)]
    with (folder / "run.log").open("w") as log:
        subprocess.run(argv, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=240)
    runs = read(folder / "RunFrames.csv")
    assert len(runs) == count and all(r["state"] == "2" for r in runs), name
    trajectory = np.loadtxt(folder / "CameraTrajectory.txt", ndmin=2)
    assert trajectory.shape == (count, 8) and np.isfinite(trajectory).all(), name
    assert (folder / "KeyFrameTrajectory.txt").stat().st_size > 0
    log_text = (folder / "run.log").read_text()
    result = {"frames": count, "tracking_states": sorted({r["state"] for r in runs}),
              "trajectory_rows": len(trajectory), "mean_track_ms": sum(float(r["track_ms"]) for r in runs) / count,
              "keypoints": [int(r["keypoints"]) for r in runs]}
    if not enable:
        assert not debug.exists(), name
        assert "SemanticMask tensor" not in log_text and "[InstanceAssociation]" not in log_text
        result.update(model_path_nonexistent=True, model_loaded=False, association=False)
    else:
        assert "SemanticMask tensor images" in log_text
        assert log_text.count("[InstanceAssociation]") == count
        frames = read(debug / "association_frames.csv")
        keys = read(debug / "association_features.csv")
        tables = read(debug / "association_instances.csv")
        inference = read(debug / "frames.csv")
        assert len(frames) == len(inference) == count
        assert all(float(r["infer_ms"]) > 0 and int(r["rejected"]) == 0 and int(r["violations"]) == 0 for r in inference)
        indexed = {}
        instances = {}
        for k in keys:
            indexed.setdefault(k["frame_id"], []).append(k)
        for t in tables:
            instances.setdefault(t["frame_id"], {})[int(t["instance_id"])] = t
        inside = background = 0
        for f in frames:
            fid = f["frame_id"]
            n = int(f["total_features"])
            features = indexed[fid]
            assert n == len(features) == int(f["features_before"]) == int(f["features_after"]) == int(f["instance_labels_length"])
            assert n == sum(int(f[k]) for k in ("background_features", "instance_features", "invalid_features"))
            assert [int(k["feature_index"]) for k in features] == list(range(n))
            labels = np.asarray(Image.open(debug / ("association_" + fid + "_labels.png")))
            assert labels.shape == ((360, 480) if resize else (480, 640))
            table = instances.get(fid, {})
            assert set(map(int, np.unique(labels))) - {0} <= set(table)
            overlay = Image.open(debug / ("association_" + fid + "_features.png"))
            assert overlay.size == (labels.shape[1], labels.shape[0])
            for k in features:
                assert k["coordinate_space"] == "slam_rgb_raw_pixels"
                u, v = float(k["u"]), float(k["v"])
                expected = int(labels[int(np.floor(v)), int(np.floor(u))]) if np.isfinite(u) and np.isfinite(v) and 0 <= u < labels.shape[1] and 0 <= v < labels.shape[0] else -1
                assert int(k["instance_id"]) == expected
                if expected > 0:
                    inside += 1
                elif expected == 0:
                    background += 1
            for instance_id, t in table.items():
                assert int(t["features"]) == sum(int(k["instance_id"]) == instance_id for k in features)
        assert inside > 0 and background > 0
        result.update(inference_frames=count, association_frames=count, rejected=0,
                      all_labels_match=True, unchanged_feature_counts=True,
                      instance_features=inside, background_features=background)
    summary[name] = result
    print(name, {k: v for k, v in result.items() if k != "keypoints"}, flush=True)

for name in ("A", "B", "missing", "reserved"):
    n = summary[name]["frames"]
    assert summary[name]["keypoints"] == summary["baseline"]["keypoints"][:n], name + " changed extraction counts"
    summary[name]["counts_match_pre_change_baseline"] = True
(OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print("PASS: all dynamic switch checks", flush=True)
