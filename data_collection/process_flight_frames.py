"""
Phase 2 of manual VLM training-data collection: batch-runs DepthAnythingV2
over raw RGB frames saved by collect_flight_frames.py, keeps only frames
where a front obstacle was actually detected (matching the live system's
fObjDetected condition), and appends them as labeled InternVL3 training
samples into the internvl3-drone-finetune repo.

Label mapping (confirmed with user):
  key 'a' (yaw left)  -> {"left": true,  "right": false, "both": false}
  key 'd' (yaw right) -> {"left": false, "right": true,  "both": false}
  key 'w' (forward)   -> {"left": true,  "right": true,  "both": true}

Usage: python process_flight_frames.py [session_dir]
       (defaults to the most recently modified session under manual_flight_raw/)
"""
import csv
import json
import os
import sys
import time

import cv2
import numpy as np
import torch
import torch.nn.functional as F
from torchvision.transforms import Compose

from depth_anything_v2.dpt import DepthAnythingV2
from depth_anything_v2.util.transform import NormalizeImage, PrepareForNet, Resize

REPO_ROOT = os.path.dirname(os.path.abspath(__file__))
RAW_ROOT = os.path.join(REPO_ROOT, "manual_flight_raw")

TRAINING_REPO = r"C:\Users\User\internvl3-drone-finetune"
TRAINING_IMAGES_DIR = os.path.join(TRAINING_REPO, "data", "images")
TRAINING_JSONL = os.path.join(TRAINING_REPO, "data", "train.jsonl")
TRAINING_META = os.path.join(TRAINING_REPO, "data", "meta.json")

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
D_HEIGHT, D_WIDTH = 144, 256
TAU = 4.0  # matches vlm_nav/config.ini TAU

PROMPT = (
    "Given the depthmap image from the front camera of a drone, where the darker area is closer, "
    "which direction should the drone fly towards to avoid collision? "
    'Respond with only a JSON object in the form {"left": <true|false>, "right": <true|false>, "both": <true|false>}, '
    "where true means flying that direction is a safe way to avoid the obstacle."
)

KEY_TO_LABEL = {
    "a": {"left": True, "right": False, "both": False},
    "d": {"left": False, "right": True, "both": False},
    "w": {"left": True, "right": True, "both": True},
}


def load_depth_model():
    transform = Compose([
        Resize(
            width=252, height=140, resize_target=False, keep_aspect_ratio=True,
            ensure_multiple_of=14, resize_method="lower_bound",
            image_interpolation_method=cv2.INTER_CUBIC,
        ),
        NormalizeImage(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        PrepareForNet(),
    ])
    model = DepthAnythingV2(encoder="vits", features=64, out_channels=[48, 96, 192, 384])
    checkpoint = os.path.join(REPO_ROOT, "checkpoints", "depth_anything_v2_vits.pth")
    model.load_state_dict(torch.load(checkpoint, map_location=DEVICE))
    model = model.to(DEVICE).eval()
    return model, transform


def estimate_depthmap(model, transform, scene):
    scene = scene / 255
    scene = transform({"image": scene})["image"]
    scene = torch.from_numpy(scene).unsqueeze(0).to(DEVICE)
    with torch.no_grad():
        depth = model(scene)
    depth = F.interpolate(depth[None], (D_HEIGHT, D_WIDTH), mode="bilinear", align_corners=False)[0, 0]
    return depth.cpu().numpy()


def compute_patch_means(depthmap, patch_shape=(40, 50)):
    h, w = depthmap.shape[-2], depthmap.shape[-1]
    patch_height, patch_width = patch_shape
    midpoint1 = patch_width // 2
    midpoint2 = w // 3
    midpoint3 = w // 2
    midpoint4 = midpoint2 + midpoint2
    midpoint5 = w - patch_width // 2
    means = []
    for midpoint in (midpoint1, midpoint2, midpoint3, midpoint4, midpoint5):
        start_y = (h - patch_height) // 2
        start_x = midpoint - (patch_width // 2)
        rect = depthmap[..., start_y:start_y + patch_height, start_x:start_x + patch_width]
        means.append(float(np.mean(rect)))
    return means


def find_latest_session():
    sessions = [os.path.join(RAW_ROOT, d) for d in os.listdir(RAW_ROOT)
                if os.path.isdir(os.path.join(RAW_ROOT, d))]
    if not sessions:
        raise RuntimeError(f"No sessions found under {RAW_ROOT}")
    return max(sessions, key=os.path.getmtime)


def next_sample_index():
    if not os.path.exists(TRAINING_JSONL):
        return 0
    with open(TRAINING_JSONL, encoding="utf-8") as f:
        return sum(1 for _ in f)


def update_meta_length(total):
    with open(TRAINING_META, encoding="utf-8") as f:
        meta = json.load(f)
    meta["drone_vlm_direction"]["length"] = total
    with open(TRAINING_META, "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)


def main():
    session_dir = sys.argv[1] if len(sys.argv) > 1 else find_latest_session()
    print(f"Processing session: {session_dir}")

    processed_marker = os.path.join(session_dir, ".processed")
    if os.path.exists(processed_marker):
        print(f"This session was already processed on {open(processed_marker).read().strip()}.")
        answer = input("Process it again anyway and add duplicate samples? [y/N] ")
        if answer.strip().lower() != "y":
            print("Aborted.")
            return

    os.makedirs(TRAINING_IMAGES_DIR, exist_ok=True)

    model, transform = load_depth_model()

    with open(os.path.join(session_dir, "frames.csv"), newline="") as f:
        rows = list(csv.DictReader(f))

    sample_idx = next_sample_index()
    kept_counts = {"a": 0, "d": 0, "w": 0}
    total_seen = 0

    with open(TRAINING_JSONL, "a", encoding="utf-8") as out_f:
        for row in rows:
            total_seen += 1
            scene = cv2.imread(os.path.join(session_dir, row["filename"]))
            if scene is None:
                continue

            depth = estimate_depthmap(model, transform, scene)
            p1, p2, p3, p4, p5 = compute_patch_means(depth)
            f_obj_detected = (p2 > TAU) or (p3 > TAU) or (p4 > TAU)

            print(f"\r{total_seen}/{len(rows)} processed, {sum(kept_counts.values())} kept "
                  f"(a={kept_counts['a']} d={kept_counts['d']} w={kept_counts['w']})", end="", flush=True)

            if not f_obj_detected:
                continue

            key = row["key"]
            label = KEY_TO_LABEL[key]

            depth_norm = (depth - depth.min()) / (depth.max() - depth.min()) * 255
            depth_inv = 255 - depth_norm
            image = cv2.cvtColor(depth_inv.astype(np.uint8), cv2.COLOR_GRAY2BGR)

            image_filename = f"sample_{sample_idx:06d}.jpg"
            cv2.imwrite(os.path.join(TRAINING_IMAGES_DIR, image_filename), image)

            sample = {
                "id": f"sample_{sample_idx:06d}",
                "image": f"images/{image_filename}",
                "conversations": [
                    {"from": "human", "value": f"<image>\n{PROMPT}"},
                    {"from": "gpt", "value": json.dumps(label)},
                ],
            }
            out_f.write(json.dumps(sample) + "\n")
            kept_counts[key] += 1
            sample_idx += 1

    print()
    update_meta_length(sample_idx)
    with open(processed_marker, "w") as f:
        f.write(time.strftime("%Y-%m-%d %H:%M:%S"))
    print(f"Done. Kept {sum(kept_counts.values())}/{total_seen} frames "
          f"(a={kept_counts['a']} d={kept_counts['d']} w={kept_counts['w']}). "
          f"Total training samples now: {sample_idx}")


if __name__ == "__main__":
    main()
