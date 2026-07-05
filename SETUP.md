# Setup notes for training on a different server

Repo: https://github.com/tzuchieh-robotics/internvl3-drone-finetune

This repo is vendored InternVL LoRA fine-tuning code, trimmed down to fine-tune
InternVL3-1B on depth-map images to predict `{"left": bool, "right": bool, "both": bool}`
obstacle-direction safety judgments. It is not fully zero-touch after `git clone` —
the following adjustments are needed before training will actually run.

## 1. Install dependencies
```bash
pip install -r requirements.txt
```
Pins `transformers==4.37.2` (older version required by this training codebase).
Use a separate virtualenv/conda env from any other project to avoid version conflicts.

## 2. Add training data
- Put training images in `data/images/`
- Put annotations in `data/train.jsonl`, one JSON object per line:
```json
{
  "id": "sample_000123",
  "image": "images/000123.jpg",
  "conversations": [
    {"from": "human", "value": "<image>\nGiven the depthmap image from the front camera of a drone, where the darker area is closer, which direction should the drone fly towards to avoid collision? Respond with only a JSON object in the form {\"left\": <true|false>, \"right\": <true|false>, \"both\": <true|false>}, where true means flying that direction is a safe way to avoid the obstacle."},
    {"from": "gpt", "value": "{\"left\": false, \"right\": true, \"both\": false}"}
  ]
}
```
Image preprocessing must match inference exactly: normalize depth to 0-255, invert
(closer = darker), convert to RGB.

## 3. Update `data/meta.json`
The `"length"` field is currently a placeholder (`0`). Set it to the actual number
of lines in `data/train.jsonl` before training.

## 4. Base model download
`shell/finetune_lora_1b.sh` points at `OpenGVLab/InternVL3-1B`, which auto-downloads
from Hugging Face on first run. The server needs internet access for this (or
pre-download the model into the HF cache beforehand if the training node is offline).

## 5. GPU / batch size settings
`shell/finetune_lora_1b.sh` reads these as environment variables (defaults are for
a single GPU):
```bash
GPUS=4 BATCH_SIZE=64 PER_DEVICE_BATCH_SIZE=8 bash shell/finetune_lora_1b.sh
```
Adjust to match the actual GPU count / VRAM on the target server.

## 6. DeepSpeed / OS notes
The training script uses DeepSpeed (`zero_stage1_config.json`). This is expected to
work more smoothly on Linux than on Windows — if the target server is Linux (typical
for rented cloud GPU boxes), fewer environment issues are expected than we hit while
setting up the Windows dev machine this repo was assembled on.

## After training
```bash
python tools/merge_lora.py <path-to-lora-checkpoint> <path-to-merged-output>
```
