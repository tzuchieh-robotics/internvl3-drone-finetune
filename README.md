# InternVL3-1B LoRA fine-tune for drone obstacle-direction feedback

Vendored training code from [OpenGVLab/InternVL](https://github.com/OpenGVLab/InternVL)
(`internvl_chat`), trimmed to just what's needed to LoRA fine-tune InternVL3-1B for one
task: given a depth-map image, output `{"left": bool, "right": bool, "both": bool}`
indicating which direction is safe to fly.

## Layout
- `internvl/` — core training package copied from upstream (`train/`, `model/`, `patch/`, `conversation.py`, `dist_utils.py`)
- `shell/finetune_lora_1b.sh` — LoRA fine-tune entrypoint, adapted from upstream's InternVL2.5-1B LoRA script
- `tools/merge_lora.py` — merges the trained LoRA adapter back into a standalone checkpoint
- `zero_stage1_config.json` / `zero_stage3_config.json` — DeepSpeed configs used by the training script
- `data/meta.json` — dataset registry read by the training script
- `data/images/` — put training images here
- `data/train.jsonl` — one JSON object per line (see format below)
- `data_collection/` — AirSim (Blocks) manual data collection pipeline: fly with keyboard
  (`collect_flight_frames.py`), batch-label with DepthAnythingV2 (`process_flight_frames.py`),
  and spawn a dense obstacle field to fly through (`spawn_obstacles.py`, see
  `data_collection/OBSTACLE_CONFIG.md` for the drone/obstacle size reference and current settings)

## Data format
Each line of `data/train.jsonl`:
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
Image preprocessing must match inference exactly: normalize depth to 0-255, invert (closer = darker), convert to RGB.
Update `length` in `data/meta.json` to match the number of lines in `train.jsonl`.

## Training
```bash
pip install -r requirements.txt
bash shell/finetune_lora_1b.sh
```
Note: `requirements.txt` pins `transformers==4.37.2` as upstream specifies for this training
codebase — this is older than what we used for inference-only testing (4.48.0), so use a
separate virtualenv from the main vlm_nav environment to avoid clobbering it.

## After training
```bash
python tools/merge_lora.py <path-to-lora-checkpoint> <path-to-merged-output>
```
