"""
Phase 1 of manual VLM training-data collection: fly manually with keyboard,
save only the raw RGB scene frame + which key was held at that instant.
No depth inference here on purpose, so the control loop stays responsive.

Controls: W = forward, A = yaw left, D = yaw right, Up/Down arrows = ascend/descend,
Esc = land + quit. Up/Down are for maneuvering around the varied-altitude obstacles
only -- they don't get logged as training samples (the label schema only covers
left/right/both, not climb/descend).
Run process_flight_frames.py afterwards to turn these into labeled
InternVL3 training samples.
"""
import csv
import os
import time

import cosysairsim as airsim
import cv2
import numpy as np
from pynput import keyboard

SPEED = 3.0          # m/s, matches config.ini VELOCITY
YAW_RATE = 10.0       # deg/s, matches config.ini YAW_ANGLE
VERTICAL_SPEED = 2.0  # m/s, ascend/descend -- not tied to config.ini, collection-only
FLIGHT_HEIGHT = -8    # NED, matches config.ini FLIGHT_HEIGHT
CONTROL_HZ = 10

RAW_ROOT = r"C:\Users\User\internvl3-drone-finetune\data_collection\manual_flight_raw"
os.makedirs(RAW_ROOT, exist_ok=True)

pressed = set()


def on_press(key):
    if key == keyboard.Key.esc:
        return False
    pressed.add(key)


def on_release(key):
    pressed.discard(key)


def pressed_chars():
    return {k.char.lower() for k in pressed if getattr(k, "char", None)}


def current_action():
    chars = pressed_chars()
    if "a" in chars:
        return "a"
    if "d" in chars:
        return "d"
    if "w" in chars:
        return "w"
    return None


def current_vertical_speed():
    # Up/down are for maneuvering only -- never returned as a logged "action".
    if keyboard.Key.up in pressed:
        return -VERTICAL_SPEED  # NED: negative = ascend
    if keyboard.Key.down in pressed:
        return VERTICAL_SPEED
    return 0.0


def main():
    session_id = time.strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(RAW_ROOT, session_id)
    os.makedirs(session_dir, exist_ok=True)
    log_path = os.path.join(session_dir, "frames.csv")

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    client.moveToZAsync(FLIGHT_HEIGHT, SPEED).join()

    scene_req = airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    print("Manual data collection: W=forward, A=yaw left, D=yaw right, "
          "Up/Down=ascend/descend (not logged), Esc=land+quit")
    print(f"Saving raw frames to {session_dir}")

    with open(log_path, "w", newline="") as f:
        csv.writer(f).writerow(["frame", "filename", "key"])

    frame_idx = 0
    saved_count = 0
    try:
        while listener.running:
            action = current_action()
            vz = current_vertical_speed()

            vx, yaw_rate = 0.0, 0.0
            if action == "w":
                vx = SPEED
            elif action == "a":
                yaw_rate = -YAW_RATE
            elif action == "d":
                yaw_rate = YAW_RATE

            client.moveByVelocityBodyFrameAsync(
                vx, 0.0, vz, 1.0 / CONTROL_HZ,
                yaw_mode=airsim.YawMode(is_rate=True, yaw_or_rate=yaw_rate),
            ).join()

            if action is not None:
                response = client.simGetImages([scene_req])[0]
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                scene = img1d.reshape(response.height, response.width, 3)

                filename = f"{frame_idx:06d}_{action}.jpg"
                cv2.imwrite(os.path.join(session_dir, filename), scene)
                with open(log_path, "a", newline="") as f:
                    csv.writer(f).writerow([frame_idx, filename, action])
                saved_count += 1
                frame_idx += 1
                print(f"\rsaved {saved_count} frames (key={action})", end="", flush=True)
    finally:
        print(f"\nDone. {saved_count} raw frames saved to {session_dir}")
        client.hoverAsync().join()
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)


if __name__ == "__main__":
    main()
