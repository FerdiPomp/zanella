from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


def current_day_stamp() -> str:
    return datetime.now().strftime("%Y%m%d")


def format_log(message: str) -> str:
    return f"[{current_day_stamp()}] {message}"


def print_log(message: str) -> None:
    print(format_log(message))


def describe_for_log(value) -> str:
    if hasattr(value, "type") and hasattr(value, "source"):
        return f"Event(source={value.source}, type={value.type}, date={current_day_stamp()})"
    if hasattr(value, "event_id") and hasattr(value, "workplace"):
        event_id = value.event_id.get("id")
        workplace_id = value.workplace.get("id")
        return f"EventXLayer(workplace={workplace_id}, event_id={event_id}, date={current_day_stamp()})"
    return str(value)


def ensure_dir(dir_name: str) -> Path:
    path = Path(dir_name)
    path.mkdir(parents=True, exist_ok=True)
    return path


def build_dated_path(dir_name: str, file_name: str, suffix: str) -> Path:
    directory = ensure_dir(dir_name)
    base_name = f"{current_day_stamp()}{file_name}"
    candidate = directory / f"{base_name}{suffix}"
    index = 1

    while candidate.exists():
        candidate = directory / f"{base_name}_{index}{suffix}"
        index += 1

    return candidate


def save_numpy_artifact(data, dir_name: str, file_name: str) -> Path:
    output_path = build_dated_path(dir_name, file_name, ".npy")
    np.save(output_path, data)
    return output_path


def save_jpg_artifact(image, dir_name: str, file_name: str) -> Path:
    output_path = build_dated_path(dir_name, file_name, ".jpg")
    if image.ndim == 3 and image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    cv2.imwrite(str(output_path), image)
    return output_path


def save_jpg_sequence(images: list, dir_name: str, file_name: str) -> Path:
    sequence_dir = build_dated_path(dir_name, file_name, "")
    sequence_dir.mkdir(parents=True, exist_ok=False)

    for index, image in enumerate(images):
        frame_path = sequence_dir / f"{current_day_stamp()}_{index:04d}.jpg"
        if image.ndim == 3 and image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        cv2.imwrite(str(frame_path), image)

    return sequence_dir
