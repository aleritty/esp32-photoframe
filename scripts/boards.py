import json
from pathlib import Path

_BOARDS_JSON = Path(__file__).resolve().parent.parent / "boards" / "boards.json"

with open(_BOARDS_JSON, encoding="utf-8") as _f:
    BOARDS = json.load(_f)

BOARDS_BY_ID = {b["id"]: b for b in BOARDS}

SUPPORTED_BOARDS = {b["id"]: f'{b["label"]} ({b["display"]})' for b in BOARDS}

BOARD_DIMENSIONS = {b["id"]: tuple(b["resolution"]) for b in BOARDS}

# Display color model per board: "gc16" (16-level grayscale), "bwry" (4-color
# black/white/red/yellow), "spectra6" (6-color, the default). Used to pick the
# dithering palette for splash generation, etc.
BOARD_DISPLAY_TYPE = {b["id"]: b.get("display_type", "spectra6") for b in BOARDS}

# ESP-IDF target chip per board. Almost every board is an ESP32-S3; the PicPak
# is an ESP32-C3. build.py passes this as -DIDF_TARGET and CI installs the
# matching toolchain.
BOARD_TARGET = {b["id"]: b.get("target", "esp32s3") for b in BOARDS}
