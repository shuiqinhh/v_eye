"""Load pre-built map database exported by ORB-SLAM3 node."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAGIC = b"VEYEMAP1"
DESCRIPTOR_SIZE = 32


@dataclass
class MapDatabase:
    points: np.ndarray
    descriptors: np.ndarray

    @property
    def size(self) -> int:
        return int(self.points.shape[0])


def load_map_database(path: str | Path) -> MapDatabase:
    path = Path(path)
    data = path.read_bytes()
    if len(data) < 12 or data[:8] != MAGIC:
        raise ValueError(f"Invalid map database magic: {path}")

    count = int.from_bytes(data[8:12], byteorder="little", signed=False)
    expected = 12 + count * (12 + DESCRIPTOR_SIZE)
    if len(data) != expected:
        raise ValueError(
            f"Corrupt map database {path}: expected {expected} bytes, got {len(data)}"
        )

    points = np.empty((count, 3), dtype=np.float32)
    descriptors = np.empty((count, DESCRIPTOR_SIZE), dtype=np.uint8)
    offset = 12
    for i in range(count):
        points[i, 0] = np.frombuffer(data, dtype=np.float32, count=1, offset=offset)[0]
        offset += 4
        points[i, 1] = np.frombuffer(data, dtype=np.float32, count=1, offset=offset)[0]
        offset += 4
        points[i, 2] = np.frombuffer(data, dtype=np.float32, count=1, offset=offset)[0]
        offset += 4
        descriptors[i] = np.frombuffer(
            data, dtype=np.uint8, count=DESCRIPTOR_SIZE, offset=offset
        )
        offset += DESCRIPTOR_SIZE

    return MapDatabase(points=points, descriptors=descriptors)
