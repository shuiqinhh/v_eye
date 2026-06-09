"""Load camera intrinsics from ORB-SLAM3 YAML settings."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float = 0.0
    k2: float = 0.0
    p1: float = 0.0
    p2: float = 0.0
    k3: float = 0.0
    width: int = 0
    height: int = 0

    @property
    def camera_matrix(self):
        import numpy as np

        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def dist_coeffs(self):
        import numpy as np

        return np.array([self.k1, self.k2, self.p1, self.p2, self.k3], dtype=np.float64)


def load_camera_yaml(yaml_path: str | Path) -> CameraIntrinsics:
    path = Path(yaml_path)
    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    return CameraIntrinsics(
        fx=float(data["Camera.fx"]),
        fy=float(data["Camera.fy"]),
        cx=float(data["Camera.cx"]),
        cy=float(data["Camera.cy"]),
        k1=float(data.get("Camera.k1", 0.0)),
        k2=float(data.get("Camera.k2", 0.0)),
        p1=float(data.get("Camera.p1", 0.0)),
        p2=float(data.get("Camera.p2", 0.0)),
        k3=float(data.get("Camera.k3", 0.0)),
        width=int(data.get("Camera.width", 0)),
        height=int(data.get("Camera.height", 0)),
    )
