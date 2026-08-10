import json
import math
from pathlib import Path

import yaml


PARAMS_PATH = Path(__file__).resolve().parents[1] / "config" / "task1_params.yaml"


def test_task1monday_layout_uses_gps1_enu_origin_and_surveyed_buoys():
    params = yaml.safe_load(PARAMS_PATH.read_text(encoding="utf-8"))[
        "task1_orchestrator"]["ros__parameters"]
    checkpoints = json.loads(params["gps_checkpoint_xy"])
    buoys = json.loads(params["buoy_position_xy"])

    assert checkpoints[0] == [0.0, 0.0]
    assert len(checkpoints) == 4
    assert len(buoys) == 4
    assert json.loads(params["buoy_marks"]) == ["RED", "E", "W", "GREEN"]

    gps3, gps4 = checkpoints[2], checkpoints[3]
    assert math.isclose(
        params["course_heading_rad"],
        math.atan2(gps4[1] - gps3[1], gps4[0] - gps3[0]),
        abs_tol=1.0e-3,
    )
