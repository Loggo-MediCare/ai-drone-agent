"""航線幾何規劃（確定性）：依任務規劃層給的順序，用 RRT* 計算每一段航線。

禁區（keep-out）＝
    實體障礙物  外擴 OBSTACLE_CLEARANCE_M
    禁航區（生效中的 NOTAM、禁限航區、未授權保護區）外擴 mission.exclusion_clearance_m

輸出的每一段直線都已避開所有禁區；安全層會再獨立檢查一次每一段（不只檢查航點）。
"""
from .geometry import Rect
from .motion import CLEARANCE_M as OBSTACLE_CLEARANCE_M
from .motion import MotionPlanner, inflate
from .safety import WaypointCommand


def keepouts(obstacles, exclusion_zones, exclusion_clearance_m):
    return inflate(obstacles, OBSTACLE_CLEARANCE_M) + inflate(exclusion_zones, exclusion_clearance_m)


class RoutePlanner:
    name = "rrt-star"

    def __init__(self, motion):
        self.motion = motion

    def plan(self, start, order, mission, exclusion_zones, obstacles):
        """回傳 (commands, "")；失敗時回傳 (None, 原因)。order 不含 HOME，會自動接上返航。"""
        zones = keepouts(obstacles, exclusion_zones, mission.exclusion_clearance_m)
        stops = [(wid, mission.point(wid)) for wid in order] + [("HOME", mission.home)]
        cmds, pos = [], start
        for wid, p in stops:
            if p is None:
                return None, f"unknown waypoint id {wid}"
            path = self.motion.plan(pos, p, zones)
            if path is None:
                return None, (f"no collision-free route to {wid} with "
                              f"{mission.exclusion_clearance_m:.0f} m exclusion clearance")
            for i, q in enumerate(path[1:-1], 1):
                cmds.append(WaypointCommand(q[0], q[1], mission.altitude_m, mission.airspeed_kmh, f"VIA{i}→{wid}"))
            cmds.append(WaypointCommand(p[0], p[1], mission.altitude_m, mission.airspeed_kmh, wid,
                                        survey_id=None if wid == "HOME" else wid))
            pos = p
        return cmds, ""
