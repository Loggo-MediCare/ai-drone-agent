"""運動規劃層（1–10 Hz）：RRT* 計算從目前位置到目標航點的無碰撞路徑。

輸入的 keepouts 是「已經外擴過安全距離」的禁區（見 route_planner.keepouts）。
直線無碰撞時直接採用直線；否則跑 RRT*，再做捷徑平滑。
固定亂數種子，使模擬可重現。
"""
import math
import random

from .geometry import Rect, dist, segment_intersects_rect

CLEARANCE_M = 50.0   # 實體障礙物的外擴距離


def inflate(rects, margin):
    return [Rect(r.xmin - margin, r.ymin - margin, r.xmax + margin, r.ymax + margin) for r in rects]


class MotionPlanner:
    def __init__(self, bounds, seed=0, iters=1500, step_m=200.0, radius_m=500.0, goal_bias=0.1):
        self.bounds = bounds
        self.rng = random.Random(seed)
        self.iters = iters
        self.step_m = step_m
        self.radius_m = radius_m
        self.goal_bias = goal_bias
        self.rrt_runs = 0

    @staticmethod
    def _free(a, b, obs):
        return not any(segment_intersects_rect(a, b, r) for r in obs)

    @staticmethod
    def _escapable(start, keepouts):
        # 已經身處某個外擴區內（例如 NOTAM 剛生效）時，該區不能阻止離開；
        # 是否真的進入原始禁區由安全層另外檢查。
        return [r for r in keepouts if not r.contains(start)]

    def path_valid(self, path, keepouts):
        obs = self._escapable(path[0], keepouts) if path else keepouts
        return all(self._free(a, b, obs) for a, b in zip(path, path[1:]))

    def plan(self, start, goal, keepouts):
        obs = self._escapable(start, keepouts)
        if any(r.contains(goal) for r in keepouts):
            return None
        if self._free(start, goal, obs):
            return [start, goal]
        self.rrt_runs += 1
        return self._rrt_star(start, goal, obs)

    def _rrt_star(self, start, goal, obs):
        b = self.bounds
        nodes, parent, cost = [start], [None], [0.0]
        best_goal = None   # (cost, node index)
        for _ in range(self.iters):
            q = goal if self.rng.random() < self.goal_bias else (
                self.rng.uniform(b.xmin, b.xmax), self.rng.uniform(b.ymin, b.ymax))
            if any(r.contains(q) for r in obs):
                continue
            near_i = min(range(len(nodes)), key=lambda i: dist(nodes[i], q))
            n = nodes[near_i]
            d = dist(n, q)
            new = q if d <= self.step_m else (n[0] + (q[0] - n[0]) / d * self.step_m,
                                              n[1] + (q[1] - n[1]) / d * self.step_m)
            if not self._free(n, new, obs):
                continue
            # 選擇成本最低的父節點
            neighbours = [i for i in range(len(nodes)) if dist(nodes[i], new) <= self.radius_m]
            best_i, best_c = near_i, cost[near_i] + dist(n, new)
            for i in neighbours:
                c = cost[i] + dist(nodes[i], new)
                if c < best_c and self._free(nodes[i], new, obs):
                    best_i, best_c = i, c
            nodes.append(new)
            parent.append(best_i)
            cost.append(best_c)
            k = len(nodes) - 1
            # 重新接線（rewire）
            for i in neighbours:
                c = best_c + dist(new, nodes[i])
                if c < cost[i] and self._free(new, nodes[i], obs):
                    parent[i] = k
                    cost[i] = c
            if dist(new, goal) <= self.step_m and self._free(new, goal, obs):
                total = best_c + dist(new, goal)
                if best_goal is None or total < best_goal[0]:
                    best_goal = (total, k)
        if best_goal is None:
            return None
        path, i = [goal], best_goal[1]
        while i is not None:
            path.append(nodes[i])
            i = parent[i]
        return self._shortcut(path[::-1], obs)

    def _shortcut(self, path, obs):
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._free(path[i], path[j], obs):
                j -= 1
            out.append(path[j])
            i = j
        return out


def path_length(path):
    return sum(dist(a, b) for a, b in zip(path, path[1:]))
