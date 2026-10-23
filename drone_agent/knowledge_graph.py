"""跨領域知識圖譜（Listing 16.4 / 16.7）與影響傳播（Listing 16.5）。

書中的治理控制在這裡實作：
- 權重鉗位：每種關係的權重被限制在專家定義的範圍內。
- 專家覆蓋：source="expert" 的權重優先於 LLM 估計。
- 稽核日誌：每次寫入都記錄來源與時間。
"""
from collections import deque
from dataclasses import dataclass

WEIGHT_BOUNDS = {
    "restricts": (0.0, 1.0),
    "part_of": (0.0, 1.0),
    "affects": (0.0, 1.0),
    "constrains": (0.0, 1.0),
}


@dataclass
class Edge:
    source_id: str
    target_id: str
    relation: str
    weight: float
    source: str


class KnowledgeGraph:
    def __init__(self):
        self.nodes = {}
        self._edges = {}
        self.audit_log = []

    def add_node(self, node_id, domain, **attrs):
        self.nodes[node_id] = {"domain": domain, **attrs}

    def add_constraint_node(self, domain, node_id, status, weight):
        """Listing 16.7 呼叫但未定義的方法。"""
        self.add_node(node_id, domain, status=status, weight=weight)
        return self.nodes[node_id]

    def add_edge(self, source_id, target_id, relation, weight, source="expert", t=0.0):
        lo, hi = WEIGHT_BOUNDS.get(relation, (0.0, 1.0))
        clamped = max(lo, min(hi, weight))
        key = (source_id, target_id, relation)
        existing = self._edges.get(key)
        if existing and existing.source == "expert" and source != "expert":
            self.audit_log.append((t, source, key, weight, "ignored: expert weight takes precedence"))
            return existing
        edge = Edge(source_id, target_id, relation, clamped, source)
        self._edges[key] = edge
        note = "clamped" if clamped != weight else "ok"
        self.audit_log.append((t, source, key, weight, note))
        return edge

    def outgoing_edges(self, node_id):
        return [e for e in self._edges.values() if e.source_id == node_id]


def propagate_influence(graph, source_id, initial_strength=1.0, threshold=0.1):
    """Listing 16.5 原樣：加權廣度優先，回傳 {entity_id: (strength, [path])}。"""
    impacts = {source_id: (initial_strength, [source_id])}
    queue = deque([(source_id, initial_strength, [source_id])])
    while queue:
        node_id, strength, path = queue.popleft()
        for edge in graph.outgoing_edges(node_id):
            propagated = strength * edge.weight
            if propagated < threshold:
                continue
            new_path = path + [edge.target_id]
            if edge.target_id not in impacts or propagated > impacts[edge.target_id][0]:
                impacts[edge.target_id] = (propagated, new_path)
                queue.append((edge.target_id, propagated, new_path))
    return impacts
