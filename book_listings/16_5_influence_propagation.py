# 摘錄自第十六章 Listing 16.5，原文照錄。
# Listing 16.5: Influence propagation via weighted breadth-first traversal
from collections import deque


def propagate_influence(
    graph, source_id: str,
    initial_strength: float = 1.0,
    threshold: float = 0.1
) -> dict:
    """Weighted BFS: propagate influence from a source
    entity through the cross-domain knowledge graph.
    Returns {entity_id: (strength, [path])}."""
    impacts = {source_id: (initial_strength, [source_id])}
    queue = deque([
        (source_id, initial_strength, [source_id])
    ])
    while queue:
        node_id, strength, path = queue.popleft()
        for edge in graph.outgoing_edges(node_id):
            propagated = strength * edge.weight
            if propagated < threshold:
                continue  # Attenuation cutoff
            new_path = path + [edge.target_id]
            # Keep strongest path to each entity
            if (
                edge.target_id not in impacts
                or propagated > impacts[edge.target_id][0]
            ):
                impacts[edge.target_id] = (
                    propagated, new_path
                )
                queue.append(
                    (edge.target_id, propagated, new_path)
                )
    return impacts
