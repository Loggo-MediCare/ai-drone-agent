# 摘錄自第十六章 Listing 16.3，原文照錄。
# 已知問題（本專案 drone_agent/supervisor.py 已修正）：
#   1. Step 4 與 Step 4b 會把同一個動作執行兩次。
#   2. 被拒絕後重新規劃的 plan 沒有被使用，continue 之後仍在迭代舊的 actions。
# Listing 16.3: Safety-constrained action execution loop
def execute_with_safety(agent, task: str, world_model, safety_monitor):
    """Execute an embodied task with pre-execution safety
    validation and continuous monitoring."""
    # Step 1: Agent reasons about the task and proposes actions
    plan = agent.invoke({"messages": [HumanMessage(task)]})
    # Step 2: Extract proposed actions from agent output
    actions = extract_actions(plan)
    # Step 3: Validate each action against A_safe(s)
    for action in actions:
        state = world_model.get_current_state()
        validation = safety_monitor.validate(
            action, state
        )
        if not validation.is_safe:
            # Rejected: replan with constraint feedback
            plan = agent.invoke({"messages": [
                HumanMessage(
                    f"Action rejected: {validation.reason}. "
                    f"Replan within: {validation.constraints}"
                )
            ]})
            continue
        # Step 4: Execute validated action
        result = control_interface.execute(action)
        world_model.update(result.observations)
        # Step 4b: timeout and actuator feedback validation
        try:
            result = control_interface.execute(action, timeout=5.0)
            if not result.actuator_ok:
                # Actuator reported fault; halt and surface for replanning
                safety_monitor.halt(reason=result.fault_code)
                break
            world_model.update(result.observations)
        except TimeoutError:
            # Execution did not complete within timeout; production systems
            # require fuller error trees (retry, degrade, emergency stop).
            safety_monitor.halt(reason="execution_timeout")
            break
