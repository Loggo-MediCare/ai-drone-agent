"""PX4 / MAVSDK 遙測（唯讀）。

先用 PX4 SITL 模擬飛控，不需要硬體：
    https://docs.px4.io/main/en/simulation/
    pip install mavsdk
    預設連線位址 udp://:14540（PX4 SITL 的 MAVSDK 埠）

本模組只訂閱 battery、position、health、armed 四個遙測串流各一筆，
**不呼叫 action 外掛**，不會解鎖、起飛或驅動馬達。
"""
import asyncio

from .base import ERROR, MISSING, OK, SourceResult, utcnow


async def _read(address):
    from mavsdk import System
    drone = System()
    await drone.connect(system_address=address)
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for b in drone.telemetry.battery():
        battery = b.remaining_percent
        break
    async for p in drone.telemetry.position():
        position = (p.latitude_deg, p.longitude_deg, p.relative_altitude_m)
        break
    async for h in drone.telemetry.health():
        gps_ok = h.is_global_position_ok and h.is_home_position_ok
        break
    async for armed in drone.telemetry.armed():
        break
    return battery, position, gps_ok, armed


def fetch(address="udp://:14540", timeout_s=10):
    try:
        import mavsdk  # noqa: F401
    except ImportError:
        return SourceResult("TELEMETRY", "MAVSDK", MISSING, detail="mavsdk not installed (pip install mavsdk)")
    try:
        battery, position, gps_ok, armed = asyncio.run(asyncio.wait_for(_read(address), timeout_s))
    except asyncio.TimeoutError:
        return SourceResult("TELEMETRY", "MAVSDK", MISSING, detail=f"no flight controller at {address}")
    except Exception as e:
        return SourceResult("TELEMETRY", "MAVSDK", ERROR, detail=f"{type(e).__name__}: {e}")
    # MAVSDK 2.x 回報 0–100；舊版為 0–1
    soc = battery / 100.0 if battery > 1.0 else battery
    data = {"battery_soc": soc, "lat": position[0], "lon": position[1],
            "relative_altitude_m": position[2], "gps_ok": gps_ok, "armed": armed}
    if not gps_ok:
        return SourceResult("TELEMETRY", "MAVSDK", ERROR, data, utcnow(), True, "GPS / home position not OK")
    return SourceResult("TELEMETRY", "MAVSDK", OK, data, utcnow(), True, f"battery {soc:.0%}")
