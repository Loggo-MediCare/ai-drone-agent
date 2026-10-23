"""資料來源共用型別：狀態、時間戳、新鮮度檢查。"""
from dataclasses import dataclass, field
from datetime import datetime, timezone

OK = "OK"
SIMULATED = "SIMULATED"
MISSING = "MISSING"
STALE = "STALE"
UNVERIFIED = "UNVERIFIED"
ERROR = "ERROR"
BLOCKED = "BLOCKED"          # 資料正常，但內容本身禁止起飛（例如空域標記為限航）


@dataclass
class SourceResult:
    domain: str                      # WEATHER / AIRSPACE / NOTAM / TELEMETRY / GIS
    source: str                      # CWA、CAA_GEOJSON、AIS_FILE、MAVSDK、SIMULATED …
    status: str
    data: dict = field(default_factory=dict)
    observed_at: datetime = None     # 資料本身的觀測／發布時間（UTC）
    verified: bool = False           # 是否已由操作員對照正式公告核實
    detail: str = ""

    def age_s(self, now):
        return None if self.observed_at is None else (now - self.observed_at).total_seconds()


def utcnow():
    return datetime.now(timezone.utc)


def parse_time(value):
    """ISO 8601 字串 → 有時區的 datetime；沒有時區時視為台灣時間 +08:00。"""
    if value is None:
        return None
    t = datetime.fromisoformat(str(value).replace("Z", "+00:00"))
    if t.tzinfo is None:
        from datetime import timedelta
        t = t.replace(tzinfo=timezone(timedelta(hours=8)))
    return t.astimezone(timezone.utc)


def check_freshness(result, now, max_age_s):
    """把過期或缺少時間戳的資料標成 STALE。"""
    if result.status != OK:
        return result
    age = result.age_s(now)
    if age is None:
        result.status, result.detail = STALE, "no timestamp"
    elif age > max_age_s:
        result.status, result.detail = STALE, f"age {age:.0f}s > {max_age_s:.0f}s"
    elif age < -300:
        result.status, result.detail = ERROR, f"timestamp {-age:.0f}s in the future"
    return result
