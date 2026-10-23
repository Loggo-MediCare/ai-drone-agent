"""空域限制：民航局（CAA）禁限航區、飛航情報（AIS）NOTAM。

這兩個來源目前沒有本專案可直接呼叫的公開 API，因此採「檔案匯入＋人工核實」：
- CAA：從 https://drone.caa.gov.tw 查詢後，把相關區域整理成 GeoJSON。
- NOTAM：從 https://ais.anws.gov.tw 查詢後，依 docs 的格式整理成 JSON。

民航局圖資僅供參考，仍以正式公告為準；所以檔案必須帶 "verified": true
（操作員已對照正式公告確認），真實資料模式才接受。
"""
import json

from .base import ERROR, MISSING, OK, SourceResult, parse_time


def _load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_caa_geojson(path):
    """GeoJSON FeatureCollection，Polygon 座標為 [lon, lat]（GeoJSON 標準順序）。

    頂層可加 "retrieved_at"（ISO 時間）與 "verified"（bool）。"""
    try:
        doc = _load(path)
    except FileNotFoundError:
        return SourceResult("AIRSPACE", "CAA_GEOJSON", MISSING, detail=f"{path} not found")
    except (OSError, json.JSONDecodeError) as e:
        return SourceResult("AIRSPACE", "CAA_GEOJSON", ERROR, detail=f"cannot read {path}: {e}")
    zones = []
    for feat in doc.get("features", []):
        geom = feat.get("geometry") or {}
        if geom.get("type") != "Polygon":
            return SourceResult("AIRSPACE", "CAA_GEOJSON", ERROR,
                                detail=f"unsupported geometry {geom.get('type')}")
        ring = geom["coordinates"][0]
        zones.append({
            "name": feat.get("properties", {}).get("name", "unnamed"),
            "area": {"type": "polygon", "coordinates": [[lat, lon] for lon, lat in ring]},
        })
    return SourceResult("AIRSPACE", "CAA_GEOJSON", OK, {"zones": zones},
                        parse_time(doc.get("retrieved_at")), bool(doc.get("verified")),
                        f"{len(zones)} zone(s)")


def load_notam_file(path):
    """格式：
    {"retrieved_at": "...", "verified": true,
     "notams": [{"id": "A1234/26", "valid_from": "...", "valid_until": "...",
                 "area": {"type": "circle", "center": [lat, lon], "radius_m": 1000}}]}"""
    try:
        doc = _load(path)
    except FileNotFoundError:
        return SourceResult("NOTAM", "AIS_FILE", MISSING, detail=f"{path} not found")
    except (OSError, json.JSONDecodeError) as e:
        return SourceResult("NOTAM", "AIS_FILE", ERROR, detail=f"cannot read {path}: {e}")
    notams = []
    try:
        for n in doc.get("notams", []):
            notams.append({
                "id": n["id"],
                "valid_from": parse_time(n["valid_from"]),
                "valid_until": parse_time(n["valid_until"]),
                "area": n["area"],
            })
    except (KeyError, ValueError) as e:
        return SourceResult("NOTAM", "AIS_FILE", ERROR, detail=f"bad NOTAM entry: {e}")
    return SourceResult("NOTAM", "AIS_FILE", OK, {"notams": notams},
                        parse_time(doc.get("retrieved_at")), bool(doc.get("verified")),
                        f"{len(notams)} NOTAM(s)")
