"""電池模型。

實際耗電由伺服層的電功率（Σ u·i）積分而來；本檔的 estimated_power_w() 是
規劃與約束包絡用的「事前估算」，以模擬量測校正後再乘上保守裕度。
低溫降低可用容量：每低於 0°C 一度，等效耗電增加 2%（示意值）。
"""
BATTERY_WH = 355.0                     # 6S 16 Ah
CAPACITY_J = BATTERY_WH * 3600
COLD_COEF = 0.02
PROJECTION_MARGIN = 1.2

# 由 calibrate.py 量測的懸停與巡航功率擬合（見 README）
P_HOVER_W = 312.5                      # 量測：懸停 312.5 W
K_AIRSPEED = 0.0009                    # 量測：36 km/h 無風 322.4 W
K_WIND = 0.0024                        # 量測：36 km/h 逆風 18.5 / 25 km/h → 335.0 / 340.6 W（取較保守值）


def cold_factor(temp_c):
    return 1.0 + COLD_COEF * max(0.0, -temp_c)


def soc_drain(power_w, dt_s, temp_c):
    return power_w * dt_s * cold_factor(temp_c) / CAPACITY_J


def estimated_power_w(airspeed_kmh, wind_kmh):
    return P_HOVER_W * (1 + K_AIRSPEED * airspeed_kmh + K_WIND * wind_kmh)


def soc_for_distance(distance_m, airspeed_kmh, wind_kmh, temp_c):
    if distance_m <= 0:
        return 0.0
    seconds = distance_m / (airspeed_kmh / 3.6)
    return soc_drain(estimated_power_w(airspeed_kmh, wind_kmh), seconds, temp_c) * PROJECTION_MARGIN
