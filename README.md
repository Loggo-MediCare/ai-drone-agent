# 自主無人機 AI 代理（AI Drone Agent）

以《每位人工智慧工程師必須建構的 30 個智能體》第十六章〈具身與實體世界智能體〉為基礎，把書中的程式片段擴充成**可以完整執行的四層控制無人機任務模擬**：

| 層級 | 頻率 | 本專案實作 | 檔案 |
|---|---|---|---|
| 任務規劃層 | 0.1–1 Hz | 規劃器（規則式或 GPT）**只決定測量順序與是否建議返航**；統一約束包絡（天氣／空域／電池／綠帶／任務幾何） | `planner.py`、`llm_planner.py`、`constraints.py`、`supervisor.py` |
| 航線幾何 | 規劃／改道時 | **RRT\*** 計算所有航線與繞行，禁航區外擴可設定的安全距離 | `route_planner.py` |
| 運動規劃層 | 1–10 Hz（2 Hz） | 檢查剩餘路徑是否仍無碰撞，失效就重算 | `motion.py` |
| 軌跡控制層 | 50–200 Hz（100 Hz） | 路徑時間參數化成設定點，**PID** 計算推力向量 | `trajectory.py` |
| 伺服控制層 | 1–10 kHz（1 kHz） | 四顆馬達**電流 PI 迴路**，抑制匯流排電壓漣波干擾 | `servo.py` |

核心原則（第十六章）：**安全不是正確規劃的副產品，而是對可執行動作集合 A_safe(s) 的明確限制。** LLM／規劃器只能「提案」，能不能飛由程式碼決定。

純 Python 標準函式庫即可執行，不需 GPU、無人機或 API 金鑰。

## 執行

```
python demo.py            # 10 個任務情境（每個完整任務約 30 秒模擬時間）
python demo.py 6 8        # 只跑指定情境
python -m unittest discover -s tests -t . -v   # 52 個測試
python calibrate.py       # 量測懸停/巡航功率，校正電池估算
```

選用：讓 GPT 決定測量順序（航線幾何與安全層仍由程式碼負責）

```
pip install -r requirements-llm.txt
copy .env.example .env    # 填入 OPENAI_API_KEY
python demo.py --llm
```

## 情境與結果（任務：起降點 → 6 km 走廊 4 個測量點 → 返航，書中渥太華一月條件 -6.2°C、風 18.5 km/h、電量 82%）

| 情境 | 結果 | 展示的機制 |
|---|---|---|
| 1. 正常任務 | `COMPLETED`，82% → 約 45% | 通訊塔擋在 WP3→WP4 與返航路線上，RRT* 自動繞過 |
| 2. 氣溫 -12°C | `NO_GO` | WEATHER=RED，馬達不解鎖 |
| 3. 起飛電量 25% | `NO_GO` | BATTERY=RED（起飛 < 30%） |
| 4. 綠帶未授權 | `NO_GO` | PARKS=RED，且綠帶橫跨走廊無法繞行 |
| 5. 天氣資料 20 分鐘未更新 | `NO_GO` | WEATHER=STALE，保守融合視同失敗 |
| 6. 飛行中 NOTAM 臨時禁航 | `COMPLETED_REROUTED` | NOTAM 推播 → 包絡轉紅 → 影響傳播 → RRT* 改道（含 100 m 安全距離） |
| 7. NOTAM 橫跨整條走廊 | `RETURNED_HOME` | 無可行改道 → 返航航線也經過安全檢查與電量預估 |
| 8. 陣風 32 km/h 40 秒 | `COMPLETED`，懸停 1 次 | 軌跡層拒絕推進、PID 原地抗風，陣風減弱 5 秒後恢復 |
| 9. 航線產生器給出 80 km/h 超速航線 | `COMPLETED` | 安全層拒絕 → 重新規劃 |
| 10. 任務規劃器回傳重複 ID、漏掉 WP1 | `COMPLETED`，4 點都實際測量 | 決策被拒絕 → 重試；漏掉的點不會被當成已完成 |

所有情境的「進入禁區」次數皆為 0。

## 架構：任務規劃與幾何規劃分離

```
任務規劃器（規則式或 GPT）        → MissionDecision {order, return_home, reason}
        │  validate_decision()：order 必須剛好是剩餘測量點 ID 的排列（無重複、無未知、無遺漏）
        │  不合格 → 回饋原因重試；連續 2 次不合格 → 改用預設順序
        ▼
RoutePlanner（確定性，RRT*）       → 每一段航線；禁航區外擴 mission.exclusion_clearance_m（預設 100 m）
        ▼
約束包絡（五個領域保守融合）       → 任何一個非 GREEN 就否決
        ▼
SafetyMonitor（獨立）              → 檢查**每一段完整直線**：不得進入禁區、不得進入安全距離
        ▼
軌跡層 → 伺服層（全部模擬）
        ▼
抵達航點 → 只有實際位置在測量點 10 m 內才記錄 SurveyEvent（測量完成）
```

| 原則 | 實作 |
|---|---|
| GPT 不產生航線或馬達指令 | `llm_planner._Decision` 只有 `order`、`return_home`、`reason` 三個欄位；測試確認沒有座標欄位 |
| 所有航線與繞行由確定性規劃器計算 | `route_planner.RoutePlanner`（RRT*），包含飛行中動態生效的 NOTAM |
| 可設定的安全距離、檢查整段直線 | `Mission.exclusion_clearance_m`、`SafetyMonitor._geometry()`；`mission_input.json` 可設定 |
| 完成狀態只看實際位置 | `MissionSupervisor._on_arrival()`；航點標籤（label）只用於顯示，不影響任何判定 |
| 返航不假設安全 | `_rth()`：RRT* 返航航線 → 安全層逐段檢查 → 電量預估；任一不通過就原地降落 |
| 已身處新生效的禁區內 | 該區域對返航豁免（只能離開）；其他禁區與安全距離照常檢查 |
| 不驅動真實飛機 | 沒有任何程式碼呼叫 MAVSDK action 或輸出到真實馬達；`telemetry.py` 只讀取 |

### 回歸測試（`tests/test_architecture.py`，15 個）

| 測試 | 驗證內容 |
|---|---|
| 轉角切過禁區 | 兩個航點都在禁區外，但連線切過禁區轉角 → 拒絕 |
| 安全距離不足 | 平行禁區 50 m：安全距離 100 m 拒絕、30 m 允許 |
| 從安全距離內離開 | 已在外擴帶內時允許離開，但不允許穿過禁區 |
| RRT* 航線保持安全距離 | 每一段都不碰外擴後的禁區；測量點離禁區太近時規劃失敗 |
| 重複／未知／遺漏航點 ID | `validate_decision` 全部拒絕；任務定義有重複 ID 時不解鎖 |
| 規劃器漏掉 WP1（假裝已完成） | 拒絕，無人機仍實際飛到並測量 WP1 |
| 規劃器一直出錯 | 改用預設順序完成任務 |
| 飛行中 NOTAM 貼近航線（未穿越） | 包絡仍全綠，但安全層發現距離不足 → 改道，進入禁區 0 次 |
| 重新規劃失敗 | NOTAM 蓋住 WP2 → 返航，WP2 不會被記為完成 |
| 偽造的測量完成 | 返航航線偽造 `survey_id=WP1` 但座標不符 → 忽略 |
| 返航路上有禁區 | 返航航線用 RRT* 繞過，進入禁區 0 次 |
| 返航電量不足 | 不嘗試返航，原地降落 |
| 起降點在禁區內 | 找不到安全返航航線 → 原地降落 |
| GPT 輸出只有順序 | 用假 LLM 驗證輸出結構與提示詞，不需 API 金鑰 |

GPT 實測（情境 6）：GPT 自行把順序改為 WP1 → WP3 → WP4 → WP2，RRT* 計算繞行，任務完成、進入禁區 0 次。改架構前，GPT 自己畫的繞行點連續 3 次切過禁區轉角而失敗。

正常任務中各層的執行次數（約 1280 秒飛行）：任務層 43、運動層 ≈2500、軌跡層 ≈128,000、伺服層 ≈1,280,000，電流追蹤 RMS 誤差約 0.09 A。

## 從書中程式摘錄的部分（`book_listings/`，原文照錄，無法直接執行）

| 檔案 | 書中清單 | 內容 |
|---|---|---|
| `16_0_lower_layer_skeletons.py` | 虛擬碼 | 運動規劃／軌跡控制／伺服控制骨架 |
| `16_1_common_setup.py` | 16.1 | WorldModel、ControlInterface、SafetyMonitor 介面 |
| `16_2_embodied_agent_tools.py` | 16.2 | 四職責工具分解的 LangChain 代理 |
| `16_3_safety_execution_loop.py` | 16.3 | 安全約束執行迴圈 |
| `16_4_cross_domain_graph.py` | 16.4 | 跨領域知識圖譜建構 |
| `16_5_influence_propagation.py` | 16.5 | 加權廣度優先影響傳播 |
| `16_6_embodied_drone_agent.py` | 16.6 | 無人機任務主管代理 |
| `16_7_constraint_assembler.py` | 16.7 | 含 NOTAM 的跨領域約束組裝器 |

## 書中程式的問題與本專案的修正

| 書中 | 問題 | 本專案 |
|---|---|---|
| 16.3 | Step 4 與 Step 4b 把同一個動作執行兩次 | 每個航點只下達一次 |
| 16.3 | 被拒後重新規劃的 plan 沒有使用，仍迭代舊動作 | 新計畫重新走完整驗證流程 |
| 16.6 | 安全只靠提示詞 Protocol，`dispatch_waypoint_command` 本身不檢查 | 航點下達前由程式碼呼叫 `SafetyMonitor.validate()`，規劃器無法繞過 |
| 16.6 | 只檢查起飛電量 30% | 加上每個航點 20%、返航 15%，並計入起降懸停耗電 |
| 16.6 | LLM 直接下達航點座標 | LLM 只決定順序；座標由 RRT* 計算 |
| 16.7 | 由 LLM 輸出 JSON 宣告包絡是否全綠 | 保守融合由確定性程式碼計算 |
| 16.7 | `ConstraintEnvelope`、`ValidationError`、`add_constraint_node` 未定義 | 全部實作 |
| 16.7 | STALE 規則只寫在文字裡 | 依領域設定過期門檻（天氣 15 分鐘、NOTAM 60 秒） |
| 16.4 | 權重鉗位、專家覆蓋、稽核日誌只寫在文字裡 | `knowledge_graph.py` 實作 |

另外處理了書中沒提到的情況：**NOTAM 生效時無人機已經在禁航區內**。這時沒有任何合法路線，本專案讓返航以直線離開該區域，而不是在禁航區中原地降落。

## 模型假設（示意值，非特定機型）

- 機體 1.5 kg，質點＋線性空氣阻力；姿態迴路視為理想（推力方向即時達到）。
- 馬達：RL 電路＋反電動勢，6S 22.2 V、每顆上限 15 A；匯流排電壓漣波 ±0.3 V 當作干擾。
- 電池 355 Wh；低溫每低於 0°C 一度，等效耗電增加 2%。
- 電量事前估算以 `calibrate.py` 的量測擬合，再乘 1.2 倍保守裕度；測試確認實際剩餘電量不低於估算值。
- 平面座標以起降點為原點（公尺），禁航區與綠帶以矩形表示。

## 關於控制迴圈的頻率（重要）

本專案的 100 Hz 軌跡層與 1 kHz 伺服層是**模擬時間**：每一步代表 10 ms／1 ms 的物理時間，但 Python 並不保證在真實的 10 ms／1 ms 內完成（實際上約以 2.7 倍速跑完）。測試通過只代表控制邏輯正確，**不代表具備即時性**。真實無人機的姿態與馬達閉迴路必須交給 PX4 等具備時序保證的飛控執行；本專案的 Python 部分只適合任務層與運動層。

## 資料輸入層（`drone_agent/inputs/`）

統一的任務輸入檔 `data/mission_input.json` → 各資料來源轉接器 → **起飛前資料閘門** → 模擬器。

```
python run_mission.py                                         # 模擬模式範例（高雄練習航線）
python run_mission.py data/mission_input.real.example.json    # 真實資料模式範例（目前會被拒絕，見下方）
python run_mission.py data/mission_input.json --llm           # 用 GPT 規劃
```

### 資料來源

| 領域 | 來源 | 轉接器 | 取得方式 | 狀態 |
|---|---|---|---|---|
| 天氣、風速、降雨 | [中央氣象署開放資料](https://opendata.cwa.gov.tw) O-A0001-001 自動氣象站 | `cwa_weather.py` | 申請會員授權碼，放進 `.env` 的 `CWA_API_KEY` | 已實作 API 呼叫；以樣本資料測試，**尚未用真實授權碼驗證** |
| 禁限航區 | [民航局無人機管理系統](https://drone.caa.gov.tw) | `airspace.load_caa_geojson()` | 查詢後整理成 GeoJSON（無本專案可用的公開 API） | 檔案匯入＋人工核實 |
| NOTAM | [臺北飛航情報服務系統](https://ais.anws.gov.tw) | `airspace.load_notam_file()` | 查詢後整理成 JSON（格式見 `data/notams.example.json`） | 檔案匯入＋人工核實 |
| 電池、GPS、解鎖狀態 | PX4 SITL／飛控，經 [MAVSDK](https://mavsdk.mavlink.io) | `telemetry.py` | `pip install mavsdk`，啟動 PX4 SITL（預設 `udp://:14540`） | **唯讀**；尚未接 SITL 實測 |
| 障礙物、保護區 | GIS、現場勘查 | `builder._gis()` | 寫在 `mission_input.json` 的 `route` | 需現場核實 |
| GPT | [OpenAI API Keys](https://platform.openai.com/api-keys) | `llm_planner.py` | `.env` 的 `OPENAI_API_KEY` | 已實測 |

民航局圖資僅供參考，仍以正式公告為準；NOTAM 以飛航情報系統公告為準。

### 兩種模式

| | 模擬模式 `"simulation": true` | 真實資料模式 `"simulation": false` |
|---|---|---|
| `SIMULATED` 來源 | 允許 | **拒絕** |
| 未核實（`verified: false`） | 允許 | **拒絕**（空域、NOTAM、GIS 必須由操作員對照正式公告／現場核實） |
| 資料過期、缺失、錯誤 | 拒絕 | 拒絕 |
| `restricted: true`、`notam_clear: false` | 拒絕 | — |
| 飛行 | 模擬 | **仍然是模擬**，不連接任何真實馬達 |

新鮮度上限（真實來源）：天氣 30 分鐘（CWA 每 10 分鐘觀測、另有發布延遲）、NOTAM 1 小時、禁限航區 7 天、遙測 5 秒。

### 轉換規則

- 經緯度以起點為原點換成平面公尺座標（等距矩形近似）。
- 多邊形、圓形區域一律換成**外接矩形**：比實際區域大，屬保守處理。
- 禁限航區視為永久限制；NOTAM 依生效時間換算成以起飛為 0 的模擬秒數。
- CWA 以 `-99` 表示缺值：缺少風速、氣溫或降雨時回報 ERROR，**不會猜測數值**。
- 資料是起飛時的快照；飛行中持續更新外部資料尚未實作。

### 用 `mission_input.real.example.json` 實際會看到

```
WEATHER   CWA          MISSING    CWA_API_KEY not set
AIRSPACE  CAA_GEOJSON  OK         （範例檔 verified=false）
NOTAM     AIS_FILE     STALE      （範例檔是過去的時間）
TELEMETRY MAVSDK       MISSING    mavsdk not installed
GIS       GIS_SURVEY   OK         （verified=false）
起飛前資料閘門：拒絕起飛
```

要讓它通過：申請 CWA 授權碼、填入真實並核實過的禁限航區與 NOTAM、啟動 PX4 SITL、完成航線現場核實。

## 下一步

1. 接上 **PX4 SITL** 實測 `telemetry.py`，再把航點改用 MAVLink 下達給 SITL（取代 `servo.py`、`physics.py`；真實姿態與馬達迴路由 PX4 負責）。
2. 用真實 CWA 授權碼驗證 `cwa_weather.py` 的欄位解析。
3. 飛行中定期重新讀取天氣與 NOTAM（目前只在起飛前讀一次）。
4. 運動層加入動態障礙物，RRT* 改為增量式重規劃。
5. 以 NVIDIA Isaac Sim 做視覺感知，更新世界模型的信念狀態 b(s)。

## Physical AI Drone：MCP 工具與安全閘道

[`physical-ai-drone/`](physical-ai-drone/) 以本專案的 `drone_agent` 作為模擬後端，加入 11 個 MCP 工具、確定性安全閘道（認證、scope、防重放、失效即關閉、稽核）、對抗性測試與能耗基準測試。只有模擬，沒有任何實機控制路徑。詳見 [physical-ai-drone/README.md](physical-ai-drone/README.md)。

專案層級的 Claude Code 子代理：`.claude/agents/physical-ai-drone-engineer.md`。
