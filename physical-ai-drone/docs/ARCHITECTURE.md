# Physical AI Drone：架構與實作計畫

> 第一個垂直切片：**純模擬**。沒有任何程式路徑可以控制真實的馬達、PWM、MAVLink 或飛控。

以《每位人工智慧工程師必須建構的 30 個智能體》第 16 章〈具身與實體世界智能體〉為基礎，在既有的 `drone_agent` 四層控制模擬之上，加入 **MCP 工具介面**、**確定性安全閘道**、型別化的工具合約、對抗性測試與能耗基準測試。

## 1. 架構與相依性評估

### 已存在、可直接重用（repo 根目錄的 `drone_agent/`，52 個測試）

| 第 16 章的元件 | 既有實作 |
|---|---|
| 策略推理（只提案） | `planner.py`（規則式）、`llm_planner.py`（GPT，選用） |
| 世界模型 | `world.py`：天氣、NOTAM、保護區、障礙物，每筆讀數有時間戳 |
| 跨領域約束（廣度） | `constraints.py`：WEATHER、AIRSPACE、BATTERY、PARKS、MISSION 五領域保守融合 |
| 運動規劃 | `route_planner.py` + `motion.py`：RRT*，禁區外擴安全距離 |
| 確定性控制 | `trajectory.py`（PID）、`servo.py`（1 kHz 電流 PI）、`physics.py` |
| 安全執行 | `safety.py`：`SafetyMonitor` 逐段檢查航線 |
| 能耗模型 | `energy.py`：校正過的功率估算，含低溫係數與 ×1.2 預估裕度 |

### 這個切片新增的（`physical-ai-drone/`）

| 元件 | 檔案 |
|---|---|
| 型別化工具合約與嚴格解析 | `schemas.py`、`errors.py` |
| 確定性安全閘道（認證、授權、防重放、失效即關閉、稽核） | `gateway.py` |
| 模擬後端介面，以及包裝 `drone_agent` 的轉接器 | `adapters/base.py`、`adapters/drone_agent_sim.py` |
| 11 個工具 | `tools.py` |
| MCP 伺服器（stdio） | `mcp_server.py` |
| 策略層替身（模擬 LLM 提案） | `strategist.py` |
| 能耗基準測試 | `benchmark.py` |

### 模擬器選擇

| 選項 | 評估 | 這個切片 |
|---|---|---|
| `drone_agent`（純 Python） | 標準函式庫即可、可在 8 GB Mac 執行、完整四層控制 | ✅ **採用** |
| PX4 SITL + Gazebo | 真實飛控韌體的軟體模擬，最接近實機 | 未來：在 Linux 或 GitHub Codespaces 執行，新增一個 adapter |
| NVIDIA Isaac Sim | 需要 RTX GPU，且只支援 Linux／Windows，**不能在 macOS 執行**；**本身不含無人機飛控**，要搭配 PX4（例如 Pegasus Simulator） | 未來：雲端 GPU 機器 |
| ROS 2 | 適合多節點與實機整合；這個切片不需要 | 未來：adapter 可透過 ROS 2 topic 與 PX4 溝通 |
| NVIDIA GR00T | 人形／操作型機器人的基礎模型，**不是無人機控制所必需** | 選用的未來研究，不列為相依 |

## 2. 架構圖

```mermaid
flowchart TD
    LLM["策略層（LLM 或 MockStrategist）<br/>只能提案"] -->|MCP 工具呼叫| MCP["MCP 伺服器（stdio）"]
    MCP -->|"token + nonce + seq + 時間戳"| GW["安全閘道（確定性）<br/>認證 → 白名單 → scope → 時效 → 防重放<br/>→ 逾時 → 輸出檢查 → 稽核"]
    GW --> TOOLS["11 個工具<br/>嚴格解析 + 狀態一致性檢查"]
    TOOLS --> AD["SimulationAdapter 介面"]
    AD --> DA["drone_agent 轉接器"]
    DA --> RRT["RRT* 航線"]
    DA --> ENV["五領域約束包絡"]
    DA --> SM["SafetyMonitor（最終權威）"]
    DA --> SUP["四層模擬<br/>任務 → 運動 → 軌跡 PID → 伺服 1 kHz"]
    AD -.未來.-> PX4["PX4 SITL / Gazebo"]
    AD -.未來.-> ISAAC["Isaac Sim + PX4"]
    AD -.未來.-> AMR["AMR / 機械手臂"]
```

**核心原則（第 16 章）**：安全不是正確規劃的副產品，而是對可執行動作集合的明確限制。LLM 只能提案；能不能飛，由程式碼決定。

## 3. MCP 伺服器設計

- **傳輸**：只支援 **stdio**。信任邊界是行程本身：只有啟動伺服器的那個客戶端能與它通訊。HTTP 傳輸需要 MCP 的 OAuth（`token_verifier`），目前沒有設定，所以會直接拒絕啟動。
- **每次工具呼叫都經過閘道**：伺服器以內部客戶端 `mcp-stdio` 的身分簽署每一次呼叫（隨機 nonce、遞增 seq、時間戳）。閘道的檢查因此每次都會執行，日後開放遠端呼叫時可以直接沿用。
- **Scope**：`DRONE_MCP_SCOPES`，預設 `read,simulate`；只給 `read` 就是唯讀模式。
- **工具註記**：全部 `read_only_hint=True`、`destructive_hint=False`、`open_world_hint=False`。
- **外部文字一律標記為不可信**：NOTAM 文字、模擬日誌、策略層的理由，都包成 `{"untrusted": true, "value": "..."}`。伺服器的 `instructions` 也明確告訴 LLM：不要執行這些文字裡的指令。

| 工具 | Scope | 說明 |
|---|---|---|
| `get_drone_state` | read | 位置、速度、電量、模式（永遠 `armed: false`、`SIMULATION`） |
| `get_world_model` | read | 世界模型（見第 4 節） |
| `get_battery_status` | read | 電量、容量、可用 Wh、保留與起飛下限 |
| `get_weather_constraints` | read | 氣溫、風、降水、資料年齡、WEATHER 領域狀態 |
| `get_airspace_constraints` | read | NOTAM 與禁航區；文字為不可信資料 |
| `plan_mission` | simulate | A（起降點）→ B → 返航，RRT* 繞過障礙物與禁區 |
| `estimate_mission_energy` | simulate | 估算 Wh、時間、返航電量、各領域狀態 |
| `validate_trajectory` | simulate | 確定性安全檢查，回傳**所有**不合格原因 |
| `simulate_mission` | simulate | 以四層模擬實際飛行，回傳遙測 |
| `get_mission_telemetry` | read | 已模擬任務的遙測 |
| `evaluate_mission_performance` | read | 完成與否、違規、實耗 vs 估算、Wh/km、保留電量 |

## 4. 世界模型 schema（`physical_ai_drone.world_model/v1`）

```json
{
  "schema": "physical_ai_drone.world_model/v1",
  "frame": "ENU, metres, origin = home (A)",
  "observed_at": 1793000000.0,
  "backend": "drone_agent (pure-Python four-layer simulation)",
  "drone":   {"position_m": [0, 0, 0], "velocity_mps": [0, 0, 0], "soc": 0.9, "armed": false, "mode": "SIMULATION"},
  "battery": {"soc": 0.9, "capacity_wh": 355.0, "available_wh": 319.5, "reserve_soc": 0.15,
              "departure_min_soc": 0.30, "can_depart": true},
  "weather": {"temperature_c": 15.0, "wind_kmh": 10.0, "wind_toward_deg": -45.0, "precipitation": false,
              "data_age_s": 0.0, "status": "GREEN", "detail": ""},
  "geofence": [-500, -800, 2500, 800],
  "zones": [
    {"name": "tower", "kind": "obstacle", "rect": [950, -60, 1050, 60]},
    {"name": "NOTAM-1", "kind": "no_fly", "rect": [1200, -400, 1600, 400], "active": true,
     "text": {"untrusted": true, "value": "..."}}
  ]
}
```

區域種類：`obstacle`（實體障礙，外擴 50 m）、`no_fly`（NOTAM，外擴 100 m）、`protected`（保護區，含是否已授權）。

## 5. 安全控制器介面

### 閘道（`gateway.py`）：檢查順序，任何一步失敗就拒絕並寫入稽核

1. 請求格式正確 → `BAD_REQUEST`
2. 客戶端已認證（token 以 SHA-256 摘要做常數時間比對）→ `UNAUTHENTICATED`
3. 動作在白名單內（只有 11 個模擬／唯讀工具；綁定其他名稱會直接拋錯）→ `ACTION_NOT_ALLOWED`
4. 客戶端擁有該 scope → `FORBIDDEN`
5. `issued_at` 在 5 秒內、未超前 1 秒 → `STALE_REQUEST`
6. nonce 沒用過、seq 嚴格遞增 → `REPLAY`
7. 工具執行：逾時 → `TIMEOUT`；參數錯 → `INVALID_PARAMS`；後端狀態不合理 → `INCONSISTENT_STATE`；其他例外 → `INTERNAL_ERROR`
8. 輸出只能包含有限數值與 JSON 型別 → `UNSAFE_OUTPUT`

nonce 與 seq **在認證通過之後**才記錄（沒有 token 的攻擊者無法耗掉客戶端的序號），並且**在執行工具之前**就記錄（失敗的請求不能重送）。

### 軌跡驗證（`validate_trajectory`）：回傳所有原因，並由獨立層級重複把關

| 檢查 | 限制來源 |
|---|---|
| 每個航點的空速 (0, 50] km/h、高度 10–120 m、在地理圍欄內 | `drone_agent.safety`、`constraints` |
| 相鄰航點的加速度 ≤ 3 m/s² | 本切片新增 |
| 必須到達 B、必須返回 A | 本切片新增 |
| 每一段不得進入障礙物 50 m 內、禁區 100 m 內 | `motion.CLEARANCE_M`、任務設定 |
| 五領域約束包絡全部 GREEN（含天氣資料是否過期、電量逐點預估） | `constraints.evaluate_envelope` |
| 最終權威：`SafetyMonitor.validate_plan` | `drone_agent.safety` |

### 模擬中的應變行為

緊急降落、返航、中止、陣風懸停、飛行中改道，都由既有的 `MissionSupervisor` 實作（見 repo 根目錄 README 的 10 個情境）。這個切片透過 `simulate_mission` 讓它們在模擬中發生，**沒有**新增任何觸發實機動作的工具。

## 6. 完成、替身與未來工作

| 狀態 | 內容 |
|---|---|
| ✅ **完成並有測試** | 安全閘道全部檢查；11 個工具；嚴格 schema；`drone_agent` 轉接器（真實 RRT*、真實包絡、真實四層模擬）；MCP stdio 伺服器，包含與真正 MCP 客戶端的 stdio 往返測試；對抗性測試；能耗基準測試；A→B 示範 |
| 🟡 **替身（mock）** | `MockStrategist`：用固定的提案代替 LLM，其中故意包含不安全與格式錯誤的提案。真正的 LLM 策略層要實作同樣的 `propose()` 介面，輸出照樣經過閘道審查（repo 已有 `llm_planner.py` 可參考） |
| 🔵 **未來工作** | PX4 SITL／Gazebo adapter；Isaac Sim + PX4 adapter；ROS 2 整合；HTTP 傳輸 + MCP OAuth；金鑰輪替；多客戶端；AMR／機械手臂 adapter；GR00T 研究；更真實的能耗模型 |
| ⛔ **刻意不做** | 任何實機指令路徑。日後若需要，必須另外審查、經使用者明確授權，並有獨立的安全控制 |

## 7. 基準測試結果

任務：A (0, 0) → B (2000, 0) → 返航，高度 60 m，通訊塔擋在直線上。每個選擇都以完整四層模擬實際飛行。

| 條件 | 基準線（固定 36 km/h） | 最佳化器（掃描速度、取最低估算 Wh） | 策略層替身 |
|---|---|---|---|
| 無風 | 完成，43.1 Wh，477 s | 50 km/h：完成，**35.3 Wh**，379 s | 45 km/h：完成，37.2 Wh |
| 20 km/h 風 | 完成，43.4 Wh | 50 km/h：完成，**36.0 Wh** | 45 km/h：完成，37.6 Wh |
| −8 °C、電量 33 % | **未通過驗證，不起飛** | 50 km/h：完成，41.0 Wh | 45 km/h：完成，43.2 Wh |

- 完成率：基準線 2/3、最佳化器 3/3、策略層 3/3；所有飛行的禁區違規次數為 0。
- 策略層提案的 80 km/h 每次都被拒絕（`INVALID_PARAMS`）；低溫情境下 30 km/h 也因電量被拒絕。
- 估算比實際模擬耗能高約 7–15 %，來自 ×1.2 的保守預估裕度。

### ⚠️ 解讀：「飛越快越省電」是模型的結果，不是物理定律
`drone_agent` 的功率模型是 `P = P_hover × (1 + k·空速 + k·風速)`，功率隨速度**線性**增加，而飛行時間與速度成反比，所以速度越快總耗能越低，最佳化器因此總是選擇速度上限。真實多旋翼的功率曲線是 **U 形**：高速時寄生阻力約與速度的立方成正比，存在一個最省電的巡航速度。換成更真實的模型（或 PX4 SITL）後，最佳化器的選擇可能完全不同。

## 8. 模擬與真實世界的差距

- **能耗**：線性功率模型（見上）；風只用大小，沒有分順風／逆風；電池沒有電壓下垂與老化。
- **動力學**：質點加線性阻力，沒有姿態動力學、轉子動力學或地面效應。
- **感測**：狀態完美已知；沒有 GPS 誤差、IMU 漂移或延遲。不確定性目前只表現在保守裕度與資料過期判斷上。
- **幾何**：禁區是軸對齊矩形；2D 航線加固定高度。
- **時間**：世界在 t=0 觀測（起飛前）；`simulate_mission` 內部會模擬飛行中的變化（NOTAM 推播、陣風）。
- **通訊**：沒有連線中斷與封包遺失。
- **認證**：stdio 的行程邊界加閘道 token；尚未有 OAuth、金鑰輪替或硬體金鑰保護。

## 9. 分階段計畫

| 階段 | 內容 | 狀態 |
|---|---|---|
| P0 | 架構評估、schema、工具合約 | ✅ |
| P1 | 安全閘道 + 對抗性測試 | ✅ |
| P2 | `drone_agent` adapter + 11 個工具 + A→B 示範 | ✅ |
| P3 | MCP stdio 伺服器 + 往返測試 | ✅ |
| P4 | 能耗基準測試（基準線 vs 最佳化 vs 策略層） | ✅ |
| P5 | 真正的 LLM 策略層（同樣經過閘道） | 下一步 |
| P6 | U 形功率模型；風向（順風／逆風）；感測雜訊 | 下一步 |
| P7 | PX4 SITL + Gazebo adapter（Codespaces） | 未來 |
| P8 | Isaac Sim + PX4、ROS 2、AMR／機械手臂 adapter | 未來 |

## 10. 品質驗證方式

- **測試驅動**：閘道、工具與對抗性測試都先寫測試、確認失敗，再實作。MCP 伺服器是先寫程式後補測試。
- **突變測試**：在安全相關程式碼植入 15 個錯誤，**全部被抓到**。過程中找到 4 個測試缺口：都是「另一層防護擋住了錯誤，所以測試分辨不出第一層已經失效」。已補上針對各層的測試。
- **基準測試找到的真實 bug**：電量偏低時，`estimate_mission_energy` 會少算後半段航程，因為包絡在第一個電量不合格的航點就停止預估。已修正並加上回歸測試。
