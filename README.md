# 自主無人機 AI 代理（AI Drone Agent）

以《每位人工智慧工程師必須建構的 30 個智能體》第十六章〈具身與實體世界智能體〉為基礎，把書中的程式片段擴充成**可以完整執行的四層控制無人機任務模擬**：

| 層級 | 頻率 | 本專案實作 | 檔案 |
|---|---|---|---|
| 任務規劃層 | 0.1–1 Hz | 統一約束包絡（天氣／空域／電池／綠帶／任務幾何）、改道、返航 | `constraints.py`、`planner.py`、`supervisor.py` |
| 運動規劃層 | 1–10 Hz（2 Hz） | **RRT\***：從目前位置到航點的無碰撞路徑，避開實體障礙與禁航區 | `motion.py` |
| 軌跡控制層 | 50–200 Hz（100 Hz） | 路徑時間參數化成設定點，**PID** 計算推力向量 | `trajectory.py` |
| 伺服控制層 | 1–10 kHz（1 kHz） | 四顆馬達**電流 PI 迴路**，抑制匯流排電壓漣波干擾 | `servo.py` |

核心原則（第十六章）：**安全不是正確規劃的副產品，而是對可執行動作集合 A_safe(s) 的明確限制。** LLM／規劃器只能「提案」，能不能飛由程式碼決定。

純 Python 標準函式庫即可執行，不需 GPU、無人機或 API 金鑰。

## 執行

```
python demo.py            # 9 個任務情境（每個完整任務約 30 秒模擬時間）
python demo.py 6 8        # 只跑指定情境
python -m unittest discover -s tests -t . -v   # 18 個測試
python calibrate.py       # 量測懸停/巡航功率，校正電池估算
```

選用：讓 GPT 擔任規劃器（安全層仍由程式碼強制）

```
pip install -r requirements-llm.txt
copy .env.example .env    # 填入 OPENAI_API_KEY
python demo.py --llm
```

## 情境與結果（任務：起降點 → 6 km 走廊 4 個測量點 → 返航，書中渥太華一月條件 -6.2°C、風 18.5 km/h、電量 82%）

| 情境 | 結果 | 展示的機制 |
|---|---|---|
| 1. 正常任務 | `COMPLETED`，82% → 45% | 通訊塔擋在 WP3→WP4 與返航路線上，RRT* 自動繞過 |
| 2. 氣溫 -12°C | `NO_GO` | WEATHER=RED，馬達不解鎖 |
| 3. 起飛電量 25% | `NO_GO` | BATTERY=RED（起飛 < 30%） |
| 4. 綠帶未授權 | `NO_GO` | PARKS=RED，且綠帶橫跨走廊無法繞行 |
| 5. 天氣資料 20 分鐘未更新 | `NO_GO` | WEATHER=STALE，保守融合視同失敗 |
| 6. 飛行中 NOTAM 臨時禁航 | `COMPLETED_REROUTED` | NOTAM 推播 → 包絡轉紅 → 影響傳播 → 改道繞行 |
| 7. NOTAM 橫跨整條走廊 | `RETURNED_HOME` | 無可行改道 → 返航 |
| 8. 陣風 32 km/h 40 秒 | `COMPLETED`，懸停 1 次 | 軌跡層拒絕推進、PID 原地抗風，陣風減弱 5 秒後恢復 |
| 9. 規劃器提出 80 km/h 超速航線 | `COMPLETED` | 安全層拒絕 → 原因回饋 → 重新規劃 |

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

## 下一步

1. 把 `SafetyMonitor` 與任務層接上 **PX4 / ArduPilot SITL**，航點改用 MAVLink 下達（取代 `servo.py`、`physics.py`）。
2. 天氣改接中央氣象署開放資料；空域限制改接民航局無人機相關公告。
3. 運動層加入動態障礙物（其他飛行器、鳥群），RRT* 改為增量式重規劃。
4. 以 NVIDIA Isaac Sim 做視覺感知，更新世界模型的信念狀態 b(s)。
