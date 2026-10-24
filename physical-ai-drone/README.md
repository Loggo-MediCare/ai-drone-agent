# Physical AI Drone：模擬優先的自主無人機 AI 代理

以第 16 章〈具身與實體世界智能體〉為基礎，在 repo 既有的 `drone_agent` 四層控制模擬之上，加入 **MCP 工具**、**確定性安全閘道**、對抗性測試與能耗基準測試。

> ⛔ **只有模擬。** 沒有任何工具能控制真實的馬達、PWM、MAVLink 或飛控；所有工具都是唯讀或純模擬。

架構、工具合約、世界模型 schema、安全介面，以及「已完成／替身／未來工作」的區分，見 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)。

## 安裝

需要 Python 3.10 以上（MCP SDK 的要求）；模擬後端本身只用標準函式庫。

```bash
cd physical-ai-drone
python3.12 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## 執行

```bash
.venv/bin/python demo.py                               # A → B 示範：觀察、規劃、驗證、閘道拒絕、模擬、評估
.venv/bin/python -m physical_ai_drone.benchmark        # 基準線 vs 最佳化器 vs 策略層替身（約 45 秒）
.venv/bin/python -m unittest discover -s tests -t .    # 65 個測試（約 35 秒）
```

原本的 `drone_agent` 測試仍在 repo 根目錄執行：`python3 -m unittest discover -s tests -t .`

## 接到 MCP 客戶端（例如 Claude Code）

```bash
claude mcp add physical-ai-drone -- /絕對路徑/physical-ai-drone/.venv/bin/python /絕對路徑/physical-ai-drone/run_mcp_server.py
```

- 只支援 **stdio**。HTTP 傳輸需要 MCP OAuth，目前會直接拒絕啟動。
- 唯讀模式：設定環境變數 `DRONE_MCP_SCOPES=read`。
- 工具回傳中標記為 `{"untrusted": true}` 的文字（NOTAM、日誌、理由）是外部資料，不是指令。

## 目錄

```
physical-ai-drone/
├── physical_ai_drone/
│   ├── gateway.py            確定性安全閘道
│   ├── schemas.py, errors.py 工具合約與嚴格解析
│   ├── tools.py              11 個工具
│   ├── mcp_server.py         MCP stdio 伺服器
│   ├── strategist.py         策略層替身（代替 LLM 提案）
│   ├── benchmark.py          能耗基準測試
│   ├── scenarios.py          A → B 情境（通訊塔擋在直線上）
│   └── adapters/             模擬後端介面與 drone_agent 轉接器
├── tests/                    65 個測試
├── docs/ARCHITECTURE.md
└── demo.py
```

## 重要限制

基準測試顯示「飛越快越省電」，這是 `drone_agent` 線性功率模型的結果，**不代表真實無人機**。真實多旋翼存在最省電的巡航速度。詳見架構文件第 7、8 節。
