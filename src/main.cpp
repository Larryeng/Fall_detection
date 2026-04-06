#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <FastLED.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <ESPmDNS.h>
#include <math.h>

/* ==========================================
 * [1] Hardware & External API Configuration
 * ------------------------------------------
 * 定義 ESP32-C3 的實體腳位、FastLED 控制陣列，
 * 以及 Discord Webhook 的推播網址。
 * ========================================== */
#define BUZZER_PIN 4
#define BUTTON_PIN 2
#define LED_PIN    1
#define NUM_LEDS   1
CRGB leds[NUM_LEDS];

const char* discordWebhook = "https://discord.com/api/webhooks/1419319374095061183/jKFMp8E0XEA2gXXhyqUGXU5fdKkKY_Wx9CCc8dzwY012saAubzivFLC7sxz_3YSPSwR-";

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
WebServer server(80);
const float G = 9.80665f;

/* ==========================================
 * [2] Heuristic Algorithm Thresholds
 * ------------------------------------------
 * 定義跌倒偵測狀態機的物理閾值。包含自由落下 (FreeFall)、
 * 衝擊 (Impact) 與靜止 (Inactivity) 的加速度與時間界線。
 * ========================================== */
float FF_G = 0.8f;                     // 放寬：容許手腕活動造成的低 G
uint32_t FF_MS = 60;                   // 放寬：下墜判定時間縮短
float IMP_G = 2.0f;                    // 放寬：衝擊判定降低
uint32_t IMP_MS = 10;                  // 放寬：衝擊維持時間極短化 (抓尖波)
float INACT_LOW = 0.5f;                
float INACT_HIGH = 1.5f;               
uint32_t INACT_MS = 500;               // 撞擊後強制等待的餘震緩衝時間
uint32_t MAX_FF_TO_IMPACT_MS = 1500;   
float ORIENT_DEG = 45.0f;              // 若手腕翻轉較小，可至 tune 頁面下修為 25~30
float IMP_HARD_G = 4.0f;               
uint32_t IMP_HARD_MS = 10;             
float EMA_ALPHA = 0.2f;                
int odr_hz = 200;                      

/* ==========================================
 * [3] Sliding Window & Variance Buffers
 * ------------------------------------------
 * 宣告環形緩衝區 (Ring Buffer)，用於儲存歷史姿態與加速度。
 * 確保系統能提取跌倒「前」的基準姿態，並計算絕對靜止的變異數。
 * ========================================== */
const int HIST_SIZE = 50;          
float histPitch[HIST_SIZE] = {0};
float histRoll[HIST_SIZE] = {0};
int histIdx = 0;

const int VAR_SIZE = 25;           
float varG[VAR_SIZE] = {1.0f};
int varIdx = 0;
float DELTA_G_THRESH = 0.40f;      // 放寬：容許人體呼吸造成的微顫
uint32_t lastBufferUpdate = 0;

/* ==========================================
 * [4] State Machine & Runtime Variables
 * ------------------------------------------
 * 系統核心有限狀態機 (FSM) 宣告、計時器、硬體狀態標記，
 * 以及前端儀表板顯示用的量測數據變數。
 * ========================================== */
enum FDState { Idle, FreeFall, Impact, PreAlarm };
FDState state = Idle;

float aG_ema = 1.0f;
float basePitch = 0.0f, baseRoll = 0.0f;
uint32_t tFFStart = 0, tImpactStart = 0, tInactStart = 0;
uint32_t preAlarmUntil = 0, blinkTicker = 0;
bool buzzerOn = false, ledOn = false;
bool wifiConnected = false;

float angRate = 0.0f;
float lastPitch = 0, lastRoll = 0;
uint32_t lastTs = 0;

/* ==========================================
 * [5] Hardware Helpers & Math Utility
 * ------------------------------------------
 * 提供三軸向量計算、姿態角轉換、I2C 掃描與警示元件控制等底層函式。
 * ========================================== */
static inline float vecMagG(const sensors_event_t& e) {
    float mps2 = sqrtf(e.acceleration.x * e.acceleration.x +
                       e.acceleration.y * e.acceleration.y +
                       e.acceleration.z * e.acceleration.z);
    return mps2 / G;
}

static inline void accelAngles(const sensors_event_t& e, float& pitch, float& roll) {
    float Ax = e.acceleration.x, Ay = e.acceleration.y, Az = e.acceleration.z;
    pitch = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az)) * 180.0f / PI;
    roll  = atan2f(Ay, Az) * 180.0f / PI;
}

static inline float angleDelta(float a, float b) {
    float d = fabsf(a - b);
    return (d > 180.0f) ? (360.0f - d) : d;
}

inline void beepOn()  { digitalWrite(BUZZER_PIN, LOW); }
inline void beepOff() { digitalWrite(BUZZER_PIN, HIGH); }

bool i2cPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

bool tryInitAccelPins(int sda, int scl) {
    Wire.end(); Wire.begin(sda, scl); delay(10);
    if (!(i2cPresent(0x53) || i2cPresent(0x1D))) return false;
    if (!accel.begin()) return false;
    Serial.printf("ADXL345 connected (SDA=%d, SCL=%d)\n", sda, scl);
    return true;
}

bool initAccelAuto() {
    const int cand[][2] = {{8, 9}, {7, 6}, {6, 7}};
    for (auto& p : cand) if (tryInitAccelPins(p[0], p[1])) return true;
    Wire.end(); Wire.begin();
    if ((i2cPresent(0x53) || i2cPresent(0x1D)) && accel.begin()) {
        Serial.println("ADXL345 connected (Default I2C)");
        return true;
    }
    return false;
}

void applyODR(int hz) {
    odr_hz = hz;
    dataRate_t rate = ADXL345_DATARATE_200_HZ;
    if      (hz >= 350) rate = ADXL345_DATARATE_400_HZ;
    else if (hz >= 180) rate = ADXL345_DATARATE_200_HZ;
    else                rate = ADXL345_DATARATE_100_HZ;
    accel.setDataRate(rate);
    Serial.printf("ADXL345 ODR set to ~%d Hz\n", hz);
}

void resetDetection() {
    state = Idle;
    tFFStart = tImpactStart = tInactStart = preAlarmUntil = blinkTicker = 0;
    buzzerOn = ledOn = false;
    leds[0] = CRGB::Black; FastLED.show(); beepOff();
    Serial.println("FSM Reset -> Idle");
}

void sendDiscord() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Warning: Wi-Fi offline, Webhook skipped");
        return;
    }
    HTTPClient http;
    http.begin(discordWebhook);
    http.addHeader("Content-Type", "application/json");
    const char* payload = R"({"username":"Fall Detection Bot","embeds":[{"title":"⚠️ 跌倒警報觸發！請立即查看配戴者狀況。","color":16711680,"image":{"url":"https://i.imgur.com/pTOOLNc.gif"}}]})";
    int code = http.POST(payload);
    Serial.printf("Webhook response code: %d\n", code);
    http.end();
}

/* ==========================================
 * [6] Web UI Templates (PROGMEM)
 * ------------------------------------------
 * 存放前端儀表板與參數調整頁面的原始碼。
 * 包含狀態中文化、CSV 資料錄製面板，宣告於 PROGMEM 節省 RAM。
 * ========================================== */
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="zh-Hant"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>ADXL345 跌倒偵測儀表板</title>
<style>
body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:16px;background:#0b0c10;color:#eaf0f6}
h1{font-size:20px;margin:0 0 12px;display:flex;align-items:center;flex-wrap:wrap;gap:8px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.card{background:#151a21;border:1px solid #2a313b;border-radius:12px;padding:12px}
.label{opacity:.7;font-size:12px;margin-bottom:4px;display:block}
.value{font-size:22px;font-variant-numeric:tabular-nums}
canvas{width:100%;height:160px;background:#0f1319;border:1px solid #2a313b;border-radius:8px}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Monaco,"Ubuntu Mono",Consolas,monospace}
.badge{display:inline-block;padding:4px 10px;border-radius:9999px;background:#243042;border:1px solid #2a313b;font-size:16px;font-weight:bold;transition:background-color 0.3s}
a.btn, button.btn{display:inline-block;padding:6px 12px;border-radius:8px;border:1px solid #2a313b;background:#1a2230;color:#cfe3ff;text-decoration:none;font-size:14px;cursor:pointer;font-family:inherit}
button.btn:hover{background:#243042}
.list{font-size:14px;line-height:1.45}
.kv{display:flex;gap:8px;align-items:center}
.k{opacity:.7;min-width:140px}

/* 錄製面板專用樣式 */
.rec-panel{display:flex;gap:8px;align-items:center;background:#1e1a15;border:1px solid #5a4220;padding:8px 12px;border-radius:8px;margin-bottom:12px;flex-wrap:wrap}
.rec-status{font-family:monospace;color:#f0ca86;margin-right:auto}
</style></head><body>

<h1>ADXL345 跌倒偵測系統 
  <span id="state" class="badge">連線中...</span> 
  <a class="btn" style="margin-left:auto" href="/tune">⚙️ 參數調整 (/tune)</a>
</h1>

<div class="rec-panel">
  <span id="recStatus" class="rec-status">⏸️ 就緒 (0 筆)</span>
  <button id="btnRec" class="btn" style="background:#2d5a3f;border-color:#3e7a55" onclick="toggleRecord()">▶️ 開始記錄</button>
  <button id="btnClear" class="btn" onclick="clearRecord()">🗑️ 清除</button>
  <button id="btnDl" class="btn" style="background:#284b7a;border-color:#38639e" onclick="download CSV()">⬇️ 下載 CSV</button>
</div>

<div class="grid">
  <div class="card"><div class="label">三軸合加速度 |a| (原始 / 平滑 EMA)</div><div id="gpair" class="value mono" style="color:#aee2ff">-- / -- g</div></div>
  <div class="card"><div class="label">角速度估算</div><div id="angrate" class="value mono" style="color:#d4b3ff">-- 度/秒</div></div>
  <div class="card"><div class="label">姿態角度 (Pitch / Roll)</div><div id="angles" class="value mono" style="color:#b3ffb3">-- 度</div></div>
  
  <div class="card"><div class="label">狀態計時器</div>
    <div class="list mono">
      <div class="kv"><div class="k">失重進行中 (FF)：</div><div id="ffms">0 秒</div></div>
      <div class="kv"><div class="k">衝擊進行中 (IMP)：</div><div id="impms">0 秒</div></div>
      <div class="kv"><div class="k">🚨 警報倒數：</div><div id="prems" style="color:#ff6b6b;font-weight:bold">0 秒</div></div>
    </div>
  </div>
  
  <div class="card"><div class="label">目前判定門檻 (Thresholds)</div>
    <div class="list mono">
      <div class="kv"><div class="k">自由落下 (G / 秒)：</div><div id="ff_pair">--</div></div>
      <div class="kv"><div class="k">著地衝擊 (G / 秒)：</div><div id="imp_pair">--</div></div>
      <div class="kv"><div class="k">靜止觀察 (G / 秒)：</div><div id="inact_pair">--</div></div>
      <div class="kv"><div class="k">躺平角度判定：</div><div id="ori_pair">--</div></div>
      <div class="kv"><div class="k">直接強衝擊後備：</div><div id="hard_pair">--</div></div>
    </div>
  </div>
</div>
<div class="card" style="margin-top:12px">
  <div class="label">加速度走勢圖 (最近 ~10 秒)</div>
  <canvas id="chart" width="900" height="200"></canvas>
</div>

<script>
const el=(id)=>document.getElementById(id);
const N=200; const data=new Array(N).fill(0); let idx=0;
const cvs=el('chart'); const ctx=cvs.getContext('2d');

// --- 錄製功能變數 ---
let isRecording = false;
let recordedRows = [];
let recStartTime = 0;

function toggleRecord() {
  isRecording = !isRecording;
  const btn = el('btnRec');
  const status = el('recStatus');
  
  if(isRecording) {
    if(recordedRows.length === 0) {
      // CSV 標頭加上明確單位 (秒, g, 度)
      recordedRows.push("Time(秒),State,aG_raw(g),aG_ema(g),Pitch(度),Roll(度)");
      recStartTime = Date.now();
    }
    btn.innerHTML = "⏸️ 暫停記錄";
    btn.style.background = "#992828";
    btn.style.borderColor = "#c43333";
    status.innerHTML = "🔴 記錄中...";
  } else {
    btn.innerHTML = "▶️ 繼續記錄";
    btn.style.background = "#2d5a3f";
    btn.style.borderColor = "#3e7a55";
    status.innerHTML = `⏸️ 已暫停 (共 ${recordedRows.length - 1} 筆)`;
  }
}

function clearRecord() {
  if(isRecording) toggleRecord(); 
  recordedRows = [];
  el('recStatus').innerHTML = "⏸️ 就緒 (0 筆)";
}

function downloadCSV() {
  if(recordedRows.length <= 1) {
    alert("沒有資料可以下載！請先按下「開始記錄」。");
    return;
  }
  const csvContent = recordedRows.join("\n");
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  const timestamp = new Date().toISOString().slice(0,19).replace(/:/g,"-");
  a.download = `fall_data_${timestamp}.csv`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

const stateMap = {
  'Idle':     { text: '🟢 正常監測中', bg: '#1c3a27', border: '#2d5a3f' },
  'FreeFall': { text: '⚠️ 偵測到跌倒 (失重)', bg: '#6b4f1b', border: '#997328' },
  'Impact':   { text: '💥 衝擊！姿勢判定中...', bg: '#6b2c1b', border: '#994028' },
  'PreAlarm': { text: '🚨 躺平靜止，警報發送倒數', bg: '#6b1b1b', border: '#992828' }
};

function draw(){
  const w=cvs.width,h=cvs.height;
  ctx.clearRect(0,0,w,h);
  ctx.globalAlpha=.5; ctx.beginPath(); ctx.moveTo(40,10); ctx.lineTo(40,h-20); ctx.lineTo(w-10,h-20);
  ctx.strokeStyle='#2a313b'; ctx.stroke();
  ctx.globalAlpha=1; ctx.beginPath();
  const maxG=4.0;
  for(let i=0;i<N;i++){const j=(idx+i)%N, val=data[j];
    const x=40+(i*(w-50)/(N-1)); const y=(h-20)-Math.min(val/maxG,1)*(h-30);
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle='#99c1ff'; ctx.lineWidth=2; ctx.stroke();
  ctx.fillStyle='#9aa6b2'; ctx.font='12px system-ui';
  for(let g=0;g<=4;g++){const y=(h-20)-(g/4)*(h-30); ctx.fillText(g+' g',5,y+4);
    ctx.beginPath(); ctx.moveTo(40,y); ctx.lineTo(w-10,y); ctx.strokeStyle='#1d232c'; ctx.stroke();}
}

async function tick(){
  try{
    const r=await fetch('/data',{cache:'no-store'}); const j=await r.json();
    
    // 寫入錄製資料 (時間改為 幾點幾秒 格式)
    if(isRecording) {
      const elapsedSec = ((Date.now() - recStartTime) / 1000).toFixed(3);
      recordedRows.push(`${elapsedSec},${j.state},${j.aG_raw},${j.aG_ema},${j.pitch},${j.roll}`);
      el('recStatus').innerHTML = `🔴 記錄中... (${recordedRows.length - 1} 筆)`;
    }

    const sInfo = stateMap[j.state] || { text: j.state, bg: '#243042', border: '#2a313b' };
    el('state').textContent = sInfo.text;
    el('state').style.backgroundColor = sInfo.bg;
    el('state').style.borderColor = sInfo.border;

    // 將所有顯示單位加上明確的 g, 度, 秒
    el('gpair').textContent=j.aG_raw.toFixed(3)+' g / '+j.aG_ema.toFixed(3)+' g';
    el('angrate').textContent=j.angRate.toFixed(1) + ' 度/秒';
    el('angles').textContent=j.pitch.toFixed(1)+'° / '+j.roll.toFixed(1)+'°';
    
    // 將毫秒換算為秒數顯示
    el('ffms').textContent=(j.ff_ms / 1000).toFixed(2) + ' 秒'; 
    el('impms').textContent=(j.impact_ms / 1000).toFixed(2) + ' 秒';
    el('prems').textContent=(j.prealarm_ms / 1000).toFixed(1) + ' 秒';
    
    // 閾值介面顯示換算
    el('ff_pair').textContent=j.base.FF_G.toFixed(2)+' g / '+(j.base.FF_MS / 1000).toFixed(2)+' 秒';
    el('imp_pair').textContent=j.base.IMP_G.toFixed(2)+' g / '+(j.base.IMP_MS / 1000).toFixed(2)+' 秒';
    el('inact_pair').textContent=j.base.INACT_LOW.toFixed(2)+'~'+j.base.INACT_HIGH.toFixed(2)+' g / '+(j.base.INACT_MS / 1000).toFixed(2)+' 秒';
    el('ori_pair').textContent=j.base.ORIENT_DEG.toFixed(0)+' 度';
    el('hard_pair').textContent=j.base.IMP_HARD_G.toFixed(2)+' g / '+(j.base.IMP_HARD_MS / 1000).toFixed(2)+' 秒';
    
    data[idx]=j.aG_raw; idx=(idx+1)%N; draw();
  }catch(e){console.log(e);}
}
setInterval(tick,200); draw();
</script></body></html>
)HTML";

const char TUNE_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="zh-Hant"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>Fall Detector - Tune</title>
<style>
body{font-family:system-ui,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:16px;background:#0b0c10;color:#eaf0f6}
h1{font-size:20px;margin:0 0 12px}
form{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.card{background:#151a21;border:1px solid #2a313b;border-radius:12px;padding:12px}
label{font-size:12px;opacity:.8;display:block;margin-bottom:4px;color:#aee2ff}
input{width:100%;padding:8px;background:#0f1319;border:1px solid #2a313b;border-radius:8px;color:#dbe8ff;box-sizing:border-box}
button{padding:10px 14px;border-radius:10px;border:1px solid #2a313b;background:#1a2230;color:#cfe3ff;cursor:pointer}
.row{grid-column:1/-1}
.badge{display:inline-block;padding:2px 8px;border-radius:9999px;background:#243042;border:1px solid #2a313b;margin-left:8px}
a{color:#9dc1ff}
</style></head><body>
<h1>參數調整 <span id="status" class="badge">--</span>　<a href="/">回到儀表板</a></h1>
<p style="font-size:12px;opacity:0.7">⚠️ 注意：時間單位為「毫秒」(1000毫秒 = 1秒)，請小心設定。</p>
<form id="f">
 <div class="card"><label>自由落下閾值 FF_G (g)</label><input step="0.01" type="number" name="FF_G"></div>
 <div class="card"><label>自由落下維持時間 FF_MS (毫秒)</label><input step="10" type="number" name="FF_MS"></div>
 <div class="card"><label>衝擊閾值 IMP_G (g)</label><input step="0.1" type="number" name="IMP_G"></div>
 <div class="card"><label>衝擊維持時間 IMP_MS (毫秒)</label><input step="5" type="number" name="IMP_MS"></div>
 <div class="card"><label>靜止下界 INACT_LOW (g)</label><input step="0.01" type="number" name="INACT_LOW"></div>
 <div class="card"><label>靜止上界 INACT_HIGH (g)</label><input step="0.01" type="number" name="INACT_HIGH"></div>
 <div class="card"><label>靜止觀察時間 INACT_MS (毫秒)</label><input step="10" type="number" name="INACT_MS"></div>
 <div class="card"><label>躺平姿態角度 ORIENT_DEG (度)</label><input step="1" type="number" name="ORIENT_DEG"></div>
 <div class="card"><label>失重至衝擊最大間隔 (毫秒)</label><input step="10" type="number" name="MAX_FF_TO_IMPACT_MS"></div>
 <div class="card"><label>強衝擊後備門檻 (g)</label><input step="0.1" type="number" name="IMP_HARD_G"></div>
 <div class="card"><label>強衝擊維持時間 (毫秒)</label><input step="1" type="number" name="IMP_HARD_MS"></div>
 <div class="card"><label>EMA 濾波係數 (0~1)</label><input step="0.05" type="number" name="EMA_ALPHA" min="0" max="1"></div>
 <div class="card"><label>感測器採樣率 ODR (Hz)</label><input step="100" type="number" name="odr"></div>
 <div class="row"><button type="submit">套用參數 (寫入 RAM)</button>　<button id="resetBtn" type="button">重置狀態機</button></div>
</form>
<script>
const f=document.getElementById('f'), statusEl=document.getElementById('status'), resetBtn=document.getElementById('resetBtn');
async function load(){
  const j=await (await fetch('/params')).json();
  for(const k in j){ if(f[k]) f[k].value=j[k]; }
  statusEl.textContent='已載入';
}
f.addEventListener('submit', async (e)=>{
  e.preventDefault();
  const fd=new FormData(f);
  const p=new URLSearchParams(); for(const [k,v] of fd.entries()) p.append(k,v);
  const r=await fetch('/params',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});
  statusEl.textContent=r.ok?'已套用':'失敗';
});
resetBtn.addEventListener('click', ()=>fetch('/reset'));
load();
</script>
</body></html>
)HTML";

/* ==========================================
 * [7] Web Server Handlers
 * ------------------------------------------
 * 處理 HTTP GET/POST 請求，包含動態數據拉取、參數寫入與狀態重置。
 * ========================================== */
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }
void handleTune() { server.send_P(200, "text/html", TUNE_HTML); }

void handleData() {
    sensors_event_t ev; accel.getEvent(&ev);
    float Ax = ev.acceleration.x, Ay = ev.acceleration.y, Az = ev.acceleration.z;
    float aG = sqrtf(Ax*Ax + Ay*Ay + Az*Az) / G;
    float pitch, roll; accelAngles(ev, pitch, roll);

    uint32_t now = millis();
    if (lastTs) {
        float dt = (now - lastTs) / 1000.0f;
        if (dt > 0) angRate = max(angleDelta(pitch, lastPitch) / dt, angleDelta(roll, lastRoll) / dt);
    }
    lastPitch = pitch; lastRoll = roll; lastTs = now;

    const char* s = (state == Idle) ? "Idle" : (state == FreeFall) ? "FreeFall" : (state == Impact) ? "Impact" : "PreAlarm";
    uint32_t ff_ms     = (tFFStart && aG < FF_G) ? (now - tFFStart) : 0;
    uint32_t impact_ms = (tImpactStart && aG > IMP_G) ? (now - tImpactStart) : 0;
    uint32_t pre_ms    = (state == PreAlarm && preAlarmUntil > now) ? (preAlarmUntil - now) : 0;

    String json; json.reserve(512); 
    json = "{";
    json += "\"state\":\"" + String(s) + "\",";
    json += "\"aG_raw\":" + String(aG, 6) + ",\"aG_ema\":" + String(aG_ema, 6) + ",";
    json += "\"x\":" + String(Ax, 6) + ",\"y\":" + String(Ay, 6) + ",\"z\":" + String(Az, 6) + ",";
    json += "\"pitch\":" + String(pitch, 3) + ",\"roll\":" + String(roll, 3) + ",";
    json += "\"angRate\":" + String(angRate, 3) + ","; 
    json += "\"ff_ms\":" + String(ff_ms) + ",\"impact_ms\":" + String(impact_ms) + ",\"prealarm_ms\":" + String(pre_ms) + ",";
    
    json += "\"base\":{";
    json += "\"FF_G\":" + String(FF_G, 3) + ",\"FF_MS\":" + String(FF_MS) + ",";
    json += "\"IMP_G\":" + String(IMP_G, 3) + ",\"IMP_MS\":" + String(IMP_MS) + ",";
    json += "\"INACT_LOW\":" + String(INACT_LOW, 3) + ",\"INACT_HIGH\":" + String(INACT_HIGH, 3) + ",\"INACT_MS\":" + String(INACT_MS) + ",";
    json += "\"ORIENT_DEG\":" + String(ORIENT_DEG, 1) + ",\"IMP_HARD_G\":" + String(IMP_HARD_G, 3) + ",\"IMP_HARD_MS\":" + String(IMP_HARD_MS);
    json += "}}";
    server.send(200, "application/json", json);
}

void handleParamsGet() {
    String s; s.reserve(256);
    s = "{";
    s += "\"FF_G\":" + String(FF_G, 3) + ",\"FF_MS\":" + String(FF_MS);
    s += ",\"IMP_G\":" + String(IMP_G, 3) + ",\"IMP_MS\":" + String(IMP_MS);
    s += ",\"INACT_LOW\":" + String(INACT_LOW, 3) + ",\"INACT_HIGH\":" + String(INACT_HIGH, 3) + ",\"INACT_MS\":" + String(INACT_MS);
    s += ",\"ORIENT_DEG\":" + String(ORIENT_DEG, 1) + ",\"MAX_FF_TO_IMPACT_MS\":" + String(MAX_FF_TO_IMPACT_MS);
    s += ",\"IMP_HARD_G\":" + String(IMP_HARD_G, 2) + ",\"IMP_HARD_MS\":" + String(IMP_HARD_MS);
    s += ",\"EMA_ALPHA\":" + String(EMA_ALPHA, 2) + ",\"odr\":" + String(odr_hz);
    s += "}";
    server.send(200, "application/json", s);
}

void handleParamsPost() {
    auto getF = [&](const String& name, float& var, float minv, float maxv) {
        if (!server.hasArg(name)) return;
        float v = server.arg(name).toFloat();
        if (!isnan(v)) var = constrain(v, minv, maxv);
    };
    auto getU = [&](const String& name, uint32_t& var, uint32_t minv, uint32_t maxv) {
        if (!server.hasArg(name)) return;
        long v = server.arg(name).toInt();
        var = (uint32_t)constrain(v, (long)minv, (long)maxv);
    };

    getF("FF_G", FF_G, 0.1f, 2.0f);          getU("FF_MS", FF_MS, 20, 2000);
    getF("IMP_G", IMP_G, 1.0f, 8.0f);        getU("IMP_MS", IMP_MS, 5, 500);
    getF("INACT_LOW", INACT_LOW, 0.1f, 2.0f); getF("INACT_HIGH", INACT_HIGH, 0.2f, 3.0f);
    if (INACT_HIGH < INACT_LOW) INACT_HIGH = INACT_LOW + 0.05f;
    getU("INACT_MS", INACT_MS, 200, 8000);
    getF("ORIENT_DEG", ORIENT_DEG, 10.0f, 120.0f);
    getU("MAX_FF_TO_IMPACT_MS", MAX_FF_TO_IMPACT_MS, 200, 8000);
    getF("IMP_HARD_G", IMP_HARD_G, 2.0f, 12.0f); getU("IMP_HARD_MS", IMP_HARD_MS, 5, 500);
    getF("EMA_ALPHA", EMA_ALPHA, 0.05f, 0.9f);

    if (server.hasArg("odr")) {
        int hz = constrain(server.arg("odr").toInt(), 100, 400);
        applyODR((hz < 150) ? 100 : (hz < 300) ? 200 : 400);
    }
    handleParamsGet();
}

void handleReset() {
    resetDetection();
    server.send(200, "text/plain", "reset");
}

/* ==========================================
 * [8] Main Initialization (Setup)
 * ------------------------------------------
 * 負責硬體通訊初始化、啟動 WiFiManager 配網，
 * 並配置 mDNS 與 Discord 開機推播機制。
 * ========================================== */
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis(); while (!Serial && millis() - t0 < 2000) { delay(10); }

    pinMode(BUZZER_PIN, OUTPUT); beepOff();
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    FastLED.addLeds<WS2812, LED_PIN, RGB>(leds, NUM_LEDS);
    leds[0] = CRGB::Black; FastLED.show();

    Serial.println("\nBooting Fall Detector...");

    WiFiManager wm;
    Serial.println("Starting WiFiManager AP...");
    bool res = wm.autoConnect("FallDetector_Setup");

    if (!res) {
        Serial.println("Failed to connect. Restarting...");
        delay(3000);
        ESP.restart();
    } 

    wifiConnected = true;
    Serial.printf("\nConnected. IP: %s\n", WiFi.localIP().toString().c_str());

    if (MDNS.begin("fall")) {
        Serial.println("mDNS started. http://fall.local");
    }

    HTTPClient http;
    http.begin(discordWebhook);
    http.addHeader("Content-Type", "application/json");
    String bootMsg = R"({"username":"Fall Detection Bot","content":"🟢 **系統開機上線**\n請點擊此連結進入儀表板：\n👉 http://)" + WiFi.localIP().toString() + R"(\n或使用蘋果/電腦原生網址：http://fall.local"})";
    int httpCode = http.POST(bootMsg);
    Serial.printf("Boot Notification sent, response code: %d\n", httpCode);
    http.end();

    if (!initAccelAuto()) {
        Serial.println("Fatal: ADXL345 init failed.");
        while (1) { delay(1000); } 
    }
    accel.setRange(ADXL345_RANGE_16_G);
    applyODR(odr_hz);

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/tune", handleTune);
    server.on("/params", HTTP_GET, handleParamsGet);
    server.on("/params", HTTP_POST, handleParamsPost);
    server.on("/reset", handleReset);
    server.begin(); 
}

/* ==========================================
 * [9] Main Event Loop
 * ------------------------------------------
 * 系統主迴圈。負責處理 Web 請求、維護 Wi-Fi 連線狀態、
 * 讀取感測器數值更新歷史緩衝區，並運行跌倒偵測有限狀態機。
 * ========================================== */
void loop() {
    server.handleClient();

    static uint32_t lastCheck = 0;
    uint32_t now = millis();
    if (now - lastCheck > 1000) {
        lastCheck = now;
        wl_status_t st = WiFi.status();
        if (st == WL_CONNECTED && !wifiConnected) {
            wifiConnected = true;
            Serial.printf("Wi-Fi restored. IP: %s\n", WiFi.localIP().toString().c_str());
        } else if (st != WL_CONNECTED && wifiConnected) {
            wifiConnected = false; Serial.println("Wi-Fi lost. ESP32 auto-reconnecting...");
        }
    }

    sensors_event_t ev; accel.getEvent(&ev);
    float aG = vecMagG(ev);                                     
    aG_ema = EMA_ALPHA * aG + (1 - EMA_ALPHA) * aG_ema;         
    float pitch, roll; accelAngles(ev, pitch, roll);

    if (now - lastBufferUpdate >= 20) {
        lastBufferUpdate = now;
        histPitch[histIdx] = pitch;
        histRoll[histIdx] = roll;
        histIdx = (histIdx + 1) % HIST_SIZE;

        varG[varIdx] = aG;
        varIdx = (varIdx + 1) % VAR_SIZE;
    }

    if (digitalRead(BUTTON_PIN) == LOW) { resetDetection(); return; }

    switch (state) {
        case Idle: {
            if (aG < FF_G) {
                if (!tFFStart) tFFStart = now;
                if (now - tFFStart >= FF_MS) {
                    int oldestIdx = (histIdx + 1) % HIST_SIZE; 
                    basePitch = histPitch[oldestIdx];
                    baseRoll = histRoll[oldestIdx];
                    state = FreeFall; tImpactStart = 0;
                    Serial.println("FSM: Idle -> FreeFall");
                }
            } else { tFFStart = 0; }

            static uint32_t tHardImp = 0;
            if (aG > IMP_HARD_G) {
                if (!tHardImp) tHardImp = now;
                if (now - tHardImp >= IMP_HARD_MS) {
                    int oldestIdx = (histIdx + 1) % HIST_SIZE;
                    basePitch = histPitch[oldestIdx];
                    baseRoll = histRoll[oldestIdx];
                    state = Impact; tImpactStart = now; 
                    Serial.println("FSM: Idle -> Impact (Bypass)");
                }
            } else { tHardImp = 0; }
            break;
        }

        case FreeFall: {
            if (aG > IMP_G) {
                if (!tImpactStart) tImpactStart = now;
                if (now - tImpactStart >= IMP_MS) {
                    state = Impact; 
                    Serial.println("FSM: FreeFall -> Impact");
                }
            } else {
                tImpactStart = 0;
                if (now - tFFStart > MAX_FF_TO_IMPACT_MS) resetDetection();
            }
            break;
        }

        case Impact: {
            float max_g = 0.0f, min_g = 10.0f;
            for (int i = 0; i < VAR_SIZE; i++) {
                if (varG[i] > max_g) max_g = varG[i];
                if (varG[i] < min_g) min_g = varG[i];
            }
            float delta_g = max_g - min_g;

            // 撞擊後強制等待 INACT_MS (500ms) 讓餘震結束，再開始檢查靜止
            if (now - tImpactStart >= INACT_MS) {
                if (aG_ema >= INACT_LOW && aG_ema <= INACT_HIGH && delta_g < DELTA_G_THRESH) {
                    float dPitch = angleDelta(pitch, basePitch);
                    float dRoll  = angleDelta(roll, baseRoll);
                    
                    if (max(dPitch, dRoll) >= ORIENT_DEG) {
                        Serial.println("FSM: Impact -> PreAlarm (Posture changed)");
                        state = PreAlarm; preAlarmUntil = now + 10000; 
                        blinkTicker = 0; buzzerOn = ledOn = false;
                    } else {
                        resetDetection();
                    }
                }
            }

            if (now - tImpactStart > 3000) resetDetection();
            break;
        }

        case PreAlarm: {
            if (now - blinkTicker >= 200) {
                blinkTicker = now;
                ledOn = !ledOn; buzzerOn = !buzzerOn;
                leds[0] = ledOn ? CRGB::Red : CRGB::Black; FastLED.show();
                buzzerOn ? beepOn() : beepOff();
            }
            if (now >= preAlarmUntil) {
                leds[0] = CRGB::Black; FastLED.show(); beepOff();
                sendDiscord();
                resetDetection();
            }
            break;
        }
    }
}