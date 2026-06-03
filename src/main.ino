#include <stdint.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ========== PINS ==========
#define TRIG_PIN 20
#define ECHO_PIN 19
#define ADC_NGUON 15

#define LED_XANH 16
#define LED_VANG 17
#define LED_DO 18

#define ENA 8
#define ENB 9
#define motorInput1 7
#define motorInput2 6
#define motorInput3 5
#define motorInput4 4

int sensorPins[8] = { 1, 2, 3, 10, 11, 12, 13, 14 };

// ========== BATTERY ==========
#define VREF 3.3f
#define ADC_MAX 4095.0f
#define R1 10000.0f   // R2 trong schematic (10k)
#define R2 3300.0f    // R5 trong schematic (3.3k)
#define V_XANH 10.5f  // > 11.5V → xanh
#define V_VANG 9.0f  // 10.0 - 11.5V → vàng
                      // < 10.0V → đỏ

// ========== SENSOR ==========
uint32_t adcValue[8] = { 0 }, sensor = 0, d[8] = { 0 };
uint32_t avgAdc[8] = { 2225, 2039, 2343, 1977, 2367, 2123, 2635, 3109 };

// ========== PID ==========
float Kp = 2.200204, Ki = 0.000001, Kd = 4.170504;
float error = 0, P = 0, I = 0, D = 0;
float previous_error = 0, previous_I = 0;
float speed_pid = 0;
int baseSpeed = 200;

// ========== OBSTACLE ==========
long duration = 0, distance_cm = 0;
bool obstacleDetected = false;
#define OBSTACLE_DIST 20

// ========== MODE ==========
// 0 = Auto (dò line), 1 = Manual
int driveMode = 0;
// manual command: 0=stop,1=forward,2=back,3=left,4=right
int manualCmd = 0;

unsigned long tMatLine = 0; 
unsigned long tTram     = 0; 
uint8_t       stateTram = 0;

// ========== WEB SERVER ==========
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ========== STATUS ==========
int speedLeft_cur = 0, speedRight_cur = 0;
float batteryVolt = 0;
String ledStatus = "xanh";

// ===================================================
//  HTML nhúng thẳng vào firmware (Light Theme & HaUI Info)
// ===================================================
const char HTML_PAGE[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hệ Thống Điều Khiển AGV</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;600;700&family=Share+Tech+Mono&display=swap');

  :root {
    --bg: #f4f7f9; 
    --panel: #ffffff; 
    --border: #e2e8f0;
    --accent: #0f62fe; /* Xanh dương công nghiệp */
    --accent-hover: #0353e9;
    --warn: #f59e0b; 
    --ok: #10b981;
    --danger: #ef4444; 
    --text-main: #1e293b; 
    --text-dim: #64748b;
    --shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.05), 0 2px 4px -1px rgba(0, 0, 0, 0.03);
  }

  *{margin:0;padding:0;box-sizing:border-box;}
  body{
    background: var(--bg); color: var(--text-main);
    font-family: 'Inter', sans-serif;
    min-height: 100vh; padding: 16px;
  }

  /* --- HEADER THÔNG TIN --- */
  .header-card {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 12px; padding: 16px; margin-bottom: 20px;
    box-shadow: var(--shadow); text-align: center;
    border-top: 4px solid var(--accent);
  }
  .header-card h1 { font-size: 1.4rem; font-weight: 700; color: var(--accent); margin-bottom: 8px;}
  .header-card p { font-size: 0.9rem; color: var(--text-dim); line-height: 1.5;}
  .header-card strong { color: var(--text-main); }

  .ws-status {
    text-align: center; font-size: 0.8rem; font-weight: 600;
    color: var(--text-dim); margin-bottom: 16px; letter-spacing: 1px;
  }

  .grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;}
  @media(max-width:600px){.grid{grid-template-columns:1fr;}}

  .panel{
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; padding: 16px; box-shadow: var(--shadow);
  }
  .panel-title{
    font-size: 0.85rem; font-weight: 700; color: var(--text-dim);
    margin-bottom: 14px; text-transform: uppercase; letter-spacing: 0.5px;
    display: flex; align-items: center; gap: 6px;
  }
  .panel-title::before {
    content: ''; display: block; width: 4px; height: 14px;
    background: var(--accent); border-radius: 2px;
  }

  /* ── Sensors ── */
  .sensors{display:flex;gap:8px;justify-content:center;flex-wrap:wrap;}
  .sensor-cell{
    width: 32px; height: 32px; border-radius: 6px; border: 1px solid var(--border);
    background: #f8fafc; transition: all 0.2s;
    display: flex; align-items: center; justify-content: center;
    font-family: 'Share Tech Mono', monospace; font-size: 0.7rem; color: var(--text-dim);
  }
  .sensor-cell.on{
    background: var(--accent); color: white; border-color: var(--accent);
    box-shadow: 0 0 8px rgba(15, 98, 254, 0.4);
  }

  /* ── Motor bars ── */
  .motor-row{display:flex;align-items:center;gap:10px;margin-bottom:12px;}
  .motor-label{width:28px;font-size:0.85rem;font-weight:600;color:var(--text-dim);}
  .bar-track{
    flex:1;height:12px;background:#e2e8f0;border-radius:6px;overflow:hidden;
  }
  .bar-fill{
    height:100%;width:0%;transition:width 0.2s; background: var(--accent);
  }
  .motor-val{
    width:36px;text-align:right;font-family:'Share Tech Mono',monospace;
    font-size:0.85rem;font-weight:700;color:var(--text-main);
  }

  /* ── PID ── */
  .pid-row{display:flex;align-items:center;gap:8px;margin-bottom:10px;}
  .pid-row label{width:30px;font-size:0.85rem;font-weight:600;color:var(--text-dim);}
  .pid-row input{
    flex:1;background:#fff;border:1px solid var(--border); color:var(--text-main);
    border-radius:6px;padding:6px 10px;font-family:'Share Tech Mono',monospace;
    font-size:0.9rem;outline:none;transition:border 0.2s;
  }
  .pid-row input:focus{border-color:var(--accent);box-shadow: 0 0 0 3px rgba(15,98,254,0.1);}
  
  .btn-apply{
    width:100%;padding:10px;border:none;border-radius:6px;cursor:pointer;
    background:var(--accent);color:white;font-weight:600;font-size:0.9rem;
    transition:background 0.2s; margin-top: 6px;
  }
  .btn-apply:hover{background:var(--accent-hover);}

  /* ── Mode toggle ── */
  .mode-row{display:flex;gap:8px;margin-bottom:16px;}
  .mode-btn{
    flex:1;padding:10px;border:1px solid var(--border);border-radius:6px;
    background:#fff;color:var(--text-dim);cursor:pointer; font-weight:600;
    font-size:0.85rem; transition:all 0.2s;
  }
  .mode-btn.active{
    border-color:var(--accent);color:white; background:var(--accent);
  }

  /* ── D-pad ── */
  .dpad{
    display:grid;grid-template-columns:repeat(3,56px); grid-template-rows:repeat(3,56px);
    gap:8px;margin:0 auto;width:fit-content;
  }
  .dpad-btn{
    background:#fff;border:1px solid var(--border);border-radius:8px;
    color:var(--text-main);font-size:1.2rem;cursor:pointer;
    display:flex;align-items:center;justify-content:center;
    transition:all 0.1s;user-select:none;box-shadow: 0 2px 4px rgba(0,0,0,0.05);
  }
  .dpad-btn:active,.dpad-btn.pressed{
    background:var(--accent);color:#fff; transform: scale(0.95);
  }
  .dpad-btn.disabled{opacity:0.4;pointer-events:none;}

  /* ── Battery ── */
  .battery-wrap{display:flex;align-items:center;gap:12px;margin-bottom:10px;}
  .battery-body{
    flex:1;height:24px;background:#e2e8f0;border:2px solid #cbd5e1;
    border-radius:6px;overflow:hidden;position:relative;
  }
  .battery-fill{height:100%;width:0%;transition:width 0.5s,background 0.5s;}
  .battery-tip{
    width:6px;height:12px;background:#cbd5e1;border-radius:0 3px 3px 0; align-self:center;
  }
  .battery-volt{
    font-family:'Share Tech Mono',monospace;font-size:1rem;
    font-weight:700;color:var(--text-main);min-width:55px;text-align:right;
  }
  
  .dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:6px;}
  .dot.ok{background:var(--ok);}
  .dot.warn{background:var(--warn);}
  .dot.danger{background:var(--danger);}

  /* ── Error Bar ── */
  .error-val{
    font-size:2rem;font-family:'Share Tech Mono',monospace; font-weight:700;
    color:var(--text-main);text-align:center;
  }
  .error-bar-wrap{
    margin-top:12px;height:10px;background:#e2e8f0;border-radius:5px;overflow:hidden;
  }
  .error-bar{height:100%;width:50%;background:var(--ok);border-radius:5px;transition:width 0.15s;}

  /* ── MODAL VẬT CẢN (POPUP) ── */
  .modal-overlay {
    position: fixed; top: 0; left: 0; right: 0; bottom: 0;
    background: rgba(15, 23, 42, 0.8); backdrop-filter: blur(4px);
    display: flex; align-items: center; justify-content: center;
    z-index: 1000; opacity: 0; pointer-events: none; transition: opacity 0.3s;
  }
  .modal-overlay.show { opacity: 1; pointer-events: auto; }
  
  .modal-content {
    background: #fff; padding: 30px; border-radius: 16px; text-align: center;
    box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1);
    border: 4px solid var(--danger); animation: pulseBorder 1s infinite alternate;
    max-width: 90%; width: 340px;
  }
  @keyframes pulseBorder { from { border-color: #ef4444; box-shadow: 0 0 10px rgba(239,68,68,0.2); } to { border-color: #991b1b; box-shadow: 0 0 20px rgba(239,68,68,0.6); } }
  
  .modal-icon { font-size: 3rem; margin-bottom: 10px; }
  .modal-title { font-size: 1.4rem; font-weight: 700; color: var(--danger); margin-bottom: 8px;}
  .modal-desc { color: var(--text-dim); font-size: 0.95rem; margin-bottom: 16px; }
  .modal-dist { font-family: 'Share Tech Mono', monospace; font-size: 2rem; font-weight: 700; color: var(--text-main); }
</style>
</head>
<body>

<!-- Thông tin cá nhân -->
<div class="header-card">
  <h1>HỆ THỐNG XE TỰ HÀNH AGV</h1>
  <p>Thực hiện: <strong>Nguyễn Văn Bình</strong> - MSV: <strong>2022605460</strong></p>
  <p>Khoa Điện - Đại học Công nghiệp Hà Nội</p>
</div>

<div class="ws-status" id="wsStatus">⬤ ĐANG KẾT NỐI...</div>

<div class="grid">
  <!-- SENSOR -->
  <div class="panel">
    <div class="panel-title">Trạng thái Cảm biến</div>
    <div class="sensors" id="sensorCells">
      <div class="sensor-cell" data-i="0">S0</div>
      <div class="sensor-cell" data-i="1">S1</div>
      <div class="sensor-cell" data-i="2">S2</div>
      <div class="sensor-cell" data-i="3">S3</div>
      <div class="sensor-cell" data-i="4">S4</div>
      <div class="sensor-cell" data-i="5">S5</div>
      <div class="sensor-cell" data-i="6">S6</div>
      <div class="sensor-cell" data-i="7">S7</div>
    </div>
  </div>

  <!-- ERROR -->
  <div class="panel">
    <div class="panel-title">Sai số (PID Error)</div>
    <div class="error-val" id="errorVal">0.00</div>
    <div class="error-bar-wrap">
      <div class="error-bar" id="errorBar"></div>
    </div>
  </div>

  <!-- MOTOR -->
  <div class="panel">
    <div class="panel-title">Tốc độ Động cơ</div>
    <div class="motor-row">
      <span class="motor-label">Trái</span>
      <div class="bar-track"><div class="bar-fill" id="barL"></div></div>
      <span class="motor-val" id="valL">0</span>
    </div>
    <div class="motor-row">
      <span class="motor-label">Phải</span>
      <div class="bar-track"><div class="bar-fill" id="barR"></div></div>
      <span class="motor-val" id="valR">0</span>
    </div>
  </div>

  <!-- BATTERY -->
  <div class="panel">
    <div class="panel-title">Nguồn Điện</div>
    <div class="battery-wrap">
      <div class="battery-body">
        <div class="battery-fill" id="batFill"></div>
      </div>
      <div class="battery-tip"></div>
      <div class="battery-volt" id="batVolt">0.0V</div>
    </div>
    <div style="font-size:0.85rem; color: var(--text-dim); font-weight: 600;">
      <span class="dot" id="ledDot"></span>
      <span id="ledLabel">--</span>
    </div>
  </div>

  <!-- CONTROL -->
  <div class="panel">
    <div class="panel-title">Chế độ Điều khiển</div>
    <div class="mode-row">
      <button class="mode-btn active" id="btnAuto" onclick="setMode(0)">TỰ ĐỘNG</button>
      <button class="mode-btn" id="btnManual" onclick="setMode(1)">THỦ CÔNG</button>
    </div>
    <div class="dpad" id="dpad">
      <div></div>
      <div class="dpad-btn disabled" id="dFwd" onmousedown="cmd(1)" onmouseup="cmd(0)"
           ontouchstart="cmd(1)" ontouchend="cmd(0)">▲</div>
      <div></div>
      <div class="dpad-btn disabled" id="dLeft" onmousedown="cmd(3)" onmouseup="cmd(0)"
           ontouchstart="cmd(3)" ontouchend="cmd(0)">◀</div>
      <div class="dpad-btn disabled" id="dStop" onclick="cmd(0)">■</div>
      <div class="dpad-btn disabled" id="dRight" onmousedown="cmd(4)" onmouseup="cmd(0)"
           ontouchstart="cmd(4)" ontouchend="cmd(0)">▶</div>
      <div></div>
      <div class="dpad-btn disabled" id="dBack" onmousedown="cmd(2)" onmouseup="cmd(0)"
           ontouchstart="cmd(2)" ontouchend="cmd(0)">▼</div>
      <div></div>
    </div>
  </div>

  <!-- PID TUNE -->
  <div class="panel">
    <div class="panel-title">Cấu hình Thông số</div>
    <div class="pid-row"><label>Kp</label><input type="number" id="inKp" step="0.01"></div>
    <div class="pid-row"><label>Ki</label><input type="number" id="inKi" step="0.000001"></div>
    <div class="pid-row"><label>Kd</label><input type="number" id="inKd" step="0.01"></div>
    <div class="pid-row">
      <label>Tốc</label>
      <input type="number" id="inSpd" step="1" min="0" max="255">
    </div>
    <button class="btn-apply" onclick="applyPID()">CẬP NHẬT</button>
  </div>
</div>

<!-- Modal Vật Cản -->
<div class="modal-overlay" id="obstacleModal">
  <div class="modal-content">
    <div class="modal-icon">🛑</div>
    <div class="modal-title">CẢNH BÁO VẬT CẢN</div>
    <div class="modal-desc">Hệ thống đã tự động dừng xe để đảm bảo an toàn. Mất vật cản xe sẽ tiếp tục.</div>
    <div class="modal-dist" id="modalDistVal">-- cm</div>
  </div>
</div>

<script>
let ws, mode = 0;

function connect(){
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = ()=>{
    const st = document.getElementById('wsStatus');
    st.innerHTML = '⬤ ĐÃ KẾT NỐI ESP32';
    st.style.color = 'var(--ok)';
    requestData();
  };
  ws.onclose = ()=>{
    const st = document.getElementById('wsStatus');
    st.innerHTML = '⬤ MẤT KẾT NỐI - ĐANG THỬ LẠI...';
    st.style.color = 'var(--danger)';
    setTimeout(connect, 2000);
  };
  ws.onmessage = e => {
    const d = JSON.parse(e.data);
    if(d.type === 'state') updateUI(d);
    if(d.type === 'pid'){
      document.getElementById('inKp').value = d.Kp;
      document.getElementById('inKi').value = d.Ki;
      document.getElementById('inKd').value = d.Kd;
      document.getElementById('inSpd').value = d.spd;
    }
  };
}

function requestData(){
  if(ws && ws.readyState===1) ws.send(JSON.stringify({cmd:'getPID'}));
}

function updateUI(d){
  // Sensors
  const cells = document.querySelectorAll('.sensor-cell');
  for(let i=0;i<8;i++){
    cells[i].classList.toggle('on', !!(d.sensor & (1<<i)));
  }
  
  // Error bar
  document.getElementById('errorVal').textContent = d.error.toFixed(2);
  const pct = 50 + (d.error / 20) * 50;
  document.getElementById('errorBar').style.width = pct + '%';
  const ec = d.error > 5 ? 'var(--warn)' : d.error < -5 ? 'var(--accent)' : 'var(--ok)';
  document.getElementById('errorBar').style.background = ec;
  
  // Motors
  document.getElementById('barL').style.width = (d.sL/255*100)+'%';
  document.getElementById('barR').style.width = (d.sR/255*100)+'%';
  document.getElementById('valL').textContent = d.sL;
  document.getElementById('valR').textContent = d.sR;
  
  // Battery
  const vMin=9.0, vMax=12.6;
  const pctBat = Math.max(0,Math.min(100,(d.volt-vMin)/(vMax-vMin)*100));
  document.getElementById('batFill').style.width = pctBat+'%';
  document.getElementById('batFill').style.background =
    d.led==='xanh'?'var(--ok)': d.led==='vang'?'var(--warn)':'var(--danger)';
  document.getElementById('batVolt').textContent = d.volt.toFixed(1)+'V';
  
  const dot = document.getElementById('ledDot');
  dot.className = 'dot ' + (d.led==='xanh'?'ok': d.led==='vang'?'warn':'danger');
  document.getElementById('ledLabel').textContent =
    d.led==='xanh'?'ĐẦY PIN': d.led==='vang'?'GẦN HẾT':'HẾT PIN';
    
  // Obstacle Popup Logic
  const modal = document.getElementById('obstacleModal');
  document.getElementById('modalDistVal').textContent = d.dist + ' cm';
  if(d.obs === 1) {
    modal.classList.add('show');
  } else {
    modal.classList.remove('show');
  }
}

function setMode(m){
  mode = m;
  document.getElementById('btnAuto').classList.toggle('active', m===0);
  document.getElementById('btnManual').classList.toggle('active', m===1);
  const btns = document.querySelectorAll('.dpad-btn');
  btns.forEach(b => b.classList.toggle('disabled', m===0));
  if(ws && ws.readyState===1) ws.send(JSON.stringify({cmd:'mode', val:m}));
}

function cmd(c){
  if(mode!==1) return;
  if(ws && ws.readyState===1) ws.send(JSON.stringify({cmd:'drive', val:c}));
}

function applyPID(){
  const payload = {
    cmd:'pid',
    Kp: parseFloat(document.getElementById('inKp').value),
    Ki: parseFloat(document.getElementById('inKi').value),
    Kd: parseFloat(document.getElementById('inKd').value),
    spd: parseInt(document.getElementById('inSpd').value)
  };
  if(ws && ws.readyState===1) ws.send(JSON.stringify(payload));
}

connect();
setInterval(()=>{ if(ws&&ws.readyState===1) ws.send(JSON.stringify({cmd:'ping'})); }, 2000);
</script>
</body>
</html>
)rawhtml";

// ===================================================
unsigned long lastWsSend = 0;

void sendState() {
  if (ws.count() == 0) return;
  StaticJsonDocument<256> doc;
  doc["type"] = "state";
  doc["sensor"] = sensor;
  doc["error"] = error;
  doc["sL"] = speedLeft_cur;
  doc["sR"] = speedRight_cur;
  doc["volt"] = batteryVolt;
  doc["led"] = ledStatus;
  doc["dist"] = distance_cm;
  doc["obs"] = obstacleDetected ? 1 : 0;
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* c,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (info->opcode != WS_TEXT) return;
  data[len] = 0;
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, (char*)data)) return;
  const char* cmd = doc["cmd"];

  if (strcmp(cmd, "mode") == 0) {
    driveMode = doc["val"];
    if (driveMode == 0) manualCmd = 0;
  } else if (strcmp(cmd, "drive") == 0) {
    if (driveMode == 1) manualCmd = doc["val"];
  } else if (strcmp(cmd, "pid") == 0) {
    Kp = doc["Kp"];
    Ki = doc["Ki"];
    Kd = doc["Kd"];
    baseSpeed = doc["spd"];
  } else if (strcmp(cmd, "getPID") == 0) {
    StaticJsonDocument<128> rep;
    rep["type"] = "pid";
    rep["Kp"] = Kp;
    rep["Ki"] = Ki;
    rep["Kd"] = Kd;
    rep["spd"] = baseSpeed;
    String out;
    serializeJson(rep, out);
    c->text(out);
  }
}

// ===================================================
void setup() {
  Serial.begin(115200);

  // Motor
  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Sensor
  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // LED
  pinMode(LED_XANH, OUTPUT);
  pinMode(LED_VANG, OUTPUT);
  pinMode(LED_DO, OUTPUT);

  // WiFi - Khởi tạo chế độ Access Point (Phát WiFi riêng)
  WiFi.setSleep(false); // Chống lag
  Serial.println("\nĐang khởi tạo WiFi AP...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("AGV_BINH", "12345678"); // Tên WiFi và Mật khẩu do xe phát ra
  
  Serial.print("Phát WiFi thành công! Truy cập IP: ");
  Serial.println(WiFi.softAPIP()); // Thường là 192.168.4.1

  // WebSocket
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send_P(200, "text/html", HTML_PAGE);
  });
  server.begin();
}

// ===================================================
void checkBattery() {
  int raw = analogRead(ADC_NGUON);
  float vAdc = raw / ADC_MAX * VREF;
  batteryVolt = vAdc * (R1 + R2) / R2;

  if (batteryVolt >= V_XANH) {
    ledStatus = "xanh";
    digitalWrite(LED_XANH, HIGH);
    digitalWrite(LED_VANG, LOW);
    digitalWrite(LED_DO, LOW);
  } else if (batteryVolt >= V_VANG) {
    ledStatus = "vang";
    digitalWrite(LED_XANH, LOW);
    digitalWrite(LED_VANG, HIGH);
    digitalWrite(LED_DO, LOW);
  } else {
    ledStatus = "do";
    digitalWrite(LED_XANH, LOW);
    digitalWrite(LED_VANG, LOW);
    digitalWrite(LED_DO, HIGH);
  }
}

long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Giảm timeout xuống 5000us (5ms) để tránh treo ESP32
  long dur = pulseIn(ECHO_PIN, HIGH, 5000);  
  if (dur == 0) return 999; // Không thấy vật cản trả về 999cm để xe không dừng sai
  
  return (dur * 0.034) / 2;
}

void readSensor() {
  for (uint8_t i = 0; i < 8; i++) {
    adcValue[i] = analogRead(sensorPins[i]);
    d[i] = (adcValue[i] > avgAdc[i]) ? 1 : 0;
  }
  sensor = (d[0] << 0) | (d[1] << 1) | (d[2] << 2) | (d[3] << 3) | (d[4] << 4) | (d[5] << 5) | (d[6] << 6) | (d[7] << 7);
}

void nhanBietLine() {
  readSensor();
  switch (sensor) {
    case 63: case 31: case 15: case 1: error = -20; break;
    case 248: case 240: case 128: error = 20; break;
    case 252: error = 20; break;
    case 7: error = -8; break;
    case 5: error = -7; break;
    case 3: error = -9; break;
    case 2: error = -7; break;
    case 14: error = -4; break;
    case 10: error = -4; break;
    case 6: error = -6; break;
    case 4: error = -4; break;
    case 20: error = -2; break;
    case 28: error = -2; break;
    case 12: error = -3; break;
    case 8: error = -1; break;
    case 24: error = 0; break;
    case 16: error = 1; break;
    case 48: error = 3; break;
    case 40: error = 2; break;
    case 56: error = 2; break;
    case 32: error = 4; break;
    case 96: error = 6; break;
    case 80: error = 4; break;
    case 112: error = 4; break;
    case 64: error = 7; break;
    case 192: error = 9; break;
    case 160: error = 7; break;
    case 224: error = 8; break;
    case 0: error = 200; break;
  }
}

void calculate_pid() {
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  speed_pid = (Kp * P) + (Ki * I) + (Kd * D);
  previous_I = I;
  previous_error = error;
}

void controlMotor() {
  speedLeft_cur = constrain(baseSpeed - speed_pid, 0, 255);
  speedRight_cur = constrain(baseSpeed + speed_pid, 0, 255);
  analogWrite(ENA, speedLeft_cur);
  analogWrite(ENB, speedRight_cur);
  forward();
}

void runManual() {
  switch (manualCmd) {
    case 1:
      analogWrite(ENA, 180); analogWrite(ENB, 180); forward(); break;
    case 2:
      analogWrite(ENA, 180); analogWrite(ENB, 180); back(); break;
    case 3:
      analogWrite(ENA, 180); analogWrite(ENB, 180); sharpLeftTurn(); break;
    case 4:
      analogWrite(ENA, 180); analogWrite(ENB, 180); sharpRightTurn(); break;
    default:
      Stop(); speedLeft_cur = 0; speedRight_cur = 0; break;
  }
}

// ===================================================

void loop() {
  unsigned long now = millis();
  static unsigned long tBat = 0, tObs = 0;

  // 1. Đo pin định kỳ 200ms
  if (now - tBat > 20000) {
    checkBattery();
    tBat = now;
  }

  // 2. Đo vật cản định kỳ 80ms
  if (now - tObs > 80) {
    distance_cm = measureDistance();
    obstacleDetected = (distance_cm > 0 && distance_cm < OBSTACLE_DIST);
    tObs = now;
  }

  // 3. LOGIC ĐIỀU KHIỂN
  if (obstacleDetected) {
    Stop();
    speedLeft_cur = 0;
    speedRight_cur = 0;

  } else if (driveMode == 1) {
    runManual();

  } else {
    // ==== AUTO (Dò Line) ====
    nhanBietLine();

    if (sensor == 255) {
      // ---- VẠCH NGANG: tất cả 8 cảm biến thấy đen ----
      tMatLine = 0;

      if (stateTram == 0) {
        Stop();
        speedLeft_cur  = 0;
        speedRight_cur = 0;
        tTram     = now;
        stateTram = 1;
      }
      else if (stateTram == 1) {
        Stop();
        speedLeft_cur  = 0;
        speedRight_cur = 0;
        if (now - tTram >= 5000) {
          error          = 0;
          previous_error = 0;
          speed_pid      = 0;
          I = 0; previous_I = 0;
          tTram     = now;
          stateTram = 2;
        }
      }
      else if (stateTram == 2) {
        // Vẫn còn trên vạch ngang → tiếp tục tiến
        analogWrite(ENA, baseSpeed);
        analogWrite(ENB, baseSpeed);
        forward();
        speedLeft_cur  = baseSpeed;
        speedRight_cur = baseSpeed;
      }

    } else if (stateTram == 2) {
      // ---- ĐANG THOÁT TRẠM: tiến thẳng cho đến khi thấy line dọc ----
      analogWrite(ENA, baseSpeed);
      analogWrite(ENB, baseSpeed);
      forward();
      speedLeft_cur  = baseSpeed;
      speedRight_cur = baseSpeed;

      if (sensor != 0) {
        // Đã thấy line dọc → reset trạm, trả về PID
        stateTram      = 0;
        error          = 0;
        previous_error = 0;
        I = 0; previous_I = 0;
        tMatLine = 0;
      }

    } else if (sensor == 0) {
      // ---- MẤT LINE THẬT SỰ ----
      if (tMatLine == 0) tMatLine = now;
      if (now - tMatLine > 100) {
        Stop();
        speedLeft_cur  = 0;
        speedRight_cur = 0;
      }

    } else {
      // ---- DÒ LINE BÌNH THƯỜNG ----
      tMatLine  = 0;
      stateTram = 0;

      if (previous_error == 20 || error == 20) {
        do {
          sharpRightTurn();
          analogWrite(ENA, 150);
          analogWrite(ENB, 150);
          readSensor();
        } while (sensor == 24);
      } else if (previous_error == -20 || error == -20) {
        do {
          sharpLeftTurn();
          analogWrite(ENA, 150);
          analogWrite(ENB, 150);
          readSensor();
        } while (sensor == 24);
      } else {
        calculate_pid();
        controlMotor();
      }
    }
  }

  // 4. Gửi WebSocket 250ms/lần
  if (now - lastWsSend > 250) {
    sendState();
    lastWsSend = now;
  }

  // 5. Dọn client cũ
  ws.cleanupClients();
}

// ===================================================
void forward() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void back() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpLeftTurn() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, HIGH);
}
void sharpRightTurn() {
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, LOW);
}
void Stop() {
  digitalWrite(motorInput1, LOW);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  digitalWrite(motorInput4, LOW);
}
