#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char* ssid = "XFDecoX55";
const char* password = "JollyCartoonB16";

WebServer server(80);
Preferences prefs;

// Left Motor
const int ENA = 13;
const int IN1 = 18;
const int IN2 = 19;

// Right Motor
const int ENB = 12;
const int IN3 = 16;
const int IN4 = 17;

// MPU6050
#define MPU_ADDR 0x68
#define SDA_PIN 21
#define SCL_PIN 22

int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

float accAngle = 0;
float gyroAngle = 0;
float filteredAngle = 0;
float gyroRate = 0;

float gyroBiasX = 0;
float angleOffset = 0;

unsigned long lastTime = 0;

float compAlpha = 0.96; // 0-1, higher = trust gyro more

// PID
float kp = 8.0;
float ki = 0.0;
float kd = 0.3;

float setpoint = 0;
float pidOutput = 0;
float errorSum = 0;
float lastError = 0;
float filteredDError = 0;

bool pidEnabled = false;
bool wasTipped = false;
bool invertOutput = false;

// Motor trim
int leftTrim = 0;
int rightTrim = 0;
bool reverseLeft = false;
bool reverseRight = false;

// Graph buffer
#define HIST_LEN 100
float angleHist[HIST_LEN];
float outputHist[HIST_LEN];
int histIndex = 0;
unsigned long lastSampleTime = 0;
const int sampleIntervalMs = 30;

// =====================================================
//                    MPU6050 RAW I2C
// =====================================================

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpuInit() {
  mpuWrite(0x6B, 0x00); // wake up
  delay(100);
  mpuWrite(0x1B, 0x00); // gyro +-250 deg/s
  mpuWrite(0x1C, 0x00); // accel +-2g
}

bool mpuReadRaw() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  uint8_t bytesReceived = Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (bytesReceived < 14) return false;

  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();
  accZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // skip temp
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();

  return true;
}

void calibrateGyro() {
  long gyroSum = 0;
  float accAngleSum = 0;
  const int samples = 500;
  for (int i = 0; i < samples; i++) {
    mpuReadRaw();
    gyroSum += gyroX;
    accAngleSum += atan2((float)accY, (float)accZ) * 180.0 / PI;
    delay(2);
  }
  gyroBiasX = (float)gyroSum / samples;
  angleOffset = accAngleSum / samples;
}

// =====================================================
//                    MOTOR CONTROL
// =====================================================

void setMotor(int ena, int a, int b, int pwm, bool reverse) {
  pwm = constrain(pwm, 0, 255);

  if (reverse) {
    digitalWrite(a, LOW);
    digitalWrite(b, HIGH);
  } else {
    digitalWrite(a, HIGH);
    digitalWrite(b, LOW);
  }

  ledcWrite(ena, pwm);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

int minPWM = 60; // lowest PWM that actually spins the wheels from a stop

void driveFromPID(float value) {
  bool goingForward = value >= 0;
  int pwm = (int)fabs(value);
  pwm = constrain(pwm, 0, 255);

  if (pwm < 3) {
    stopMotors();
    return;
  }

  if (pwm < minPWM) pwm = minPWM; // deadband compensation

  int leftPwm = constrain(pwm + leftTrim, 0, 255);
  int rightPwm = constrain(pwm + rightTrim, 0, 255);

  if (goingForward) {
    setMotor(ENA, IN1, IN2, leftPwm, reverseLeft);
    setMotor(ENB, IN3, IN4, rightPwm, reverseRight);
  } else {
    setMotor(ENA, IN2, IN1, leftPwm, reverseLeft);
    setMotor(ENB, IN4, IN3, rightPwm, reverseRight);
  }
}

// =====================================================
//                    SETTINGS SAVE/LOAD
// =====================================================

void saveSettings() {
  prefs.begin("pidcfg", false);
  prefs.putFloat("kp", kp);
  prefs.putFloat("ki", ki);
  prefs.putFloat("kd", kd);
  prefs.putFloat("setpoint", setpoint);
  prefs.putFloat("alpha", compAlpha);
  prefs.putInt("ltrim", leftTrim);
  prefs.putInt("rtrim", rightTrim);
  prefs.putBool("revl", reverseLeft);
  prefs.putBool("revr", reverseRight);
  prefs.putBool("invert", invertOutput);
  prefs.putInt("minpwm", minPWM);
  prefs.end();
}

void loadSettings() {
  prefs.begin("pidcfg", true);
  kp = prefs.getFloat("kp", kp);
  ki = prefs.getFloat("ki", ki);
  kd = prefs.getFloat("kd", kd);
  setpoint = prefs.getFloat("setpoint", setpoint);
  compAlpha = prefs.getFloat("alpha", compAlpha);
  leftTrim = prefs.getInt("ltrim", leftTrim);
  rightTrim = prefs.getInt("rtrim", rightTrim);
  reverseLeft = prefs.getBool("revl", reverseLeft);
  reverseRight = prefs.getBool("revr", reverseRight);
  invertOutput = prefs.getBool("invert", invertOutput);
  minPWM = prefs.getInt("minpwm", minPWM);
  prefs.end();
}

// =====================================================
//                    WEB PAGE
// =====================================================

void handleRoot() {

String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Self-Balancing Robot</title>
<style>
body{font-family:Arial;background:#1c1c1c;color:#eee;text-align:center;margin:0;padding:10px;}
h1{font-size:22px;}
h2{font-size:16px;margin:10px 0 4px;}
.card{background:#2a2a2a;border-radius:10px;padding:12px;margin:10px auto;max-width:480px;}
.slider{width:90%;}
button{width:130px;height:50px;font-size:18px;margin:6px;border-radius:8px;border:none;}
#startBtn{background:#2ecc71;color:#fff;}
#stopBtn{background:#e74c3c;color:#fff;}
#saveBtn{background:#3498db;color:#fff;}
canvas{background:#111;border-radius:8px;width:95%;height:200px;}
.row{display:flex;justify-content:space-between;max-width:480px;margin:auto;}
.val{font-weight:bold;color:#5dade2;}
label{display:block;margin-top:6px;}
</style>
</head>
<body>

<h1>Self-Balancing Robot Tuner</h1>

<div class="card">
  <canvas id="graph" width="460" height="200"></canvas>
  <div>Angle: <span id="angleVal" class="val">0</span> deg &nbsp; | &nbsp; Output: <span id="outVal" class="val">0</span></div>
</div>

<div class="card">
  <button id="startBtn" onclick="setPid(true)">START</button>
  <button id="stopBtn" onclick="setPid(false)">STOP</button>
  <button id="saveBtn" onclick="saveTune()">SAVE</button>
</div>

<div class="card">
  <h2>Kp: <span id="kpVal" class="val">0</span></h2>
  <input id="kp" class="slider" type="range" min="0" max="60" step="0.1" value="18">

  <h2>Ki: <span id="kiVal" class="val">0</span></h2>
  <input id="ki" class="slider" type="range" min="0" max="10" step="0.01" value="0.5">

  <h2>Kd: <span id="kdVal" class="val">0</span></h2>
  <input id="kd" class="slider" type="range" min="0" max="5" step="0.01" value="0.8">

  <h2>Setpoint (balance angle): <span id="spVal" class="val">0</span></h2>
  <input id="sp" class="slider" type="range" min="-15" max="15" step="0.1" value="0">

  <h2>Min PWM (deadband): <span id="mpVal" class="val">0</span></h2>
  <input id="mp" class="slider" type="range" min="0" max="150" step="1" value="60">

  <h2>Filter Alpha (gyro trust): <span id="alphaVal" class="val">0</span></h2>
  <input id="alpha" class="slider" type="range" min="0.80" max="0.999" step="0.001" value="0.96">
</div>

<div class="card">
  <h2>Left Trim: <span id="ltVal" class="val">0</span></h2>
  <input id="lt" class="slider" type="range" min="-50" max="50" value="0">

  <h2>Right Trim: <span id="rtVal" class="val">0</span></h2>
  <input id="rt" class="slider" type="range" min="-50" max="50" value="0">

  <label><input id="rl" type="checkbox"> Reverse Left</label>
  <label><input id="rr" type="checkbox"> Reverse Right</label>
  <label><input id="inv" type="checkbox"> Invert PID Output (try this if it leans worse and worse)</label>
</div>

<script>
const canvas = document.getElementById('graph');
const ctx = canvas.getContext('2d');
let angleData = [];
let outputData = [];
const MAXPTS = 100;

function drawGraph(){
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.strokeStyle = "#444";
  ctx.beginPath();
  ctx.moveTo(0, canvas.height/2);
  ctx.lineTo(canvas.width, canvas.height/2);
  ctx.stroke();

  ctx.strokeStyle = "#5dade2";
  ctx.beginPath();
  for(let i=0;i<angleData.length;i++){
    let x = i*(canvas.width/MAXPTS);
    let y = canvas.height/2 - (angleData[i]/45)*(canvas.height/2);
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();

  ctx.strokeStyle = "#e67e22";
  ctx.beginPath();
  for(let i=0;i<outputData.length;i++){
    let x = i*(canvas.width/MAXPTS);
    let y = canvas.height/2 - (outputData[i]/255)*(canvas.height/2);
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.stroke();
}

async function poll(){
  try{
    const res = await fetch('/status');
    const data = await res.json();
    document.getElementById('angleVal').innerText = data.angle.toFixed(2);
    document.getElementById('outVal').innerText = data.output.toFixed(0);

    angleData.push(data.angle);
    outputData.push(data.output);
    if(angleData.length > MAXPTS) angleData.shift();
    if(outputData.length > MAXPTS) outputData.shift();
    drawGraph();
  }catch(e){}
}
setInterval(poll, 150);

function setPid(state){
  fetch('/pid?state=' + (state ? 'on' : 'off'));
}

function saveTune(){
  fetch('/save').then(()=>alert('Saved to flash!'));
}

function bindSlider(id, labelId, url, isInt){
  const el = document.getElementById(id);
  const label = document.getElementById(labelId);
  label.innerText = el.value;
  el.oninput = () => {
    label.innerText = el.value;
    fetch(url + '?x=' + el.value);
  };
}

bindSlider('kp','kpVal','/setkp');
bindSlider('ki','kiVal','/setki');
bindSlider('kd','kdVal','/setkd');
bindSlider('sp','spVal','/setsp');
bindSlider('mp','mpVal','/setminpwm');
bindSlider('alpha','alphaVal','/setalpha');
bindSlider('lt','ltVal','/ltrim');
bindSlider('rt','rtVal','/rtrim');

document.getElementById('rl').onchange = (e)=> fetch('/revleft?x=' + e.target.checked);
document.getElementById('rr').onchange = (e)=> fetch('/revright?x=' + e.target.checked);
document.getElementById('inv').onchange = (e)=> fetch('/invert?x=' + e.target.checked);

async function loadCurrent(){
  const res = await fetch('/current');
  const d = await res.json();
  document.getElementById('kp').value = d.kp; document.getElementById('kpVal').innerText = d.kp;
  document.getElementById('ki').value = d.ki; document.getElementById('kiVal').innerText = d.ki;
  document.getElementById('kd').value = d.kd; document.getElementById('kdVal').innerText = d.kd;
  document.getElementById('sp').value = d.sp; document.getElementById('spVal').innerText = d.sp;
  document.getElementById('alpha').value = d.alpha; document.getElementById('alphaVal').innerText = d.alpha;
  document.getElementById('mp').value = d.mp; document.getElementById('mpVal').innerText = d.mp;
  document.getElementById('lt').value = d.lt; document.getElementById('ltVal').innerText = d.lt;
  document.getElementById('rt').value = d.rt; document.getElementById('rtVal').innerText = d.rt;
  document.getElementById('rl').checked = d.rl;
  document.getElementById('rr').checked = d.rr;
}
loadCurrent();
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", page);
}

// =====================================================
//                    WEB HANDLERS
// =====================================================

void handleStatus() {
  String json = "{\"angle\":" + String(filteredAngle, 2) +
                ",\"output\":" + String(pidOutput, 1) +
                ",\"running\":" + String(pidEnabled ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

void handleCurrent() {
  String json = "{";
  json += "\"kp\":" + String(kp, 2) + ",";
  json += "\"ki\":" + String(ki, 3) + ",";
  json += "\"kd\":" + String(kd, 3) + ",";
  json += "\"sp\":" + String(setpoint, 2) + ",";
  json += "\"alpha\":" + String(compAlpha, 3) + ",";
  json += "\"mp\":" + String(minPWM) + ",";
  json += "\"lt\":" + String(leftTrim) + ",";
  json += "\"rt\":" + String(rightTrim) + ",";
  json += "\"rl\":" + String(reverseLeft ? "true" : "false") + ",";
  json += "\"rr\":" + String(reverseRight ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void setupServer() {
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/current", handleCurrent);

  server.on("/pid", []() {
    String state = server.arg("state");
    if (state == "on") {
      errorSum = 0;
      lastError = 0;
      pidEnabled = true;
    } else {
      pidEnabled = false;
      stopMotors();
    }
    server.send(200, "text/plain", "ok");
  });

  server.on("/save", []() {
    saveSettings();
    server.send(200, "text/plain", "saved");
  });

  server.on("/setkp", []() { kp = server.arg("x").toFloat(); server.send(200); });
  server.on("/setki", []() { ki = server.arg("x").toFloat(); server.send(200); });
  server.on("/setkd", []() { kd = server.arg("x").toFloat(); server.send(200); });
  server.on("/setsp", []() { setpoint = server.arg("x").toFloat(); server.send(200); });
  server.on("/setalpha", []() { compAlpha = server.arg("x").toFloat(); server.send(200); });

  server.on("/ltrim", []() { leftTrim = server.arg("x").toInt(); server.send(200); });
  server.on("/rtrim", []() { rightTrim = server.arg("x").toInt(); server.send(200); });
  server.on("/setminpwm", []() { minPWM = server.arg("x").toInt(); server.send(200); });
  server.on("/invert", []() { invertOutput = (server.arg("x") == "true"); server.send(200); });
  server.on("/revleft", []() { reverseLeft = (server.arg("x") == "true"); server.send(200); });
  server.on("/revright", []() { reverseRight = (server.arg("x") == "true"); server.send(200); });

  server.begin();
}

// =====================================================
//                    SETUP / LOOP
// =====================================================

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  mpuInit();

  loadSettings();

  Serial.println("Calibrating gyro, keep robot still...");
  calibrateGyro();
  Serial.println("Calibration done.");

  mpuReadRaw();
  accAngle = (atan2((float)accY, (float)accZ) * 180.0 / PI) - angleOffset;
  filteredAngle = accAngle;
  gyroAngle = accAngle;

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP());

  setupServer();

  lastTime = micros();
}

void loop() {
  server.handleClient();

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  if (dt < 0.005) {
    yield();
    return;
  }
  lastTime = now;

  if (!mpuReadRaw()) return;

  accAngle = (atan2((float)accY, (float)accZ) * 180.0 / PI) - angleOffset;

  gyroRate = ((float)gyroX - gyroBiasX) / 131.0;

  gyroAngle = filteredAngle + gyroRate * dt;

  filteredAngle = compAlpha * gyroAngle + (1.0 - compAlpha) * accAngle;

  if (pidEnabled) {
    float error = setpoint - filteredAngle;

    errorSum += error * dt;
    errorSum = constrain(errorSum, -255, 255);

    float dError = (error - lastError) / dt;
    lastError = error;

    filteredDError = 0.8 * filteredDError + 0.2 * dError;

    pidOutput = kp * error + ki * errorSum + kd * filteredDError;
    pidOutput = constrain(pidOutput, -255, 255);

    if (fabs(filteredAngle) > 45) {
      pidOutput = 0;
      stopMotors();
      wasTipped = true;
    } else {
      if (wasTipped) {
        errorSum = 0;
        lastError = error;
        wasTipped = false;
      }
      driveFromPID(invertOutput ? -pidOutput : pidOutput);
    }
  } else {
    pidOutput = 0;
  }
}
