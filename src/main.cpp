#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include "esp_camera.h"

// --- Camera Pins (OV2640 / OV3660 for ESP32-S3 CAM) ---
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5
#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

// Dedicated Clean I2C Pins on Port 1
#define PIN_SDA 1
#define PIN_SCL 2
#define MPU_ADDR 0x68

const int MOTOR_PINS[] = {41, 42, 39, 40}; 
const int PWM_FREQ = 250;
const int PWM_RES  = 12;

WebServer server(80);

volatile float roll_angle = 0.0f;
volatile float pitch_angle = 0.0f;
volatile int16_t rc_throttle = 1000;
volatile int16_t rc_roll = 0;
volatile int16_t rc_pitch = 0;
volatile int16_t rc_yaw = 0;
volatile bool is_armed = false;
volatile bool emergency_stop = false;
volatile bool mpu_connected = false;
bool camera_ok = false;
String cam_err_msg = "Starting...";
volatile unsigned long last_cmd_ms = 0;

float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;
float Kp_roll = 1.30f, Ki_roll = 0.04f, Kd_roll = 0.30f;
float Kp_pitch = 1.30f, Ki_pitch = 0.04f, Kd_pitch = 0.30f;
float Kp_yaw = 2.40f,   Ki_yaw = 0.02f,  Kd_yaw = 0.00f;
float err_i_roll = 0, err_i_pitch = 0, err_i_yaw = 0;

void set_motor_us(uint8_t index, int us) {
    us = constrain(us, 1000, 2000);
    uint32_t duty = (us * 4095) / 4000;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(MOTOR_PINS[index], duty);
#else
    ledcWrite(index, duty);
#endif
}

bool readMPU_Direct(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(0x3B);
    if (Wire1.endTransmission(false) != 0) return false;

    if (Wire1.requestFrom((uint16_t)MPU_ADDR, (uint8_t)14, (bool)true) != 14) return false;

    ax = (Wire1.read() << 8) | Wire1.read();
    ay = (Wire1.read() << 8) | Wire1.read();
    az = (Wire1.read() << 8) | Wire1.read();
    Wire1.read(); Wire1.read(); // Skip temp
    gx = (Wire1.read() << 8) | Wire1.read();
    gy = (Wire1.read() << 8) | Wire1.read();
    gz = (Wire1.read() << 8) | Wire1.read();
    return true;
}

bool initMPU_Safe() {
    Wire1.begin(PIN_SDA, PIN_SCL, 100000);
    delay(40);

    Wire1.beginTransmission(MPU_ADDR);
    if (Wire1.endTransmission() != 0) {
        Wire1.beginTransmission(0x69);
        if (Wire1.endTransmission() != 0) return false;
    }

    Wire1.beginTransmission(MPU_ADDR);
    Wire1.write(0x6B);
    Wire1.write(0x00);
    Wire1.endTransmission();
    return true;
}

void calibrate_imu() {
    long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int samples = 0;

    for (int i = 0; i < 200; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (readMPU_Direct(ax, ay, az, gx, gy, gz)) {
            ax_sum += ax; ay_sum += ay; az_sum += az;
            gx_sum += gx; gy_sum += gy; gz_sum += gz;
            samples++;
        }
        delay(2);
    }

    if (samples > 50) {
        ax_bias = ax_sum / (float)samples;
        ay_bias = ay_sum / (float)samples;
        az_bias = (az_sum / (float)samples) - 16384.0f;
        gx_bias = gx_sum / (float)samples;
        gy_bias = gy_sum / (float)samples;
        gz_bias = gz_sum / (float)samples;
    }
}

void flightControlTask(void *pvParameters) {
    mpu_connected = initMPU_Safe();
    if (mpu_connected) calibrate_imu();

    for (int i = 0; i < 4; i++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
        ledcAttach(MOTOR_PINS[i], PWM_FREQ, PWM_RES);
#else
        ledcSetup(i, PWM_FREQ, PWM_RES);
        ledcAttachPin(MOTOR_PINS[i], i);
#endif
        set_motor_us(i, 1000);
    }

    unsigned long last_us = micros();
    const unsigned long PERIOD_US = 4000;

    while (true) {
        unsigned long now_us = micros();
        if (now_us - last_us < PERIOD_US) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        float dt = (now_us - last_us) / 1000000.0f;
        last_us = now_us;

        if (!mpu_connected) {
            mpu_connected = initMPU_Safe();
            if (mpu_connected) calibrate_imu();
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        if (!readMPU_Direct(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz)) continue;

        float gx = (raw_gx - gx_bias) / 131.0f;
        float gy = (raw_gy - gy_bias) / 131.0f;
        float gz = (raw_gz - gz_bias) / 131.0f;

        float acc_r = atan2(raw_ay - ay_bias, raw_az - az_bias) * 57.2958f;
        float acc_p = atan2(-(raw_ax - ax_bias), sqrt(pow(raw_ay - ay_bias, 2) + pow(raw_az - az_bias, 2))) * 57.2958f;

        roll_angle  = 0.95f * (roll_angle + gx * dt) + 0.05f * acc_r;
        pitch_angle = 0.95f * (pitch_angle + gy * dt) + 0.05f * acc_p;

        bool failsafe = false;
        if (emergency_stop) failsafe = true;
        if (abs(roll_angle) > 60.0f || abs(pitch_angle) > 60.0f) failsafe = true;
        if (millis() - last_cmd_ms > 600) {
            rc_throttle = max(1000, rc_throttle - 8);
            if (rc_throttle <= 1050) failsafe = true;
        }

        float r_err = rc_roll - roll_angle;
        err_i_roll = constrain(err_i_roll + r_err * dt, -150.0f, 150.0f);
        float r_out = (Kp_roll * r_err) + (Ki_roll * err_i_roll) + (Kd_roll * -gx);

        float p_err = rc_pitch - pitch_angle;
        err_i_pitch = constrain(err_i_pitch + p_err * dt, -150.0f, 150.0f);
        float p_out = (Kp_pitch * p_err) + (Ki_pitch * err_i_pitch) + (Kd_pitch * -gy);

        float y_err = rc_yaw - gz;
        err_i_yaw = constrain(err_i_yaw + y_err * dt, -100.0f, 100.0f);
        float y_out = (Kp_yaw * y_err) + (Ki_yaw * err_i_yaw);

        if (is_armed && !failsafe && rc_throttle > 1050) {
            int m1 = rc_throttle - r_out + p_out - y_out; // FR (Pin 41)
            int m2 = rc_throttle - r_out - p_out + y_out; // RR (Pin 42)
            int m3 = rc_throttle + r_out - p_out - y_out; // RL (Pin 39)
            int m4 = rc_throttle + r_out + p_out + y_out; // FL (Pin 40)

            set_motor_us(0, m1);
            set_motor_us(1, m2);
            set_motor_us(2, m3);
            set_motor_us(3, m4);
        } else {
            err_i_roll = 0; err_i_pitch = 0; err_i_yaw = 0;
            for (int i = 0; i < 4; i++) set_motor_us(i, 1000);
        }
    }
}

// Embedded Web Cockpit with Hardware-Accelerated Video Rotation
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Aero Drone Cockpit</title>
<style>
  body { margin:0; background:#0e1117; color:#fff; font-family:sans-serif; text-align:center; overflow:hidden; }
  .video-container { width:92%; max-width:440px; height:240px; margin:6px auto 0 auto; display:flex; align-items:center; justify-content:center; overflow:hidden; border-radius:8px; border:2px solid #333; background:#1b222c; }
  #stream { max-width:100%; max-height:100%; transition: transform 0.2s ease; }
  .rot-btn { padding:5px 12px; font-size:12px; background:#222; color:#00e5ff; border:1px solid #444; border-radius:4px; margin-top:4px; cursor:pointer; }
  .hud { font-size:13px; margin:4px 0; color:#00e5ff; font-weight:bold; }
  .btn-row { margin: 4px 0; }
  button { padding:12px 24px; font-size:14px; font-weight:bold; border-radius:6px; border:none; margin:0 5px; cursor:pointer; }
  .btn-arm { background:#00c853; color:#fff; }
  .btn-kill { background:#d50000; color:#fff; }
  .controls { display:flex; justify-content:space-around; align-items:center; margin-top:6px; }
  .pad { width:130px; height:130px; background:#1b222c; border-radius:50%; border:2px solid #444; position:relative; touch-action:none; }
  .knob { width:44px; height:44px; background:#00e5ff; border-radius:50%; position:absolute; top:43px; left:43px; }
</style>
</head>
<body>
  <div class="video-container">
    <img id="stream" src="/stream">
  </div>
  <button class="rot-btn" onclick="rotateVideo()">🔄 Rotate Video 90°</button>
  <div class="hud" id="hud">Connecting to Drone...</div>
  <div class="btn-row">
    <button class="btn-arm" id="arm-btn" onclick="toggleArm()">ARM</button>
    <button class="btn-kill" onclick="killSwitch()">EMERGENCY STOP</button>
  </div>
  <div class="controls">
    <div class="pad" id="pad-left"><div class="knob" id="knob-left"></div></div>
    <div class="pad" id="pad-right"><div class="knob" id="knob-right"></div></div>
  </div>

<script>
let state = { throttle: 1000, yaw: 0, pitch: 0, roll: 0, armed: 0, kill: 0 };
let currentRotation = 0;

function rotateVideo() {
  currentRotation = (currentRotation + 90) % 360;
  document.getElementById('stream').style.transform = `rotate(${currentRotation}deg)`;
  localStorage.setItem('cam_rot', currentRotation);
}

// Load saved rotation on start
window.addEventListener('load', () => {
  let saved = localStorage.getItem('cam_rot');
  if (saved) {
    currentRotation = parseInt(saved);
    document.getElementById('stream').style.transform = `rotate(${currentRotation}deg)`;
  }
});

function toggleArm() {
  state.armed = state.armed ? 0 : 1;
  state.kill = 0;
  document.getElementById('arm-btn').style.background = state.armed ? '#ff9100' : '#00c853';
  document.getElementById('arm-btn').innerText = state.armed ? 'DISARM' : 'ARM';
}

function killSwitch() {
  state.kill = 1;
  state.armed = 0;
  state.throttle = 1000;
  document.getElementById('arm-btn').style.background = '#00c853';
  document.getElementById('arm-btn').innerText = 'ARM';
}

function setupJoystick(padId, knobId, onMove, onEnd) {
  const pad = document.getElementById(padId);
  const knob = document.getElementById(knobId);
  const maxDist = 43;

  function handleTouch(e) {
    e.preventDefault();
    const rect = pad.getBoundingClientRect();
    const touch = e.touches ? e.touches[0] : e;
    let dx = touch.clientX - (rect.left + rect.width / 2);
    let dy = touch.clientY - (rect.top + rect.height / 2);
    const dist = Math.min(Math.hypot(dx, dy), maxDist);
    const angle = Math.atan2(dy, dx);
    const kx = Math.cos(angle) * dist;
    const ky = Math.sin(angle) * dist;
    knob.style.transform = `translate(${kx}px, ${ky}px)`;
    onMove(kx / maxDist, -ky / maxDist);
  }

  pad.addEventListener('touchstart', handleTouch);
  pad.addEventListener('touchmove', handleTouch);
  pad.addEventListener('touchend', onEnd);
}

setupJoystick('pad-left', 'knob-left', (x, y) => {
  state.yaw = Math.round(x * 150);
  state.throttle = Math.round(1000 + ((y + 1) / 2) * 1000);
}, () => { state.yaw = 0; });

setupJoystick('pad-right', 'knob-right', (x, y) => {
  state.roll = Math.round(x * 30);
  state.pitch = Math.round(y * 30);
}, () => {
  state.roll = 0;
  state.pitch = 0;
  document.getElementById('knob-right').style.transform = `translate(0px, 0px)`;
});

setInterval(() => {
  fetch(`/control?t=${state.throttle}&r=${state.roll}&p=${state.pitch}&y=${state.yaw}&a=${state.armed}&k=${state.kill}`);
}, 50);

setInterval(async () => {
  try {
    const res = await fetch('/telemetry');
    const d = await res.json();
    document.getElementById('hud').innerText = 
      `Roll: ${d.roll.toFixed(1)}° | Pitch: ${d.pitch.toFixed(1)}° | Cam: ${d.cam}`;
  } catch(e) {}
}, 150);
</script>
</body>
</html>
)rawliteral";

void handle_stream() {
    WiFiClient client = server.client();
    client.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n");

    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) break;

        client.print("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\n\r\n");
        client.write(fb->buf, fb->len);
        client.print("\r\n");
        esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void setup() {
    Serial.begin(115200);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 10000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;

    if (psramFound()) {
        config.fb_count = 2;
        config.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t cam_err = esp_camera_init(&config);
    if (cam_err == ESP_OK) {
        camera_ok = true;
        cam_err_msg = "ONLINE (OK)";
    } else {
        camera_ok = false;
        cam_err_msg = "Init Failed (0x" + String(cam_err, HEX) + ")";
    }

    WiFi.softAP("AERO_DRONE_WIFI", "12345678");

    server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });
    server.on("/stream", HTTP_GET, handle_stream);
    server.on("/control", HTTP_GET, []() {
        if (server.hasArg("t")) rc_throttle = server.arg("t").toInt();
        if (server.hasArg("r")) rc_roll = server.arg("r").toInt();
        if (server.hasArg("p")) rc_pitch = server.arg("p").toInt();
        if (server.hasArg("y")) rc_yaw = server.arg("y").toInt();
        if (server.hasArg("a")) is_armed = server.arg("a").toInt();
        if (server.hasArg("k")) emergency_stop = server.arg("k").toInt();
        last_cmd_ms = millis();
        server.send(200, "text/plain", "OK");
    });
    server.on("/telemetry", HTTP_GET, []() {
        String json = "{\"roll\":" + String(roll_angle, 1) +
                      ",\"pitch\":" + String(pitch_angle, 1) +
                      ",\"cam\":\"" + cam_err_msg + "\"}";
        server.send(200, "application/json", json);
    });
    server.begin();

    xTaskCreatePinnedToCore(
        flightControlTask,
        "FlightCtrl",
        8192,
        NULL,
        10,
        NULL,
        1
    );
}

void loop() {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
}