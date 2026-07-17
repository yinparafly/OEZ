/*
 * ESP32 Power Control for CUAV Nora
 * 功能：继电器控制 Nora 供电，支持 WiFi/蓝牙/HTTP 远程重启
 * 
 * 硬件接线：
 *   ESP32 GPIO12 → 继电器 IN
 *   继电器 COM → Nora VCC (电池+)
 *   继电器 NO  → Nora VCC (通过继电器)
 *   GND 共地
 *
 * 使用方法：
 *   1. 烧录固件，WiFi 名称密码在下方修改
 *   2. ESP32 上电后自动连接 WiFi，串口打印 IP 地址
 *   3. 浏览器访问 http://<IP>/reboot 重启 Nora
 *   4. 浏览器访问 http://<IP>/status 查看状态
 *   5. 浏览器访问 http://<IP>/off 断电
 *   6. 浏览器访问 http://<IP>/on 上电
 */

#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>

// ============================================================
// 配置区 - 修改这里
// ============================================================
const char* WIFI_SSID = "YOUR_WIFI_SSID";      // WiFi 名称
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";   // WiFi 密码
const int RELAY_PIN = 12;                        // 继电器引脚
const int LED_PIN = 2;                           // 板载 LED
const int REBOOT_DELAY_MS = 2000;                // 断电等待时间 (ms)
const uint16_t HTTP_PORT = 80;                   // HTTP 端口

// ============================================================
// 全局变量
// ============================================================
WebServer server(HTTP_PORT);
bool relayState = false;
unsigned long lastRebootTime = 0;
int rebootCount = 0;

// ============================================================
// 继电器控制
// ============================================================
void relayOn() {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
    relayState = true;
}

void relayOff() {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
    relayState = false;
}

// 重启：断电 → 等待 → 上电
void rebootNora() {
    relayOff();
    delay(REBOOT_DELAY_MS);
    relayOn();
    lastRebootTime = millis();
    rebootCount++;
}

// ============================================================
// HTTP 处理器
// ============================================================
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset='UTF-8'>";
    html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
    html += "<title>Nora Power Control</title>";
    html += "<style>";
    html += "body{font-family:Arial,sans-serif;text-align:center;margin:40px;background:#1a1a2e;color:#eee}";
    html += "h1{color:#e94560}";
    html += ".btn{display:inline-block;padding:15px 30px;margin:10px;font-size:18px;";
    html += "border:none;border-radius:8px;cursor:pointer;text-decoration:none;color:#fff}";
    html += ".btn-on{background:#0f3460}";
    html += ".btn-off{background:#e94560}";
    html += ".btn-reboot{background:#533483}";
    html += ".status{background:#16213e;padding:20px;border-radius:10px;margin:20px auto;max-width:400px}";
    html += "</style></head><body>";
    html += "<h1>🔌 Nora Power Control</h1>";
    html += "<div class='status'>";
    html += "<p>State: <b>" + String(relayState ? "ON" : "OFF") + "</b></p>";
    html += "<p>Uptime: " + String(millis() / 1000) + "s</p>";
    html += "<p>Reboots: " + String(rebootCount) + "</p>";
    html += "</div>";
    html += "<a class='btn btn-on' href='/on'>ON</a>";
    html += "<a class='btn btn-off' href='/off'>OFF</a>";
    html += "<a class='btn btn-reboot' href='/reboot'>REBOOT</a>";
    html += "</body></html>";
    server.send(200, "text/html", html);
}

void handleOn() {
    relayOn();
    server.send(200, "text/plain", "Nora: ON");
}

void handleOff() {
    relayOff();
    server.send(200, "text/plain", "Nora: OFF");
}

void handleReboot() {
    rebootNora();
    server.send(200, "text/plain", "Nora: Rebooted (off " + String(REBOOT_DELAY_MS) + "ms)");
}

void handleStatus() {
    String json = "{";
    json += "\"relay\":" + String(relayState ? "true" : "false") + ",";
    json += "\"uptime\":" + String(millis() / 1000) + ",";
    json += "\"reboots\":" + String(rebootCount);
    json += "}";
    server.send(200, "application/json", json);
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Nora Power Control ===");
    
    // GPIO
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    relayOff();
    
    // WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nWiFi failed! Starting AP mode...");
        WiFi.softAP("NoraPower", "12345678");
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
    
    // HTTP 路由
    server.on("/", handleRoot);
    server.on("/on", handleOn);
    server.on("/off", handleOff);
    server.on("/reboot", handleReboot);
    server.on("/status", handleStatus);
    
    server.begin();
    Serial.println("HTTP server started on port " + String(HTTP_PORT));
    Serial.println("Commands: /on /off /reboot /status");
}

// ============================================================
// Loop
// ============================================================
void loop() {
    server.handleClient();
    
    // WiFi 断线重连
    if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
        delay(5000);
    }
}
