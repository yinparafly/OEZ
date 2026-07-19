/************************************************
 * BLE NUS 实现（对齐 Arduino-ESP32 UART 官方示例）
 ************************************************/
#include "ble_host.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static BLEServer* g_ble_server = nullptr;
static BLECharacteristic* g_ble_tx = nullptr;
static BLECharacteristic* g_ble_rx = nullptr;
static volatile bool g_ble_connected = false;
static volatile bool g_ble_ready = false;

static uint16_t g_ble_telem_hz = BLE_TELEM_HZ_DEFAULT;
static bool g_ble_lite = true;
static uint32_t g_ble_last_telem_ms = 0;

static char g_ble_rx_line[96];
static size_t g_ble_rx_len = 0;
static char g_ble_rx_ready[96];
static volatile bool g_ble_rx_has = false;

class BleServerCbs : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    g_ble_connected = true;
    Serial.println("# BLE client connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_ble_connected = false;
    Serial.println("# BLE client disconnected");
    delay(100);
    s->startAdvertising();
  }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) override {
    (void)s;
    (void)param;
    g_ble_connected = true;
    Serial.println("# BLE client connected (bluedroid)");
  }
  void onDisconnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) override {
    (void)param;
    g_ble_connected = false;
    Serial.println("# BLE client disconnected (bluedroid)");
    delay(100);
    s->startAdvertising();
  }
#endif
#if defined(CONFIG_NIMBLE_ENABLED)
  void onConnect(BLEServer* s, ble_gap_conn_desc* desc) override {
    (void)s;
    (void)desc;
    g_ble_connected = true;
    Serial.println("# BLE client connected (nimble)");
  }
  void onDisconnect(BLEServer* s, ble_gap_conn_desc* desc) override {
    (void)desc;
    g_ble_connected = false;
    Serial.println("# BLE client disconnected (nimble)");
    delay(100);
    s->startAdvertising();
  }
#endif
};

class BleRxCbs : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    // 部分栈 onConnect 不回调；收到写即视为已连接
    if (!g_ble_connected) {
      g_ble_connected = true;
      Serial.println("# BLE connected (via RX write)");
    }
    String v = ch->getValue();
    for (unsigned i = 0; i < v.length(); ++i) {
      char c = v[i];
      if (c == '\n' || c == '\r') {
        if (g_ble_rx_len > 0 && !g_ble_rx_has) {
          g_ble_rx_line[g_ble_rx_len] = '\0';
          memcpy(g_ble_rx_ready, g_ble_rx_line, g_ble_rx_len + 1);
          g_ble_rx_has = true;
          g_ble_rx_len = 0;
        }
        continue;
      }
      if (g_ble_rx_len + 1 < sizeof(g_ble_rx_line)) {
        g_ble_rx_line[g_ble_rx_len++] = c;
      } else {
        g_ble_rx_len = 0;
      }
    }
  }
};

bool bleConnected() {
  if (!g_ble_ready || g_ble_tx == nullptr) return false;
  if (g_ble_connected) return true;
  if (g_ble_server != nullptr && g_ble_server->getConnectedCount() > 0) {
    g_ble_connected = true;
    return true;
  }
  return false;
}

void bleSendRaw(const char* data, size_t n) {
  if (!bleConnected() || !data || n == 0) return;
  g_ble_tx->setValue((uint8_t*)data, n);
  g_ble_tx->notify();
}

void bleSendLine(const char* s) {
  if (!s) return;
  size_t n = strlen(s);
  if (n == 0) return;
  if (s[n - 1] == '\n') {
    bleSendRaw(s, n);
  } else {
    char tmp[256];
    if (n + 2 > sizeof(tmp)) n = sizeof(tmp) - 2;
    memcpy(tmp, s, n);
    tmp[n] = '\n';
    tmp[n + 1] = '\0';
    bleSendRaw(tmp, n + 1);
  }
}

void bleBegin() {
  Serial.println("# BLE init…");
  BLEDevice::init(BLE_DEVICE_NAME);

  g_ble_server = BLEDevice::createServer();
  if (!g_ble_server) {
    Serial.println("# BLE ERR createServer failed");
    return;
  }
  g_ble_server->setCallbacks(new BleServerCbs());

  // 多留 handle，避免特征/描述符装不下导致服务不可见
  BLEService* svc = g_ble_server->createService(BLEUUID(NUS_SERVICE_UUID), 40);
  if (!svc) {
    Serial.println("# BLE ERR createService failed");
    return;
  }

  // READ+NOTIFY：部分手机/Win 栈订阅更稳
  g_ble_tx = svc->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  if (!g_ble_tx) {
    Serial.println("# BLE ERR create TX failed");
    return;
  }
  // NimBLE 会为 NOTIFY 自动加 CCCD(2902)；再手动 add 会导致订阅失效、notify 发不出
#if !defined(CONFIG_NIMBLE_ENABLED)
  g_ble_tx->addDescriptor(new BLE2902());
  Serial.println("# BLE TX CCCD=manual2902");
#else
  Serial.println("# BLE TX CCCD=nimble-auto");
#endif

  g_ble_rx = svc->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  if (!g_ble_rx) {
    Serial.println("# BLE ERR create RX failed");
    return;
  }
  g_ble_rx->setCallbacks(new BleRxCbs());

  svc->start();
  Serial.printf("# BLE service started uuid=%s\n", NUS_SERVICE_UUID);

  BLEAdvertising* adv = g_ble_server->getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->start();

  g_ble_ready = true;
  Serial.printf("# BLE advertising name=%s\n", BLE_DEVICE_NAME);
}

bool bleTakeRxLine(char* out, size_t out_sz) {
  if (!g_ble_rx_has || !out || out_sz == 0) return false;
  strncpy(out, g_ble_rx_ready, out_sz - 1);
  out[out_sz - 1] = '\0';
  g_ble_rx_has = false;
  return true;
}

void bleEmitTelemLite(uint32_t t_ms, float rpm, uint16_t pulse, float target, int mode, int run) {
  if (!bleConnected()) return;
  uint32_t period = 1000UL / (uint32_t)g_ble_telem_hz;
  if (period < 20) period = 20;
  if ((uint32_t)(t_ms - g_ble_last_telem_ms) < period) return;
  g_ble_last_telem_ms = t_ms;

  char buf[96];
  if (g_ble_lite) {
    snprintf(buf, sizeof(buf), "B,%lu,%.1f,%u,%.1f,%d,%d\n", (unsigned long)t_ms, (double)rpm,
             (unsigned)pulse, (double)target, mode, run);
  } else {
    snprintf(buf, sizeof(buf), "%lu,0,0,0,%.2f,0,0,0,0,%u,%.1f,%d,0,0,0,0,%d\n",
             (unsigned long)t_ms, (double)rpm, (unsigned)pulse, (double)target, mode, run);
  }
  // setValue + notify：Win 上若未订阅，PC 仍可用 READ 轮询拿到最新行
  bleSendRaw(buf, strlen(buf));
}

void bleSetRate(uint16_t hz) {
  if (hz < BLE_TELEM_HZ_MIN) hz = BLE_TELEM_HZ_MIN;
  if (hz > BLE_TELEM_HZ_MAX) hz = BLE_TELEM_HZ_MAX;
  g_ble_telem_hz = hz;
}

void bleSetLite(bool on) { g_ble_lite = on; }
uint16_t bleTelemHz() { return g_ble_telem_hz; }
bool bleLiteOn() { return g_ble_lite; }

void hostPrintln(const char* s) {
  Serial.println(s);
  if (bleConnected()) bleSendLine(s);
}

void hostPrintf(const char* fmt, ...) {
  char buf[384];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof(buf)) n = (int)sizeof(buf) - 1;
  Serial.write((const uint8_t*)buf, (size_t)n);
  if (!bleConnected()) return;
  if (buf[0] >= '0' && buf[0] <= '9') return;  // USB 全速遥测不灌 BLE
  bleSendRaw(buf, (size_t)n);
  if (buf[n - 1] != '\n') bleSendRaw("\n", 1);
}
