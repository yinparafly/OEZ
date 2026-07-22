/************************************************
 * BLE NUS 实现（对齐 Arduino-ESP32 UART 官方示例）
 ************************************************/
#include "ble_host.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

static BLEServer* g_ble_server = nullptr;
static BLECharacteristic* g_ble_tx = nullptr;
static BLECharacteristic* g_ble_rx = nullptr;
static volatile bool g_ble_connected = false;
static volatile bool g_ble_ready = false;
static volatile bool g_ble_enabled = false;

static uint16_t g_ble_telem_hz = BLE_TELEM_HZ_DEFAULT;
static bool g_ble_lite = true;
static uint32_t g_ble_last_telem_ms = 0;
// 最近一帧瘦身遥测缓存：ACK/HALL meta 的 setValue 会冲掉 TX，手机 READ 易读到错行
static char g_ble_last_telem[96];
static size_t g_ble_last_telem_n = 0;

static char g_ble_rx_line[96];
static size_t g_ble_rx_len = 0;
static char g_ble_rx_ready[96];
static volatile bool g_ble_rx_has = false;

// ---- 异步日志环：实时路径（采集回调/控制任务）禁止直打 Serial ----
static const int ASYNC_LOG_DEPTH = 48;
static const int ASYNC_LOG_LEN = 384;  // 与 hostPrintf 栈缓冲对齐，防 LEARN/HALL 截断
static char g_async_buf[ASYNC_LOG_DEPTH][ASYNC_LOG_LEN];
static volatile uint8_t g_async_w = 0;
static volatile uint8_t g_async_r = 0;
static volatile uint32_t g_async_drop = 0;
static portMUX_TYPE g_async_mux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t g_forbid_print_task = nullptr;
static volatile bool g_in_realtime_cb = false;

void hostForbidPrintFromTask(TaskHandle_t t) { g_forbid_print_task = t; }
void hostEnterRealtimeCb() { g_in_realtime_cb = true; }
void hostExitRealtimeCb() { g_in_realtime_cb = false; }
uint32_t hostAsyncLogDropped() { return g_async_drop; }
void hostAsyncLogDropReset() {
  portENTER_CRITICAL(&g_async_mux);
  g_async_drop = 0;
  portEXIT_CRITICAL(&g_async_mux);
}

static bool hostMustDeferPrint() {
  if (g_in_realtime_cb) return true;
  if (g_forbid_print_task != nullptr &&
      xTaskGetCurrentTaskHandle() == g_forbid_print_task) {
    return true;
  }
  return false;
}

static void asyncLogPush(const char* data, size_t n) {
  if (!data || n == 0) return;
  if (n >= (size_t)ASYNC_LOG_LEN) n = (size_t)ASYNC_LOG_LEN - 1;
  portENTER_CRITICAL(&g_async_mux);
  uint8_t next = (uint8_t)((g_async_w + 1) % ASYNC_LOG_DEPTH);
  if (next == g_async_r) {
    // 追尾丢旧：推进读指针
    g_async_r = (uint8_t)((g_async_r + 1) % ASYNC_LOG_DEPTH);
    g_async_drop++;
  }
  memcpy(g_async_buf[g_async_w], data, n);
  g_async_buf[g_async_w][n] = '\0';
  g_async_w = next;
  portEXIT_CRITICAL(&g_async_mux);
}

void hostDrainAsyncLog() {
  for (int i = 0; i < ASYNC_LOG_DEPTH; ++i) {
    char line[ASYNC_LOG_LEN];
    bool have = false;
    portENTER_CRITICAL(&g_async_mux);
    if (g_async_r != g_async_w) {
      memcpy(line, g_async_buf[g_async_r], ASYNC_LOG_LEN);
      g_async_r = (uint8_t)((g_async_r + 1) % ASYNC_LOG_DEPTH);
      have = true;
    }
    portEXIT_CRITICAL(&g_async_mux);
    if (!have) break;
    size_t n = strlen(line);
    if (n == 0) continue;
    Serial.write((const uint8_t*)line, n);
    if (line[n - 1] != '\n') Serial.write('\n');
    if (bleConnected()) {
      if (line[n - 1] == '\n') bleSendLine(line);
      else {
        char tmp[ASYNC_LOG_LEN + 2];
        memcpy(tmp, line, n);
        tmp[n] = '\n';
        tmp[n + 1] = '\0';
        bleSendLine(tmp);
      }
    }
  }
}

class BleServerCbs : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    g_ble_connected = true;
    Serial.println("# BLE client connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_ble_connected = false;
    Serial.println("# BLE client disconnected");
    if (g_ble_enabled) {
      delay(100);
      s->startAdvertising();
    }
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
    if (g_ble_enabled) {
      delay(100);
      s->startAdvertising();
    }
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
    if (g_ble_enabled) {
      delay(100);
      s->startAdvertising();
    }
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

bool bleIsEnabled() { return g_ble_enabled && g_ble_ready; }

bool bleConnected() {
  if (!g_ble_enabled || !g_ble_ready || g_ble_tx == nullptr) return false;
  if (g_ble_connected) return true;
  if (g_ble_server != nullptr && g_ble_server->getConnectedCount() > 0) {
    g_ble_connected = true;
    return true;
  }
  return false;
}

void bleSendRaw(const char* data, size_t n) {
  if (!g_ble_enabled || !bleConnected() || !data || n == 0) return;
  g_ble_tx->setValue((uint8_t*)data, n);
  g_ble_tx->notify();
}

/** meta（ACK/HALL…）通知后，把 TX 特征值恢复为最新遥测，避免 READ 轮询读到 ACK */
static void bleRestoreTelemValue() {
  if (!bleConnected() || g_ble_tx == nullptr || g_ble_last_telem_n == 0) return;
  g_ble_tx->setValue((uint8_t*)g_ble_last_telem, g_ble_last_telem_n);
}

void bleSendMeta(const char* data, size_t n) {
  if (!g_ble_enabled || !bleConnected() || !data || n == 0) return;
  g_ble_tx->setValue((uint8_t*)data, n);
  g_ble_tx->notify();
  bleRestoreTelemValue();
}

void bleSendLine(const char* s) {
  if (!s) return;
  size_t n = strlen(s);
  if (n == 0) return;
  if (s[n - 1] == '\n') {
    bleSendMeta(s, n);
  } else {
    char tmp[256];
    if (n + 2 > sizeof(tmp)) n = sizeof(tmp) - 2;
    memcpy(tmp, s, n);
    tmp[n] = '\n';
    tmp[n + 1] = '\0';
    bleSendMeta(tmp, n + 1);
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
  g_ble_enabled = true;
  Serial.printf("# BLE advertising name=%s\n", BLE_DEVICE_NAME);
}

void bleSetEnabled(bool on) {
  if (on) {
    if (!g_ble_ready) {
      bleBegin();
      return;
    }
    g_ble_enabled = true;
    if (g_ble_server != nullptr) {
      BLEAdvertising* adv = g_ble_server->getAdvertising();
      if (adv) adv->start();
    }
    Serial.println("# BLE enabled (adv on)");
    return;
  }
  g_ble_enabled = false;
  g_ble_connected = false;
  g_ble_rx_has = false;
  g_ble_rx_len = 0;
  if (g_ble_server != nullptr) {
    // 尽量断开现有连接；栈差异下失败可忽略，TX/RX 已短路
    uint16_t n = g_ble_server->getConnectedCount();
    for (uint16_t i = 0; i < n; ++i) {
#if defined(CONFIG_NIMBLE_ENABLED)
      // NimBLE: disconnect by conn handle if available
#endif
      (void)i;
    }
    BLEAdvertising* adv = g_ble_server->getAdvertising();
    if (adv) adv->stop();
  }
  Serial.println("# BLE disabled (adv off, NUS shorted)");
}

void bleEnd() { bleSetEnabled(false); }

bool bleTakeRxLine(char* out, size_t out_sz) {
  if (!g_ble_enabled || !g_ble_rx_has || !out || out_sz == 0) return false;
  strncpy(out, g_ble_rx_ready, out_sz - 1);
  out[out_sz - 1] = '\0';
  g_ble_rx_has = false;
  return true;
}

void bleEmitTelemLite(uint32_t t_ms, float rpm, uint16_t pulse, float target, int mode, int run,
                      float out_hz_meas, float gear_meas) {
  if (!g_ble_enabled || !bleConnected()) return;
  uint32_t period = 1000UL / (uint32_t)g_ble_telem_hz;
  if (period < 20) period = 20;
  if ((uint32_t)(t_ms - g_ble_last_telem_ms) < period) return;
  g_ble_last_telem_ms = t_ms;

  // 手机/PC 显示用转速大小（与控制环一致），避免符号造成“差很大”的错觉
  float rpm_out = fabsf(rpm);
  float oh = isnan(out_hz_meas) ? -1.0f : out_hz_meas;
  float gm = isnan(gear_meas) ? -1.0f : gear_meas;

  char buf[112];
  if (g_ble_lite) {
    // B,t_ms,motor_rpm,pulse,target,mode,run,out_hz_meas,gear_meas
    snprintf(buf, sizeof(buf), "B,%lu,%.1f,%u,%.1f,%d,%d,%.3f,%.3f\n",
             (unsigned long)t_ms, (double)rpm_out, (unsigned)pulse, (double)target, mode, run,
             (double)oh, (double)gm);
  } else {
    snprintf(buf, sizeof(buf), "%lu,0,0,0,%.2f,0,0,0,0,%u,%.1f,%d,0,0,0,0,%d\n",
             (unsigned long)t_ms, (double)rpm_out, (unsigned)pulse, (double)target, mode, run);
  }
  size_t n = strlen(buf);
  if (n >= sizeof(g_ble_last_telem)) n = sizeof(g_ble_last_telem) - 1;
  memcpy(g_ble_last_telem, buf, n);
  g_ble_last_telem[n] = '\0';
  g_ble_last_telem_n = n;
  // setValue + notify：Win/手机若未订阅，仍可用 READ 拿到最新行
  bleSendRaw(buf, n);
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
  if (!s) return;
  if (hostMustDeferPrint()) {
    asyncLogPush(s, strlen(s));
    return;
  }
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
  if (hostMustDeferPrint()) {
    asyncLogPush(buf, (size_t)n);
    return;
  }
  Serial.write((const uint8_t*)buf, (size_t)n);
  if (!bleConnected()) return;
  if (buf[0] >= '0' && buf[0] <= '9') return;  // USB 遥测不灌 BLE
  if (buf[n - 1] != '\n' && n + 1 < (int)sizeof(buf)) {
    buf[n++] = '\n';
    buf[n] = '\0';
  }
  bleSendMeta(buf, (size_t)n);
}
