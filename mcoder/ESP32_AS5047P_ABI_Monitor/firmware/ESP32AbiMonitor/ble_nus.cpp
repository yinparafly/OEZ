/************************************************
 * Nordic UART BLE 实现（OEZ-ABI）
 ************************************************/
#include "ble_nus.h"

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

static BLEServer* g_server = nullptr;
static BLECharacteristic* g_tx = nullptr;
static BLECharacteristic* g_rx = nullptr;
static volatile bool g_connected = false;
static volatile bool g_ready = false;
static uint16_t g_telem_hz = BLE_TELEM_HZ_DEFAULT;
static bool g_conn_seen = false;
static bool g_connect_edge = false;

static char g_rx_line[128];
static size_t g_rx_len = 0;
static char g_rx_ready[128];
static volatile bool g_rx_has = false;

class ServerCbs : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    (void)s;
    g_connected = true;
    g_connect_edge = true;
    Serial.println("# BLE connected");
  }
  void onDisconnect(BLEServer* s) override {
    g_connected = false;
    g_conn_seen = false;
    Serial.println("# BLE disconnected");
    delay(80);
    s->startAdvertising();
  }
};

class RxCbs : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    if (!g_connected) g_connected = true;
    String v = ch->getValue();
    for (unsigned i = 0; i < v.length(); ++i) {
      char c = v[i];
      if (c == '\n' || c == '\r') {
        if (g_rx_len > 0 && !g_rx_has) {
          g_rx_line[g_rx_len] = '\0';
          memcpy(g_rx_ready, g_rx_line, g_rx_len + 1);
          g_rx_has = true;
          g_rx_len = 0;
        }
        continue;
      }
      if (g_rx_len + 1 < sizeof(g_rx_line)) {
        g_rx_line[g_rx_len++] = c;
      } else {
        g_rx_len = 0;
      }
    }
  }
};

bool bleConnected() {
  if (!g_ready || g_tx == nullptr) return false;
  if (g_connected) return true;
  if (g_server != nullptr && g_server->getConnectedCount() > 0) {
    g_connected = true;
    return true;
  }
  return false;
}

bool bleTakeConnectEdge() {
  // 边沿：回调置位，或 getConnectedCount 从 0→>0
  bool c = bleConnected();
  if (c && !g_conn_seen) {
    g_conn_seen = true;
    g_connect_edge = false;
    return true;
  }
  if (g_connect_edge) {
    g_connect_edge = false;
    g_conn_seen = true;
    return true;
  }
  if (!c) g_conn_seen = false;
  return false;
}

void bleSetRate(uint16_t hz) {
  if (hz < 5) hz = 5;
  if (hz > 50) hz = 50;
  g_telem_hz = hz;
}

uint16_t bleTelemHz() { return g_telem_hz; }

static volatile bool g_host_ble_mute = false;

void bleMuteHost(bool mute) { g_host_ble_mute = mute; }

bool bleHostMuted() { return g_host_ble_mute; }

void bleSendRaw(const char* data, size_t n) {
  if (!bleConnected() || g_tx == nullptr || data == nullptr || n == 0) return;
  if (n > 512) n = 512;

  uint16_t mtu = BLEDevice::getMTU();
  size_t chunk = 18;
  if (mtu > 23) {
    chunk = (size_t)mtu - 3;
    if (chunk > 180) chunk = 180;
    if (chunk < 18) chunk = 18;
  }

  if (n <= chunk) {
    g_tx->setValue((uint8_t*)data, n);
    g_tx->notify();
    delay(4);  // 给协议栈消化 Notify，避免背靠背冲垮连接
    return;
  }

  // 多包 Notify（手机拼包）；最后必须把完整行写回特征值，
  // 否则 Windows PC 的 READ 只能读到最后一截，转速会一直像 0/解析失败。
  size_t off = 0;
  while (off < n) {
    size_t m = n - off;
    if (m > chunk) m = chunk;
    g_tx->setValue((uint8_t*)(data + off), m);
    g_tx->notify();
    off += m;
    delay(off < n ? 6 : 4);
  }
  g_tx->setValue((uint8_t*)data, n);
}

void bleSendLine(const char* s) {
  if (!s) return;
  size_t n = strlen(s);
  if (n == 0) return;
  if (s[n - 1] == '\n') {
    bleSendRaw(s, n);
  } else {
    char tmp[256];
    if (n + 2 >= sizeof(tmp)) n = sizeof(tmp) - 2;
    memcpy(tmp, s, n);
    tmp[n] = '\n';
    tmp[n + 1] = '\0';
    bleSendRaw(tmp, n + 1);
  }
}

bool bleTakeRxLine(char* out, size_t out_sz) {
  if (!g_rx_has || out == nullptr || out_sz == 0) return false;
  noInterrupts();
  strncpy(out, g_rx_ready, out_sz - 1);
  out[out_sz - 1] = '\0';
  g_rx_has = false;
  interrupts();
  return true;
}

void bleBegin() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(247);
  g_server = BLEDevice::createServer();
  g_server->setCallbacks(new ServerCbs());
  BLEService* svc = g_server->createService(NUS_SERVICE_UUID);
  g_tx = svc->createCharacteristic(
      NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  g_tx->addDescriptor(new BLE2902());
  g_rx = svc->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE |
                                                     BLECharacteristic::PROPERTY_WRITE_NR);
  g_rx->setCallbacks(new RxCbs());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  g_ready = true;
  Serial.printf("# BLE name=%s NUS telem_default=%uHz\n", BLE_DEVICE_NAME, (unsigned)g_telem_hz);
}

void hostSerialPrintln(const char* s) {
  if (s) Serial.println(s);
}

void hostPrintln(const char* s) {
  if (!s) return;
  Serial.println(s);
  if (bleConnected() && !g_host_ble_mute) bleSendLine(s);
}

void hostPrintf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  if (bleConnected() && !g_host_ble_mute) {
    // 保证以换行结束便于客户端分行
    size_t n = strlen(buf);
    if (n > 0 && buf[n - 1] == '\n') bleSendLine(buf);
    else {
      char line[260];
      snprintf(line, sizeof(line), "%s\n", buf);
      bleSendLine(line);
    }
  }
}
