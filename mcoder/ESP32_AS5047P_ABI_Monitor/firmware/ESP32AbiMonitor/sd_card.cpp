#include "sd_card.h"

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <string.h>

#include "esp_heap_caps.h"

// 与文档约定一致；避开 ABI 15/16/17、USB 19/20
static const int PIN_SD_CS = 10;
static const int PIN_SD_SCK = 12;
static const int PIN_SD_MOSI = 11;
static const int PIN_SD_MISO = 13;

static bool g_sd_ok = false;
static SPIClass g_sd_spi(FSPI);

bool sdBegin() {
  if (g_sd_ok) {
    SD.end();
    g_sd_ok = false;
    delay(20);
  }
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  g_sd_spi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  // 先慢速挂载，兼容杂牌模块
  if (!SD.begin(PIN_SD_CS, g_sd_spi, 4000000)) {
    Serial.println("# SD FAIL: begin (check wiring CS=10 SCK=12 MOSI=11 MISO=13 3V3)");
    return false;
  }
  uint8_t t = SD.cardType();
  if (t == CARD_NONE) {
    Serial.println("# SD FAIL: no card");
    SD.end();
    return false;
  }
  g_sd_ok = true;
  const char* tn = "UNKNOWN";
  if (t == CARD_MMC) tn = "MMC";
  else if (t == CARD_SD) tn = "SDSC";
  else if (t == CARD_SDHC) tn = "SDHC";
  uint64_t sz = SD.cardSize() / (1024ULL * 1024ULL);
  Serial.printf("# SD OK type=%s size_MB=%llu CS=%d SCK=%d MOSI=%d MISO=%d\n", tn,
                (unsigned long long)sz, PIN_SD_CS, PIN_SD_SCK, PIN_SD_MOSI, PIN_SD_MISO);
  return true;
}

bool sdReady() { return g_sd_ok; }

void sdPrintStatus() {
  if (!g_sd_ok) {
    Serial.println("# SD ready=0 (send SD INIT or check wiring)");
    return;
  }
  uint64_t total = SD.totalBytes();
  uint64_t used = SD.usedBytes();
  Serial.printf("# SD ready=1 type=%u size_MB=%llu used_MB=%llu free_MB=%llu\n",
                (unsigned)SD.cardType(), (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)),
                (unsigned long long)(used / (1024ULL * 1024ULL)),
                (unsigned long long)((total > used ? total - used : 0) / (1024ULL * 1024ULL)));
}

bool sdSelfTest() {
  if (!g_sd_ok && !sdBegin()) return false;
  Serial.printf("# SD heap free_internal=%u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  const char* path = "/oez_sd_test.txt";
  const char* msg = "OEZ SD TEST OK\n";

  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.println("# SD TEST FAIL: open write");
    return false;
  }
  size_t n = f.print(msg);
  f.flush();
  f.close();
  if (n < strlen(msg)) {
    Serial.println("# SD TEST FAIL: short write");
    return false;
  }

  f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println("# SD TEST FAIL: open read");
    return false;
  }
  char buf[64];
  memset(buf, 0, sizeof(buf));
  size_t r = f.read((uint8_t*)buf, sizeof(buf) - 1);
  f.close();
  if (r >= sizeof(buf)) r = sizeof(buf) - 1;
  buf[r] = '\0';
  if (strncmp(buf, "OEZ SD TEST OK", 14) != 0) {
    Serial.printf("# SD TEST FAIL: content mismatch got='%s'\n", buf);
    return false;
  }
  Serial.printf("# SD TEST OK wrote+read %s (%u bytes)\n", path, (unsigned)n);
  return true;
}

bool sdWriteBinary(const char* path, const uint8_t* data, size_t len) {
  if (!g_sd_ok && !sdBegin()) return false;
  if (!path || !data) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("# SD WRITE FAIL open %s\n", path);
    return false;
  }
  size_t w = f.write(data, len);
  f.flush();
  f.close();
  if (w != len) {
    Serial.printf("# SD WRITE FAIL %s wrote=%u expect=%u\n", path, (unsigned)w, (unsigned)len);
    return false;
  }
  Serial.printf("# SD WRITE OK %s bytes=%u\n", path, (unsigned)w);
  return true;
}

bool sdWriteText(const char* path, const char* text) {
  if (!g_sd_ok && !sdBegin()) return false;
  if (!path || !text) return false;
  File f = SD.open(path, FILE_WRITE);
  if (!f) {
    Serial.printf("# SD WRITE FAIL open %s\n", path);
    return false;
  }
  size_t n = strlen(text);
  size_t w = f.write((const uint8_t*)text, n);
  f.flush();
  f.close();
  if (w != n) {
    Serial.printf("# SD WRITE FAIL %s wrote=%u expect=%u\n", path, (unsigned)w, (unsigned)n);
    return false;
  }
  Serial.printf("# SD META OK %s bytes=%u\n", path, (unsigned)w);
  return true;
}

void sdListRoot(uint8_t max_files) {
  if (!g_sd_ok && !sdBegin()) return;
  Serial.printf("# SD LIST heap free_internal=%u\n",
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  // 大卡目录项多时 openNextFile 易吃堆；只列少量并尽早结束
  if (max_files > 12) max_files = 12;
  File root = SD.open("/");
  if (!root) {
    Serial.println("# SD LIST FAIL open /");
    return;
  }
  if (!root.isDirectory()) {
    root.close();
    Serial.println("# SD LIST FAIL not dir");
    return;
  }
  Serial.println("# SD LIST /");
  uint8_t n = 0;
  while (n < max_files) {
    File e = root.openNextFile();
    if (!e) break;
    char name[48];
    const char* nm = e.name();
    if (!nm) nm = "?";
    strncpy(name, nm, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    uint32_t sz = (uint32_t)e.size();
    bool is_dir = e.isDirectory();
    e.close();
    Serial.printf("#   %s%s  %lu\n", name, is_dir ? "/" : "", (unsigned long)sz);
    n++;
    delay(2);
    yield();
  }
  root.close();
  Serial.printf("# SD LIST end n=%u\n", (unsigned)n);
}
