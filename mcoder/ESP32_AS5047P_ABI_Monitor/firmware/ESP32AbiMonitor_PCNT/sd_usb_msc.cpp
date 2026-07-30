#include "sd_usb_msc.h"

#include <Arduino.h>
#include <SD.h>
#include <string.h>

#include "sd_card.h"

#if SOC_USB_OTG_SUPPORTED && !ARDUINO_USB_MODE
#include <USB.h>
#include <USBMSC.h>
#define SD_USB_MSC_BUILD 1
#else
#define SD_USB_MSC_BUILD 0
#endif

#if SD_USB_MSC_BUILD
static USBMSC g_msc;
static bool g_msc_on = false;
static bool g_usb_begun = false;

static int32_t mscOnWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  (void)lba;
  (void)offset;
  (void)buffer;
  // 只读盘：拒绝写，避免 PC 与固件抢卡
  return -1;
}

static int32_t mscOnRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  (void)offset;
  uint32_t secSize = SD.sectorSize();
  if (!secSize || !buffer || !bufsize) return -1;
  if (bufsize % secSize) return -1;
  uint8_t* out = (uint8_t*)buffer;
  uint32_t n = bufsize / secSize;
  for (uint32_t x = 0; x < n; x++) {
    if (!SD.readRAW(out + (x * secSize), lba + x)) return -1;
  }
  return (int32_t)bufsize;
}

static bool mscOnStartStop(uint8_t power_condition, bool start, bool load_eject) {
  (void)power_condition;
  (void)start;
  (void)load_eject;
  return true;
}
#endif

bool sdMscAvailable() {
#if SD_USB_MSC_BUILD
  return true;
#else
  return false;
#endif
}

bool sdMscIsOn() {
#if SD_USB_MSC_BUILD
  return g_msc_on;
#else
  return false;
#endif
}

bool sdMscOn() {
#if !SD_USB_MSC_BUILD
  Serial.println("# USB DISK FAIL: need USBMode=USB-OTG (TinyUSB), not Hardware CDC");
  return false;
#else
  if (g_msc_on) {
    Serial.println("# USB DISK already ON");
    return true;
  }
  if (!sdReady() && !sdBegin()) {
    Serial.println("# USB DISK FAIL: no SD");
    return false;
  }
  size_t sectors = SD.numSectors();
  size_t secSize = SD.sectorSize();
  if (!sectors || !secSize) {
    Serial.println("# USB DISK FAIL: sector info");
    return false;
  }

  g_msc.vendorID("OEZCON");
  g_msc.productID("ABI_SD");
  g_msc.productRevision("1.0");
  g_msc.onRead(mscOnRead);
  g_msc.onWrite(mscOnWrite);
  g_msc.onStartStop(mscOnStartStop);
  g_msc.mediaPresent(true);
  g_msc.isWritable(false);
  if (!g_msc.begin(sectors, secSize)) {
    Serial.println("# USB DISK FAIL: MSC begin");
    return false;
  }
  if (!g_usb_begun) {
    USB.begin();
    g_usb_begun = true;
  } else {
    // 已 begin 过：刷新介质，促使主机重新枚举
    g_msc.mediaPresent(false);
    delay(50);
    g_msc.mediaPresent(true);
  }
  g_msc_on = true;
  Serial.printf("# USB DISK ON sectors=%u sec=%u size_MB=%llu READ-ONLY product=ABI_SD\n",
                (unsigned)sectors, (unsigned)secSize,
                (unsigned long long)(SD.cardSize() / (1024ULL * 1024ULL)));
  Serial.println("# → Native USB 应变为 U盘(ABI_SD)，不是 JTAG");
  Serial.println("# → 若仍是 JTAG：拔掉再插一次 Native USB（CH343 串口可保持）");
  Serial.println("# → 拷完 snap_*.bin 后发 USB DISK OFF，再武装下一发");
  return true;
#endif
}

bool sdMscOff() {
#if !SD_USB_MSC_BUILD
  Serial.println("# USB DISK OFF (not built)");
  return false;
#else
  if (!g_msc_on) {
    Serial.println("# USB DISK already OFF");
    return true;
  }
  g_msc.end();
  g_msc_on = false;
  // 重新挂载 FatFS，保证后续 SD SAVE 可用
  if (!sdBegin()) {
    Serial.println("# USB DISK OFF but SD remount FAIL — send SD INIT");
    return false;
  }
  Serial.println("# USB DISK OFF — SD remounted, REC/SD SAVE OK");
  return true;
#endif
}

void sdMscPrintStatus() {
#if !SD_USB_MSC_BUILD
  Serial.println("# USB DISK available=0 (rebuild with USBMode=default TinyUSB)");
#else
  Serial.printf("# USB DISK available=1 on=%u sd=%u\n", (unsigned)g_msc_on, (unsigned)sdReady());
#endif
}
