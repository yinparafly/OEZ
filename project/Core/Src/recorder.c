#include "recorder.h"
#include "ring_buffer.h"
#include "encoder.h"
#include "main.h"
#include "stm32f10x_it.h"
#include "usart_cmd.h"
#include "ff.h"

static RecorderState_t state = RECORDER_STARTUP_DELAY;
static bool rpm_ok = false;
static bool g_fs_mounted = false;
volatile uint32_t g_trigger_index = 0;
uint16_t g_file_index = 0;

static uint32_t g_startup_ticks = 0;
static uint32_t g_post_count = 0;
static uint32_t g_start_debounce = 0;
static uint32_t g_rpm_check_counter = 0;
static uint32_t g_last_rpm_count = 0;
static uint32_t g_led_tick = 0;

static FATFS g_fs;

static void Recorder_Trigger(void) {
    if (state == RECORDER_MONITOR) {
        state = RECORDER_TRIGGERED;
        g_trigger_index = Buffer_GetHead();
        g_post_count = 0;
    }
}

static void Recorder_StartMonitor(void) {
    state = RECORDER_MONITOR;
    rpm_ok = false;
    g_last_rpm_count = Encoder_GetCount();
    g_rpm_check_counter = 0;
    g_start_debounce = 0;
}

void Recorder_Init(void) {
    state = RECORDER_STARTUP_DELAY;
    rpm_ok = false;
    g_trigger_index = 0;
    g_z_signal_occurred = false;
    g_startup_ticks = 0;
    g_post_count = 0;
    g_start_debounce = 0;
    g_rpm_check_counter = 0;
    g_led_tick = 0;
    GREEN_LED_OFF();
    RED_LED_OFF();
    STATUS_LED_OFF();
    Recorder_ReadFileIndex();
}

void Recorder_ISR_Check(uint32_t count) {
    (void)count;
    g_led_tick++;
    switch (state) {
        case RECORDER_STARTUP_DELAY:
            g_startup_ticks++;
            if ((g_led_tick % 400) == 0) STATUS_LED_TOGGLE();
            if (g_startup_ticks >= 12000) {
                STATUS_LED_OFF();
                state = RECORDER_IDLE;
            }
            break;

        case RECORDER_IDLE:
            if ((g_led_tick % 4000) == 0) STATUS_LED_TOGGLE();
            GREEN_LED_OFF();
            RED_LED_OFF();
            if (GPIO_ReadInputDataBit(START_PORT, START_PIN) == Bit_RESET) {
                g_start_debounce++;
                if (g_start_debounce >= 200) {
                    g_start_debounce = 0;
                    Recorder_StartMonitor();
                }
            } else {
                g_start_debounce = 0;
            }
            if (Cmd_GetStartFlag()) {
                Cmd_ClearStartFlag();
                Recorder_StartMonitor();
            }
            break;

        case RECORDER_MONITOR:
            if ((g_led_tick % 4000) == 0) GREEN_LED_TOGGLE();
            RED_LED_OFF();
            if (GPIO_ReadInputDataBit(START_PORT, START_PIN) == Bit_RESET) {
                g_start_debounce++;
                if (g_start_debounce >= 200) {
                    g_start_debounce = 0;
                    Recorder_Trigger();
                }
            } else {
                g_start_debounce = 0;
            }
            if (rpm_ok && g_z_signal_occurred) {
                g_z_signal_occurred = false;
                Recorder_Trigger();
            }
            g_rpm_check_counter++;
            if (g_rpm_check_counter >= 200) {
                g_rpm_check_counter = 0;
                uint32_t current_count = Encoder_GetCount();
                int32_t delta = (int32_t)(current_count - g_last_rpm_count);
                g_last_rpm_count = current_count;
                float rpm = (float)delta / 4000.0f / (50.0f / 60000.0f);
                if (rpm > RPM_TRIGGER_HIGH) rpm_ok = true;
                if (rpm < RPM_TRIGGER_LOW) rpm_ok = false;
            }
            break;

        case RECORDER_TRIGGERED:
            GREEN_LED_OFF();
            if ((g_led_tick % 400) == 0) RED_LED_TOGGLE();
            g_post_count++;
            if (g_post_count >= POST_TRIGGER_SAMPLES) {
                state = RECORDER_WRITING;
            }
            break;

        case RECORDER_WRITING:
            GREEN_LED_OFF();
            RED_LED_OFF();
            break;

        case RECORDER_PATTERN_DONE:
            RED_LED_OFF();
            if (g_post_count < PATTERN_FAST_TICKS) {
                if ((g_post_count % 200) == 0) GREEN_LED_TOGGLE();
            } else if (g_post_count < PATTERN_TOTAL_TICKS) {
                if ((g_post_count % 2000) == 0) GREEN_LED_TOGGLE();
            } else {
                GREEN_LED_OFF();
                state = RECORDER_IDLE;
            }
            g_post_count++;
            break;
    }
}

RecorderState_t Recorder_GetState(void) { return state; }

void Record_SaveToSD(void) {
    if (g_buffer.count < TOTAL_RECORD_SAMPLES) {
        state = RECORDER_MONITOR;
        return;
    }

    if (!g_fs_mounted) {
        if (f_mount(0, &g_fs) != FR_OK) { state = RECORDER_MONITOR; return; }
        g_fs_mounted = true;
    }

    char fname[12];
    fname[0] = '0';
    fname[1] = ':';
    fname[2] = '0' + g_file_index / 1000;
    fname[3] = '0' + (g_file_index / 100) % 10;
    fname[4] = '0' + (g_file_index / 10) % 10;
    fname[5] = '0' + g_file_index % 10;
    fname[6] = '.';
    fname[7] = 'C';
    fname[8] = 'S';
    fname[9] = 'V';
    fname[10] = '\0';

    uint32_t start = (g_trigger_index - PRE_TRIGGER_SAMPLES) & RING_BUFFER_MASK;

    FIL fil;
    if (f_open(&fil, fname, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        state = RECORDER_MONITOR; return;
    }

    f_printf(&fil, "index,count\n");
    for (uint32_t i = 0; i < TOTAL_RECORD_SAMPLES; i++) {
        uint32_t pos = (start + i) & RING_BUFFER_MASK;
        f_printf(&fil, "%lu,%lu\n", i, g_buffer.data[pos]);
    }
    f_close(&fil);

    g_file_index++;
    Recorder_WriteFileIndex();

    state = RECORDER_PATTERN_DONE;
    g_post_count = 0;
}

void Recorder_ReadFileIndex(void) {
    if (!g_fs_mounted) {
        if (f_mount(0, &g_fs) != FR_OK) {
            g_file_index = 0;
            return;
        }
        g_fs_mounted = true;
    }

    FIL fil;
    if (f_open(&fil, "0:INDEX.TXT", FA_READ) != FR_OK) {
        g_file_index = 0;
        return;
    }

    char buf[8];
    unsigned int br;
    if (f_read(&fil, buf, sizeof(buf) - 1, &br) == FR_OK) {
        buf[br] = '\0';
        g_file_index = 0;
        for (char* p = buf; *p >= '0' && *p <= '9'; p++) {
            g_file_index = g_file_index * 10 + (*p - '0');
        }
    } else {
        g_file_index = 0;
    }
    f_close(&fil);
}

void Recorder_WriteFileIndex(void) {
    FIL fil;
    if (f_open(&fil, "0:INDEX.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK) {
        f_printf(&fil, "%u", (unsigned int)g_file_index);
        f_close(&fil);
    }
}