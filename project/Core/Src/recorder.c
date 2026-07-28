#include "recorder.h"
#include "ring_buffer.h"
#include "encoder.h"
#include "stm32f10x_it.h"
#include "ff.h"
#include "string.h"

static RecorderState_t state = RECORDER_IDLE;
static bool rpm_ok = false;
static bool record_ready = false;
volatile uint32_t g_trigger_index = 0;
static uint32_t trigger_start_ms = 0;
static uint32_t last_rpm_check_count = 0;
static uint32_t last_rpm_check_time = 0;

static float CalcRPM(uint32_t now_ms) {
    uint32_t dt = now_ms - last_rpm_check_time;
    if (dt < 50) return 0.0f;
    uint32_t current_count = Encoder_GetCount();
    int32_t delta = (int32_t)(current_count - last_rpm_check_count);
    float rpm = (float)delta / 4000.0f / ((float)dt / 60000.0f);
    last_rpm_check_count = current_count;
    last_rpm_check_time = now_ms;
    return rpm;
}

void Recorder_Init(void) {
    state = RECORDER_IDLE;
    rpm_ok = false;
    record_ready = false;
    g_trigger_index = 0;
    g_z_signal_occurred = false;
}

void Recorder_StartMonitor(void) {
    state = RECORDER_MONITOR;
    rpm_ok = false;
    trigger_start_ms = 0;
    last_rpm_check_count = Encoder_GetCount();
    last_rpm_check_time = 0;
}

void Recorder_ISR_Check(uint32_t count) {
    (void)count;
    if (state == RECORDER_MONITOR && rpm_ok && g_z_signal_occurred) {
        state = RECORDER_TRIGGERED;
        g_trigger_index = Buffer_GetHead();
        g_z_signal_occurred = false;
    }
}

void Recorder_MainLoop(uint32_t now_ms) {
    switch (state) {
        case RECORDER_IDLE:
            break;
        case RECORDER_MONITOR: {
            float rpm = CalcRPM(now_ms);
            if (rpm > RPM_TRIGGER_HIGH) rpm_ok = true;
            if (rpm < RPM_TRIGGER_LOW)  rpm_ok = false;
            break;
        }
        case RECORDER_TRIGGERED:
            if (trigger_start_ms == 0) trigger_start_ms = now_ms;
            if ((now_ms - trigger_start_ms) >= POST_TRIGGER_MS) {
                state = RECORDER_WRITING;
                record_ready = true;
            }
            break;
        case RECORDER_WRITING:
            break;
        case RECORDER_DONE:
            record_ready = false;
            state = RECORDER_MONITOR;
            break;
    }
}

bool Recorder_IsTriggered(void) { return record_ready; }
RecorderState_t Recorder_GetState(void) { return state; }

static FATFS g_fs;
static bool g_fs_mounted = false;

void Record_SaveToSD(void) {
    if (g_buffer.count < TOTAL_RECORD_SAMPLES) {
        state = RECORDER_MONITOR;
        return;
    }

    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);

    uint32_t start = (g_trigger_index - PRE_TRIGGER_SAMPLES) & RING_BUFFER_MASK;

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    if (!g_fs_mounted) {
        if (f_mount(0, &g_fs) != FR_OK) { state = RECORDER_MONITOR; return; }
        g_fs_mounted = true;
    }

    FIL fil;
    if (f_open(&fil, "0:TEST.CSV", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        state = RECORDER_MONITOR; return;
    }

    f_printf(&fil, "index,count\n");
    TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
    for (uint32_t i = 0; i < TOTAL_RECORD_SAMPLES; i++) {
        uint32_t pos = (start + i) & RING_BUFFER_MASK;
        f_printf(&fil, "%lu,%lu\n", i, g_buffer.data[pos]);
    }
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    f_close(&fil);

    record_ready = false;
    state = RECORDER_DONE;
}
