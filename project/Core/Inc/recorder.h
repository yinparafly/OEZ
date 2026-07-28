#ifndef __RECORDER_H
#define __RECORDER_H

#include "stm32f10x.h"
#include <stdbool.h>

#define RPM_TRIGGER_HIGH    12
#define RPM_TRIGGER_LOW     8

#define PRE_TRIGGER_SAMPLES     200
#define POST_TRIGGER_MS         800
#define POST_TRIGGER_SAMPLES    3200
#define TOTAL_RECORD_SAMPLES    (PRE_TRIGGER_SAMPLES + POST_TRIGGER_SAMPLES)

typedef enum {
    RECORDER_IDLE,
    RECORDER_MONITOR,
    RECORDER_TRIGGERED,
    RECORDER_WRITING,
    RECORDER_DONE
} RecorderState_t;

void Recorder_Init(void);
void Recorder_ISR_Check(uint32_t count);
void Recorder_MainLoop(uint32_t now_ms);
bool Recorder_IsTriggered(void);
RecorderState_t Recorder_GetState(void);
void Recorder_StartMonitor(void);
void Record_SaveToSD(void);

extern volatile uint32_t g_trigger_index;

#endif
