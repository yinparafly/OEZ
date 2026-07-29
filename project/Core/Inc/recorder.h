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

#define PATTERN_FAST_TICKS      1600
#define PATTERN_SLOW_TICKS      8000
#define PATTERN_TOTAL_TICKS     9600

typedef enum {
    RECORDER_STARTUP_DELAY,
    RECORDER_IDLE,
    RECORDER_MONITOR,
    RECORDER_TRIGGERED,
    RECORDER_WRITING,
    RECORDER_PATTERN_DONE
} RecorderState_t;

void Recorder_Init(void);
void Recorder_ISR_Check(uint32_t count);
RecorderState_t Recorder_GetState(void);
void Record_SaveToSD(void);
void Recorder_ReadFileIndex(void);
void Recorder_WriteFileIndex(void);

extern volatile uint32_t g_trigger_index;
extern uint16_t g_file_index;

#endif