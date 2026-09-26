#pragma once

#include "motor_metrics.h"

// Builds the dashboard widgets on the active LVGL screen. Call once after lv_init()
// and display/buffer setup, on both the hardware target and the PC simulator.
void create_dashboard();

// Refreshes the dashboard widgets with a new data snapshot. Call periodically
// from both the hardware UI task and the simulator's main loop.
void update_dashboard(const MotorMetrics &metrics);

// Optional hook invoked after the user commits a new date/time via the on-screen
// clock editor, so a real RTC chip (if present) can be written to persist the
// change across power cycles. Register before create_dashboard() is called.
using RtcWriteCallback = void (*)(int year, unsigned month, unsigned day, int hour, int minute);
void set_rtc_write_callback(RtcWriteCallback callback);
