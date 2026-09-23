#pragma once

#include "motor_metrics.h"

// Builds the dashboard widgets on the active LVGL screen. Call once after lv_init()
// and display/buffer setup, on both the hardware target and the PC simulator.
void create_dashboard();

// Refreshes the dashboard widgets with a new data snapshot. Call periodically
// from both the hardware UI task and the simulator's main loop.
void update_dashboard(const MotorMetrics &metrics);
