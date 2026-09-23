#include "dashboard_layout.h"

#include <cstdio>
#include <lvgl.h>

LV_FONT_DECLARE(lv_font_montserrat_24);
LV_FONT_DECLARE(lv_font_montserrat_48);

namespace {
lv_obj_t *speed_label = nullptr;
lv_obj_t *power_label = nullptr;
lv_obj_t *cadence_label = nullptr;
lv_obj_t *battery_label = nullptr;
lv_obj_t *assist_label = nullptr;
lv_obj_t *temp_label = nullptr;
lv_obj_t *status_label = nullptr;
} // namespace

void create_dashboard() {
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(0xF4F7FA), 0);

    static lv_style_t style_header;
    lv_style_init(&style_header);
    lv_style_set_text_font(&style_header, &lv_font_montserrat_24);
    lv_style_set_text_color(&style_header, lv_color_hex(0x65D6FF));

    static lv_style_t style_metric;
    lv_style_init(&style_metric);
    lv_style_set_text_font(&style_metric, &lv_font_montserrat_48);

    speed_label = lv_label_create(screen);
    lv_obj_add_style(speed_label, &style_metric, 0);
    lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -140);
    lv_label_set_text(speed_label, "0.0 km/h");

    power_label = lv_label_create(screen);
    lv_obj_add_style(power_label, &style_header, 0);
    lv_obj_align(power_label, LV_ALIGN_CENTER, -260, -40);
    lv_label_set_text(power_label, "0 W");

    cadence_label = lv_label_create(screen);
    lv_obj_add_style(cadence_label, &style_header, 0);
    lv_obj_align(cadence_label, LV_ALIGN_CENTER, 0, -40);
    lv_label_set_text(cadence_label, "0 RPM");

    assist_label = lv_label_create(screen);
    lv_obj_add_style(assist_label, &style_header, 0);
    lv_obj_align(assist_label, LV_ALIGN_CENTER, 260, -40);
    lv_label_set_text(assist_label, "Assist 0");

    battery_label = lv_label_create(screen);
    lv_obj_add_style(battery_label, &style_header, 0);
    lv_obj_align(battery_label, LV_ALIGN_CENTER, -200, 60);
    lv_label_set_text(battery_label, "0% / 0.0V");

    temp_label = lv_label_create(screen);
    lv_obj_add_style(temp_label, &style_header, 0);
    lv_obj_align(temp_label, LV_ALIGN_CENTER, 200, 60);
    lv_label_set_text(temp_label, "0 C");

    status_label = lv_label_create(screen);
    lv_obj_add_style(status_label, &style_header, 0);
    lv_obj_align(status_label, LV_ALIGN_CENTER, 0, 140);
    lv_label_set_text(status_label, "CAN: waiting...");
}

void update_dashboard(const MotorMetrics &metrics) {
    char buf[32];

    snprintf(buf, sizeof(buf), "%.1f km/h", metrics.speedKmh);
    lv_label_set_text(speed_label, buf);
    snprintf(buf, sizeof(buf), "%u W", metrics.powerWatts);
    lv_label_set_text(power_label, buf);
    snprintf(buf, sizeof(buf), "%u RPM", metrics.cadenceRpm);
    lv_label_set_text(cadence_label, buf);
    snprintf(buf, sizeof(buf), "Assist %u", metrics.assistLevel);
    lv_label_set_text(assist_label, buf);
    snprintf(buf, sizeof(buf), "%u%% / %.1fV", metrics.batterySocPercent, metrics.batteryVoltage);
    lv_label_set_text(battery_label, buf);
    snprintf(buf, sizeof(buf), "%d C", metrics.motorTempC);
    lv_label_set_text(temp_label, buf);
    lv_label_set_text(status_label, metrics.canActive ? "CAN: active" : "CAN: no data");
}
