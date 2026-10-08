#include "dashboard_layout.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <lvgl.h>

LV_FONT_DECLARE(lv_font_montserrat_24);
LV_FONT_DECLARE(lv_font_montserrat_32);
LV_FONT_DECLARE(lv_font_montserrat_48);

namespace {
lv_obj_t *speed_label = nullptr;
lv_obj_t *power_label = nullptr;
lv_obj_t *cadence_label = nullptr;
lv_obj_t *battery_label = nullptr;
lv_obj_t *assist_label = nullptr;
lv_obj_t *temp_label = nullptr;
lv_obj_t *status_label = nullptr;
lv_obj_t *clock_label = nullptr;

lv_obj_t *time_settings_overlay = nullptr;
lv_obj_t *year_spinbox = nullptr;
lv_obj_t *month_spinbox = nullptr;
lv_obj_t *day_spinbox = nullptr;
lv_obj_t *hour_spinbox = nullptr;
lv_obj_t *minute_spinbox = nullptr;

// Software clock: kept as an epoch-seconds base captured at a known LVGL tick,
// advanced by the tick delta since then. Seeded from the system clock (which
// main.cpp sets from the PCF8563 RTC at boot, if present) in create_dashboard().
int64_t clock_epoch_base = 0;
uint32_t clock_tick_base = 0;

// Set via set_rtc_write_callback() so a real RTC chip can be persisted when the
// user edits the time on-screen; stays null (no-op) on builds without an RTC.
RtcWriteCallback rtc_write_callback = nullptr;

// Days since 1970-01-01 for a civil (year, month, day) date, and the inverse.
// http://howardhinnant.github.io/date_algorithms.html - avoids depending on
// mktime()/timezone behavior, which differs between the ESP32 and PC builds.
int64_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + static_cast<int64_t>(doe) - 719468;
}

void civil_from_days(int64_t z, int &y, unsigned &m, unsigned &d) {
    z += 719468;
    const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    const int64_t y0 = static_cast<int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    const unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp + (mp < 10 ? 3 : static_cast<unsigned>(-9));
    y = static_cast<int>(y0 + (m <= 2 ? 1 : 0));
}

int64_t current_epoch_seconds() {
    uint32_t elapsed_ms = lv_tick_get() - clock_tick_base;
    return clock_epoch_base + static_cast<int64_t>(elapsed_ms) / 1000;
}

void format_epoch(int64_t epoch, char *buf, size_t buf_len) {
    int64_t days = epoch >= 0 ? epoch / 86400 : (epoch - 86399) / 86400;
    int64_t secs_of_day = epoch - days * 86400;
    int y;
    unsigned mo, d;
    civil_from_days(days, y, mo, d);
    int h = static_cast<int>(secs_of_day / 3600);
    int mi = static_cast<int>((secs_of_day % 3600) / 60);
    int s = static_cast<int>(secs_of_day % 60);
    snprintf(buf, buf_len, "%04d-%02u-%02u  %02d:%02d:%02d", y, mo, d, h, mi, s);
}

void refresh_clock_label() {
    if (clock_label == nullptr) {
        return;
    }
    static char lastText[32] = "";
    char buf[32];
    format_epoch(current_epoch_seconds(), buf, sizeof(buf));
    if (strcmp(buf, lastText) != 0) {
        lv_label_set_text(clock_label, buf);
        strcpy(lastText, buf);
    }
}

void close_time_settings_overlay() {
    if (time_settings_overlay != nullptr) {
        lv_obj_delete(time_settings_overlay);
        time_settings_overlay = nullptr;
    }
}

void spin_increment_cb(lv_event_t *e) {
    lv_spinbox_increment(static_cast<lv_obj_t *>(lv_event_get_user_data(e)));
}

void spin_decrement_cb(lv_event_t *e) {
    lv_spinbox_decrement(static_cast<lv_obj_t *>(lv_event_get_user_data(e)));
}

void apply_time_settings_cb(lv_event_t *) {
    int y = lv_spinbox_get_value(year_spinbox);
    unsigned mo = static_cast<unsigned>(lv_spinbox_get_value(month_spinbox));
    unsigned d = static_cast<unsigned>(lv_spinbox_get_value(day_spinbox));
    int h = lv_spinbox_get_value(hour_spinbox);
    int mi = lv_spinbox_get_value(minute_spinbox);

    clock_epoch_base = days_from_civil(y, mo, d) * 86400 + h * 3600 + mi * 60;
    clock_tick_base = lv_tick_get();

    if (rtc_write_callback != nullptr) {
        rtc_write_callback(y, mo, d, h, mi);
    }

    refresh_clock_label();
    close_time_settings_overlay();
}

void cancel_time_settings_cb(lv_event_t *) {
    close_time_settings_overlay();
}

// Builds a titled stepper (-/value/+) inside `parent`, seeded with `value`. Uses a flex
// layout so fields always stack without overlap regardless of font/theme sizing, and the
// spinbox itself is non-interactive (read-only display) - only the +/- buttons edit it.
lv_obj_t *create_spin_field(lv_obj_t *parent, const char *title, int min, int max, int digits, int value) {
    lv_obj_t *field = lv_obj_create(parent);
    lv_obj_remove_style_all(field);
    lv_obj_set_size(field, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(field, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(field, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(field, 8, 0);
    lv_obj_set_scrollable(field, false);

    lv_obj_t *title_label = lv_label_create(field);
    lv_label_set_text(title_label, title);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0x65D6FF), 0);

    lv_obj_t *stepper = lv_obj_create(field);
    lv_obj_remove_style_all(stepper);
    lv_obj_set_size(stepper, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(stepper, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(stepper, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(stepper, 8, 0);
    lv_obj_set_scrollable(stepper, false);

    lv_obj_t *minus_btn = lv_button_create(stepper);
    lv_obj_set_size(minus_btn, 60, 60);
    lv_obj_t *minus_label = lv_label_create(minus_btn);
    lv_label_set_text(minus_label, "-");
    lv_obj_set_style_text_font(minus_label, &lv_font_montserrat_32, 0);
    lv_obj_center(minus_label);

    lv_obj_t *spinbox = lv_spinbox_create(stepper);
    lv_spinbox_set_range(spinbox, min, max);
    lv_spinbox_set_digit_format(spinbox, static_cast<uint32_t>(digits), 0);
    lv_spinbox_set_value(spinbox, value);
    lv_obj_set_style_text_font(spinbox, &lv_font_montserrat_48, 0);
    lv_obj_set_size(spinbox, digits > 2 ? 118 : 90, 64);
    lv_obj_set_clickable(spinbox, false);

    lv_obj_t *plus_btn = lv_button_create(stepper);
    lv_obj_set_size(plus_btn, 60, 60);
    lv_obj_t *plus_label = lv_label_create(plus_btn);
    lv_label_set_text(plus_label, "+");
    lv_obj_set_style_text_font(plus_label, &lv_font_montserrat_32, 0);
    lv_obj_center(plus_label);

    lv_obj_add_event_cb(minus_btn, spin_decrement_cb, LV_EVENT_CLICKED, spinbox);
    lv_obj_add_event_cb(plus_btn, spin_increment_cb, LV_EVENT_CLICKED, spinbox);

    return spinbox;
}

void open_time_settings_overlay(lv_event_t *) {
    if (time_settings_overlay != nullptr) {
        return;
    }

    int64_t epoch = current_epoch_seconds();
    int64_t days = epoch >= 0 ? epoch / 86400 : (epoch - 86399) / 86400;
    int64_t secs_of_day = epoch - days * 86400;
    int cur_year;
    unsigned cur_month, cur_day;
    civil_from_days(days, cur_year, cur_month, cur_day);
    int cur_hour = static_cast<int>(secs_of_day / 3600);
    int cur_minute = static_cast<int>((secs_of_day % 3600) / 60);

    time_settings_overlay = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(time_settings_overlay);
    lv_obj_set_size(time_settings_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(time_settings_overlay, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(time_settings_overlay, LV_OPA_60, 0);
    lv_obj_set_scrollable(time_settings_overlay, false);
    lv_obj_set_clickable(time_settings_overlay, true);

    lv_obj_t *panel = lv_obj_create(time_settings_overlay);
    lv_obj_set_size(panel, 770, 440);
    lv_obj_center(panel);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x1A2530), 0);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scrollable(panel, false);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "Set date & time");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0x65D6FF), 0);

    // Fixed width + wrap so the 5 fields never overflow the panel; they flow onto a
    // second row (e.g. Year/Month/Day, then Hour/Min) instead of getting clipped.
    lv_obj_t *fields_row = lv_obj_create(panel);
    lv_obj_remove_style_all(fields_row);
    lv_obj_set_size(fields_row, 720, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(fields_row, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(fields_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_column(fields_row, 40, 0);
    lv_obj_set_style_pad_row(fields_row, 30, 0);
    lv_obj_set_scrollable(fields_row, false);

    year_spinbox = create_spin_field(fields_row, "Year", 2000, 2099, 4, cur_year);
    month_spinbox = create_spin_field(fields_row, "Month", 1, 12, 2, static_cast<int>(cur_month));
    day_spinbox = create_spin_field(fields_row, "Day", 1, 31, 2, static_cast<int>(cur_day));
    hour_spinbox = create_spin_field(fields_row, "Hour", 0, 23, 2, cur_hour);
    minute_spinbox = create_spin_field(fields_row, "Min", 0, 59, 2, cur_minute);

    // Full panel width + space-between so Cancel lands at the bottom-left corner
    // and Set lands at the bottom-right corner.
    lv_obj_t *buttons_row = lv_obj_create(panel);
    lv_obj_remove_style_all(buttons_row);
    lv_obj_set_size(buttons_row, 720, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(buttons_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(buttons_row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_scrollable(buttons_row, false);

    lv_obj_t *cancel_btn = lv_button_create(buttons_row);
    lv_obj_set_size(cancel_btn, 150, 56);
    lv_obj_t *cancel_label = lv_label_create(cancel_btn);
    lv_label_set_text(cancel_label, "Cancel");
    lv_obj_set_style_text_font(cancel_label, &lv_font_montserrat_24, 0);
    lv_obj_center(cancel_label);
    lv_obj_add_event_cb(cancel_btn, cancel_time_settings_cb, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *set_btn = lv_button_create(buttons_row);
    lv_obj_set_size(set_btn, 150, 56);
    lv_obj_t *set_label = lv_label_create(set_btn);
    lv_label_set_text(set_label, "Set");
    lv_obj_set_style_text_font(set_label, &lv_font_montserrat_24, 0);
    lv_obj_center(set_label);
    lv_obj_add_event_cb(set_btn, apply_time_settings_cb, LV_EVENT_CLICKED, nullptr);
}

// Only touches a label's text when the formatted string actually changed - calling
// lv_label_set_text unconditionally every loop invalidates/redraws it constantly,
// which visibly jitters the RGB LCD (same root cause fixed in the 701display project).
void set_label_if_changed(lv_obj_t *label, char *lastText, size_t lastTextSize, const char *newText) {
    if (strcmp(newText, lastText) != 0) {
        lv_label_set_text(label, newText);
        strncpy(lastText, newText, lastTextSize - 1);
        lastText[lastTextSize - 1] = '\0';
    }
}
} // namespace

void set_rtc_write_callback(RtcWriteCallback callback) {
    rtc_write_callback = callback;
}

void create_dashboard() {
    // Seed the software clock from the system clock, which main.cpp already
    // synchronized from the RTC chip at boot (if one is present).
    time_t now = time(nullptr);
    if (now > 0) {
        clock_epoch_base = static_cast<int64_t>(now);
    }
    clock_tick_base = lv_tick_get();

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

    static lv_style_t style_emphasis;
    lv_style_init(&style_emphasis);
    lv_style_set_text_font(&style_emphasis, &lv_font_montserrat_32);
    lv_style_set_text_color(&style_emphasis, lv_color_hex(0x65D6FF));
    lv_style_set_text_outline_stroke_color(&style_emphasis, lv_color_hex(0x65D6FF));
    lv_style_set_text_outline_stroke_opa(&style_emphasis, LV_OPA_COVER);
    lv_style_set_text_outline_stroke_width(&style_emphasis, 1);

    speed_label = lv_label_create(screen);
    lv_obj_add_style(speed_label, &style_metric, 0);
    lv_obj_align(speed_label, LV_ALIGN_CENTER, 0, -150);
    lv_label_set_text(speed_label, "0.0 km/h");

    power_label = lv_label_create(screen);
    lv_obj_add_style(power_label, &style_header, 0);
    lv_obj_add_style(power_label, &style_emphasis, 0);
    lv_obj_align(power_label, LV_ALIGN_CENTER, 0, -45);
    lv_label_set_text(power_label, "Motor power: 0 W");

    cadence_label = lv_label_create(screen);
    lv_obj_add_style(cadence_label, &style_header, 0);
    lv_obj_align(cadence_label, LV_ALIGN_CENTER, 0, -85);
        lv_label_set_text(cadence_label, "Pedal cadence: 0 RPM");

    assist_label = lv_label_create(screen);
    lv_obj_add_style(assist_label, &style_header, 0);
    lv_obj_add_style(assist_label, &style_emphasis, 0);
    lv_obj_align(assist_label, LV_ALIGN_CENTER, 0, -5);
    lv_label_set_text(assist_label, "Assist 0");

    battery_label = lv_label_create(screen);
    lv_obj_add_style(battery_label, &style_header, 0);
    lv_obj_add_style(battery_label, &style_emphasis, 0);
    lv_obj_align(battery_label, LV_ALIGN_CENTER, 0, 75);
    lv_label_set_text(battery_label, "Battery: 0% / 0.0V");

    temp_label = lv_label_create(screen);
    lv_obj_add_style(temp_label, &style_header, 0);
    lv_obj_align(temp_label, LV_ALIGN_CENTER, 0, 35);
        lv_label_set_text(temp_label, "Motor temp: 0 C");

    status_label = lv_label_create(screen);
    lv_obj_add_style(status_label, &style_header, 0);
    lv_obj_set_width(status_label, 776);
    lv_label_set_long_mode(status_label, LV_LABEL_LONG_MODE_DOTS);
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_align(status_label, LV_ALIGN_BOTTOM_RIGHT, -12, -12);
    lv_label_set_text(status_label, "UART");

    // Tap the clock to open the date/time setter (no RTC on this board).
    clock_label = lv_label_create(screen);
    lv_obj_add_style(clock_label, &style_header, 0);
    lv_obj_align(clock_label, LV_ALIGN_TOP_LEFT, 12, 12);
    lv_obj_set_clickable(clock_label, true);
    lv_obj_add_event_cb(clock_label, open_time_settings_overlay, LV_EVENT_CLICKED, nullptr);
    refresh_clock_label();
}

// Only touches a label's text when the formatted string actually changed - calling
// lv_label_set_text unconditionally every loop invalidates/redraws it constantly,
// which visibly jitters the RGB LCD (same root cause fixed in the 701display project).
void update_dashboard(const MotorMetrics &metrics) {
    static char lastSpeed[32] = "";
    static char lastPower[32] = "";
    static char lastCadence[32] = "";
    static char lastAssist[32] = "";
    static char lastBattery[32] = "";
    static char lastTemp[32] = "";
    static char lastStatus[32] = "";
    char buf[32];

    snprintf(buf, sizeof(buf), "%.1f mph", metrics.speedKmh * 0.621371f);
    set_label_if_changed(speed_label, lastSpeed, sizeof(lastSpeed), buf);
    snprintf(buf, sizeof(buf), "Motor power: %u W", metrics.motorPowerWatts);
    set_label_if_changed(power_label, lastPower, sizeof(lastPower), buf);
    snprintf(buf, sizeof(buf), "Pedal cadence: %u RPM", metrics.cadenceRpm);
    set_label_if_changed(cadence_label, lastCadence, sizeof(lastCadence), buf);
    snprintf(buf, sizeof(buf), "Assist %u", metrics.assistLevel);
    set_label_if_changed(assist_label, lastAssist, sizeof(lastAssist), buf);
    if (metrics.batteryVoltage > 0.0f) {
        snprintf(buf, sizeof(buf), "Battery: %u%% / %.1fV", metrics.batterySocPercent, metrics.batteryVoltage);
    } else {
        snprintf(buf, sizeof(buf), "Battery: %u%%", metrics.batterySocPercent);
    }
    set_label_if_changed(battery_label, lastBattery, sizeof(lastBattery), buf);
    snprintf(buf, sizeof(buf), "Motor temp: %d C", metrics.motorTempC);
    set_label_if_changed(temp_label, lastTemp, sizeof(lastTemp), buf);
    set_label_if_changed(status_label, lastStatus, sizeof(lastStatus),
                         metrics.canActive ? "UART: Receiving" : "UART: No data");
    refresh_clock_label();
}
