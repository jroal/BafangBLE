#include <Arduino.h>
#include <Wire.h>
#include <driver/twai.h>    // ESP32 native CAN driver (Bafang M620 uses CAN)
#include <SD_MMC.h>
#include <ESP_Panel_Library.h>
#include <esp_heap_caps.h>
#include <sys/time.h>
#include <time.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#if __has_include(<lvgl.h>)
    #include <lvgl.h>
#elif __has_include(<lvgl/lvgl.h>)
    #include <lvgl/lvgl.h>
#else
    #error "LVGL is not installed or its include directory is not configured"
#endif

#include "dashboard_layout.h"
#include "motor_metrics.h"
#include "pins.h"

// =========================================================================
// CONFIGURATION - verify against your wiring / captured bus traffic
// =========================================================================
static const gpio_num_t CAN_TX_PIN = static_cast<gpio_num_t>(PIN_CAN_TX);
static const gpio_num_t CAN_RX_PIN = static_cast<gpio_num_t>(PIN_CAN_RX);
// Match the confirmed low 16 bits; the complete 29-bit identifier was not provided.
static const uint32_t CAN_TELEMETRY_ID_SUFFIX = 0x3200;
#ifndef BAFANG_CAN_ROUTE_ENABLED
#define BAFANG_CAN_ROUTE_ENABLED 1
#endif
static constexpr bool CAN_ROUTE_ENABLED = BAFANG_CAN_ROUTE_ENABLED != 0;

static const unsigned long BLE_NOTIFY_INTERVAL_MS = 500;
static const unsigned long SD_LOG_FLUSH_INTERVAL_MS = 1000;

// Waveshare's own 04_RTC_Test example confirms this board's RTC chip is a PCF85063A
// (register layout differs from the more common PCF8563 despite sharing address 0x51).
static constexpr uint8_t PCF85063A_I2C_ADDRESS = 0x51;
static constexpr uint8_t PCF85063A_CTRL1_REGISTER = 0x00;
static constexpr uint8_t PCF85063A_SECONDS_REGISTER = 0x04;
static constexpr int PCF85063A_YEAR_OFFSET = 1970;
static const uint8_t USB_CAN_SELECT_PIN = 5;
static const uint8_t CH422G_IO_WRITE_ADDRESS = 0x38;
static constexpr const char *CAN_LOG_PATH = "/can_capture.csv";
static constexpr UBaseType_t CAN_LOG_QUEUE_LENGTH = 1024;
static constexpr unsigned long SD_LOG_RETRY_INTERVAL_MS = 5000;

#define BLE_SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TELEMETRY_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

// =========================================================================
// SHARED STATE
// =========================================================================
SemaphoreHandle_t dataMutex;
volatile MotorMetrics liveData;

// Packed layout broadcast over BLE (little-endian), mirrors MotorMetrics.
#pragma pack(push, 1)
struct BleTelemetryPacket {
    float speedKmh;
    uint16_t cadenceRpm;
    uint16_t powerWatts;
    uint8_t assistLevel;
    int8_t motorTempC;
    float batteryVoltage;
    float batteryCurrent;
    uint8_t batterySocPercent;
    uint8_t errorCode;
};
#pragma pack(pop)

TaskHandle_t CANTaskHandle = NULL;
TaskHandle_t UITaskHandle = NULL;
TaskHandle_t SDLoggerTaskHandle = NULL;

struct CanLogRecord {
    uint32_t timestampMs;
    twai_message_t message;
};

QueueHandle_t canLogQueue = NULL;
File canLogFile;
volatile bool sdLoggingEnabled = false;
volatile uint32_t canLogDroppedFrames = 0;
bool canDriverStarted = false;

// =========================================================================
// DISPLAY / LVGL
// =========================================================================
ESP_Panel hardwarePanel;
ESP_PanelLcd *hardwareLcd = nullptr;
ESP_PanelLcdTouch *hardwareTouch = nullptr;
lv_indev_t *hardwareTouchIndev = nullptr;
lv_display_t *hardwareDisplay = nullptr;
static uint8_t *lvglFrameBuffer = nullptr;
static uint8_t *lvglFrameBuffer2 = nullptr;

// Touch is polled from its own task on core 0, away from the RGB LCD flush on core 1 -
// the ESP32-S3's RGB panel DMA is sensitive to bus contention from I2C activity on the same core.
SemaphoreHandle_t touchMutex;
volatile bool touchPressedShared = false;
volatile uint16_t touchXShared = 0;
volatile uint16_t touchYShared = 0;
TaskHandle_t TouchTaskHandle = NULL;

static bool expander_write_io(uint8_t outputLevels) {
    uint8_t outputEnable = 0x01;
    Wire.beginTransmission(0x24);
    Wire.write(outputEnable);
    if (Wire.endTransmission() != 0) {
        return false;
    }
    Wire.beginTransmission(CH422G_IO_WRITE_ADDRESS);
    Wire.write(outputLevels);
    return Wire.endTransmission() == 0;
}

static uint8_t expander_usb_can_levels() {
    uint8_t outputLevels = 0xFFU;
    if (!CAN_ROUTE_ENABLED) {
        outputLevels = static_cast<uint8_t>(outputLevels & ~(1U << USB_CAN_SELECT_PIN));
    }
    return outputLevels;
}

static void hardware_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *pixelMap) {
    if (hardwareLcd != nullptr) {
        hardwareLcd->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, pixelMap);
    }
    lv_display_flush_ready(display);
}

static void hardware_touch_read_cb(lv_indev_t *, lv_indev_data_t *data) {
    xSemaphoreTake(touchMutex, portMAX_DELAY);
    bool pressed = touchPressedShared;
    uint16_t x = touchXShared;
    uint16_t y = touchYShared;
    xSemaphoreGive(touchMutex);

    if (pressed) {
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void TouchProcessingLoop(void *) {
    while (true) {
        if (hardwareTouch != nullptr) {
            hardwareTouch->readData();
            bool pressed = hardwareTouch->getTouchState();
            TouchPoint point = pressed ? hardwareTouch->getPoint() : TouchPoint();

            xSemaphoreTake(touchMutex, portMAX_DELAY);
            touchPressedShared = pressed;
            touchXShared = point.x;
            touchYShared = point.y;
            xSemaphoreGive(touchMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Configures CH422G outputs after Wire has brought up the shared I2C bus.
static bool expander_init() {
    uint8_t outputLevels = expander_usb_can_levels();
    if (!expander_write_io(outputLevels)) {
        Serial.println("EXPANDER: output level write failed");
        return false;
    }
    return true;
}

static bool decode_bcd(uint8_t rawValue, int &decodedValue) {
    uint8_t ones = rawValue & 0x0F;
    uint8_t tens = (rawValue >> 4) & 0x0F;
    if (ones > 9 || tens > 9) {
        return false;
    }
    decodedValue = tens * 10 + ones;
    return true;
}

static uint8_t encode_bcd(int value) {
    return static_cast<uint8_t>(((value / 10) << 4) | (value % 10));
}

static int64_t days_from_civil(int year, unsigned month, unsigned day) {
    year -= month <= 2;
    const int64_t era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
    const unsigned adjustedMonth = month > 2 ? month - 3 : month + 9;
    const unsigned dayOfYear = (153 * adjustedMonth + 2) / 5 + day - 1;
    const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
    return era * 146097 + static_cast<int64_t>(dayOfEra) - 719468;
}

static bool rtc_sync_system_time() {
    Wire.beginTransmission(PCF85063A_I2C_ADDRESS);
    Wire.write(PCF85063A_SECONDS_REGISTER);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(PCF85063A_I2C_ADDRESS, static_cast<uint8_t>(7)) != 7) {
        Serial.println("RTC: PCF85063A read failed");
        return false;
    }

    // Register order: seconds, minutes, hours, days, weekdays, months, years.
    uint8_t registers[7];
    for (uint8_t &value : registers) {
        value = Wire.read();
    }

    int second, minute, hour, day, month, year;
    if ((registers[0] & 0x80) != 0 ||
        !decode_bcd(registers[0] & 0x7F, second) ||
        !decode_bcd(registers[1] & 0x7F, minute) ||
        !decode_bcd(registers[2] & 0x3F, hour) ||
        !decode_bcd(registers[3] & 0x3F, day) ||
        !decode_bcd(registers[5] & 0x1F, month) ||
        !decode_bcd(registers[6], year)) {
        Serial.println("RTC: invalid or unset PCF85063A time");
        return false;
    }

    year += PCF85063A_YEAR_OFFSET;
    if (second > 59 || minute > 59 || hour > 23 || month < 1 || month > 12 || day < 1) {
        Serial.println("RTC: out-of-range PCF85063A time");
        return false;
    }

    static const uint8_t daysPerMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t monthLength = daysPerMonth[month - 1];
    if (month == 2 && (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0))) {
        ++monthLength;
    }
    if (day > monthLength) {
        Serial.println("RTC: invalid PCF8563 calendar date");
        return false;
    }

    int64_t epoch = days_from_civil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;
    timeval systemTime = {static_cast<time_t>(epoch), 0};
    if (settimeofday(&systemTime, nullptr) != 0) {
        Serial.println("RTC: settimeofday failed");
        return false;
    }
    Serial.printf("RTC: system time synchronized from PCF85063A (%04d-%02d-%02d %02d:%02d:%02d UTC)\n",
                  year, month, day, hour, minute, second);
    return true;
}

// Registered with dashboard_layout via set_rtc_write_callback() so the on-screen clock
// editor persists the chosen time to the battery-backed PCF85063A, surviving power cycles.
static void rtc_write_time(int year, unsigned month, unsigned day, int hour, int minute) {
    // CTRL_1 default (oscillator running, no stop/reset) - some boards boot with STOP set.
    Wire.beginTransmission(PCF85063A_I2C_ADDRESS);
    Wire.write(PCF85063A_CTRL1_REGISTER);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();

    Wire.beginTransmission(PCF85063A_I2C_ADDRESS);
    Wire.write(PCF85063A_SECONDS_REGISTER);
    Wire.write(encode_bcd(0));               // seconds, also clears the oscillator-stop/invalid flag
    Wire.write(encode_bcd(minute));
    Wire.write(encode_bcd(hour));
    Wire.write(encode_bcd(static_cast<int>(day)));
    Wire.write(0x00);                         // weekday, unused
    Wire.write(encode_bcd(static_cast<int>(month)));
    Wire.write(encode_bcd(year - PCF85063A_YEAR_OFFSET));
    if (Wire.endTransmission() != 0) {
        Serial.println("RTC: PCF85063A write failed");
        return;
    }

    int64_t epoch = days_from_civil(year, month, day) * 86400 + hour * 3600 + minute * 60;
    timeval systemTime = {static_cast<time_t>(epoch), 0};
    settimeofday(&systemTime, nullptr);
    Serial.printf("RTC: PCF85063A set to %04d-%02d-%02u %02d:%02d:00 UTC\n", year, month, day, hour, minute);
}

static bool hardware_display_init() {
    constexpr size_t BUFFER_LINES = 40;

    hardwarePanel.init();
    hardwarePanel.begin();
    uint8_t outputLevels = expander_usb_can_levels();
    if (!expander_write_io(outputLevels)) {
        Serial.println("EXPANDER: failed to restore USB routing after panel initialization");
    }
    hardwareLcd = hardwarePanel.getLcd();
    if (hardwareLcd == nullptr) {
        Serial.println("DISPLAY: LCD initialization failed");
        return false;
    }

    lv_init();
    hardwareDisplay = lv_display_create(800, 480);
    lv_display_set_color_format(hardwareDisplay, LV_COLOR_FORMAT_RGB565);
    lvglFrameBuffer = static_cast<uint8_t *>(
        heap_caps_malloc(800 * BUFFER_LINES * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    lvglFrameBuffer2 = static_cast<uint8_t *>(
        heap_caps_malloc(800 * BUFFER_LINES * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    if (lvglFrameBuffer == nullptr || lvglFrameBuffer2 == nullptr) {
        Serial.println("DISPLAY: frame buffer allocation failed");
        return false;
    }
    lv_display_set_buffers(hardwareDisplay, lvglFrameBuffer, lvglFrameBuffer2,
                            800 * BUFFER_LINES * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(hardwareDisplay, hardware_flush_cb);
    lv_display_set_default(hardwareDisplay);

    hardwareTouch = hardwarePanel.getLcdTouch();
    if (hardwareTouch == nullptr) {
        Serial.println("DISPLAY: touch initialization failed, screen will be view-only");
    } else {
        hardwareTouchIndev = lv_indev_create();
        lv_indev_set_type(hardwareTouchIndev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(hardwareTouchIndev, hardware_touch_read_cb);
    }
    return true;
}

static bool sd_log_init() {
    uint8_t outputLevels = expander_usb_can_levels();
    if (!expander_write_io(outputLevels)) {
        Serial.println("SD: failed to preserve selected USB/CAN route through CH422G");
        return false;
    }

    if (!SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0) ||
        !SD_MMC.begin("/sdcard", true, false)) {
        Serial.println("SD: mount failed; CAN capture disabled");
        SD_MMC.end();
        return false;
    }

    canLogFile = SD_MMC.open(CAN_LOG_PATH, FILE_APPEND);
    if (!canLogFile) {
        Serial.println("SD: failed to open CAN capture file");
        SD_MMC.end();
        return false;
    }

    if (canLogFile.size() == 0) {
        canLogFile.println("timestamp_ms,can_id,extended,dlc,data0,data1,data2,data3,data4,data5,data6,data7");
    }
    canLogFile.printf("# capture started at %lu ms\n", millis());
    canLogFile.flush();
    sdLoggingEnabled = true;
    Serial.printf("SD: logging raw CAN frames to %s\n", CAN_LOG_PATH);
    return true;
}

static void queue_can_log(const twai_message_t &message) {
    if (!sdLoggingEnabled || canLogQueue == NULL) {
        ++canLogDroppedFrames;
        return;
    }

    CanLogRecord record = {millis(), message};
    if (xQueueSend(canLogQueue, &record, 0) != pdTRUE) {
        ++canLogDroppedFrames;
    }
}

static void SDLoggerLoop(void *) {
    if (canLogQueue == NULL) {
        Serial.println("SD: failed to create CAN capture queue");
        vTaskDelete(NULL);
        return;
    }

    unsigned long now = millis();
    unsigned long lastFlush = now;
    unsigned long lastMountAttempt = now - SD_LOG_RETRY_INTERVAL_MS;
    unsigned long lastStatus = now;
    CanLogRecord record;

    while (true) {
        now = millis();
        if (!sdLoggingEnabled && now - lastMountAttempt >= SD_LOG_RETRY_INTERVAL_MS) {
            lastMountAttempt = now;
            if (sd_log_init()) {
                lastFlush = now;
            }
        }

        if (sdLoggingEnabled) {
            if (xQueueReceive(canLogQueue, &record, pdMS_TO_TICKS(100)) == pdTRUE) {
                const twai_message_t &message = record.message;
                canLogFile.printf("%lu,0x%08lX,%u,%u",
                                  record.timestampMs,
                                  static_cast<unsigned long>(message.identifier),
                                  message.extd ? 1 : 0,
                                  message.data_length_code);
                for (uint8_t dataIndex = 0; dataIndex < 8; ++dataIndex) {
                    if (!message.rtr && dataIndex < message.data_length_code) {
                        canLogFile.printf(",%02X", message.data[dataIndex]);
                    } else {
                        canLogFile.print(',');
                    }
                }
                canLogFile.println();
            }

            now = millis();
            if (now - lastFlush >= SD_LOG_FLUSH_INTERVAL_MS) {
                canLogFile.flush();
                lastFlush = now;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        now = millis();
        if (now - lastStatus >= 5000) {
            Serial.printf("SD: %s; queued=%u; dropped_frames=%lu\n",
                          sdLoggingEnabled ? "capture active" : "capture unavailable",
                          static_cast<unsigned>(uxQueueMessagesWaiting(canLogQueue)),
                          static_cast<unsigned long>(canLogDroppedFrames));
            lastStatus = now;
        }
    }
}

// =========================================================================
// BLE - custom GATT service broadcasting raw motor telemetry
// =========================================================================
BLEServer *bleServer = nullptr;
BLECharacteristic *telemetryCharacteristic = nullptr;
bool bleClientConnected = false;

class TelemetryServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *server) override { bleClientConnected = true; }
    void onDisconnect(BLEServer *server) override {
        bleClientConnected = false;
        server->getAdvertising()->start();
    }
};

static void ble_init() {
    BLEDevice::init("BafangBLE-M620");
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(new TelemetryServerCallbacks());

    BLEService *service = bleServer->createService(BLE_SERVICE_UUID);
    telemetryCharacteristic = service->createCharacteristic(
        BLE_TELEMETRY_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    telemetryCharacteristic->addDescriptor(new BLE2902());
    service->start();

    BLEAdvertising *advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    BLEDevice::startAdvertising();
}

static void ble_notify_telemetry() {
    if (!bleClientConnected || telemetryCharacteristic == nullptr) {
        return;
    }
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    BleTelemetryPacket packet = {
        liveData.speedKmh,
        liveData.cadenceRpm,
        liveData.powerWatts,
        liveData.assistLevel,
        liveData.motorTempC,
        liveData.batteryVoltage,
        liveData.batteryCurrent,
        liveData.batterySocPercent,
        liveData.errorCode,
    };
    xSemaphoreGive(dataMutex);

    telemetryCharacteristic->setValue(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    telemetryCharacteristic->notify();
}

// =========================================================================
// CORE 0: CAN BUS READING - decodes Bafang M620 motor/battery frames
// =========================================================================
static bool can_init() {
    twai_general_config_t generalConfig =
        // LISTEN_ONLY: TX is fully disabled at the hardware level - no ACKs, no error
        // frames, no bus-off recovery attempts. Safe to tap into the bike's live CAN bus.
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_LISTEN_ONLY);
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) != ESP_OK) {
        Serial.println("CAN: driver install failed");
        return false;
    }
    if (twai_start() != ESP_OK) {
        Serial.println("CAN: start failed");
        return false;
    }
    return true;
}

static void decode_motor_telemetry(const twai_message_t &msg) {
    liveData.powerWatts = static_cast<uint16_t>(msg.data[0]) |
                          (static_cast<uint16_t>(msg.data[1]) << 8);
    liveData.cadenceRpm = static_cast<uint16_t>(msg.data[2]) |
                          (static_cast<uint16_t>(msg.data[3]) << 8);
}

void CANProcessingLoop(void *pvParameters) {
    if (!CAN_ROUTE_ENABLED || !canDriverStarted) {
        Serial.println("CAN: processing task stopped because TWAI is not initialized");
        vTaskDelete(NULL);
        return;
    }
    Serial.printf("CAN: listen-only started at 250 kbps on TX=%d/RX=%d\n", PIN_CAN_TX, PIN_CAN_RX);

    unsigned long lastValidFrame = 0;
    unsigned long lastStatus = millis();
    uint32_t receivedFrames = 0;

    while (true) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            ++receivedFrames;
            queue_can_log(rx_msg);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            lastValidFrame = millis();
            liveData.canActive = true;

            if (rx_msg.extd && !rx_msg.rtr && rx_msg.data_length_code >= 4 &&
                (rx_msg.identifier & 0xFFFFU) == CAN_TELEMETRY_ID_SUFFIX) {
                decode_motor_telemetry(rx_msg);
            }
            xSemaphoreGive(dataMutex);
        }

        if (millis() - lastStatus >= 5000) {
            twai_status_info_t status = {};
            if (twai_get_status_info(&status) == ESP_OK) {
                Serial.printf("CAN: rx=%lu state=%u rx_err=%u tx_err=%u bus_err=%u missed=%u overrun=%u\n",
                              static_cast<unsigned long>(receivedFrames),
                              static_cast<unsigned>(status.state),
                              static_cast<unsigned>(status.rx_error_counter),
                              static_cast<unsigned>(status.tx_error_counter),
                              static_cast<unsigned>(status.bus_error_count),
                              static_cast<unsigned>(status.rx_missed_count),
                              static_cast<unsigned>(status.rx_overrun_count));
            } else {
                Serial.printf("CAN: rx=%lu; unable to read TWAI status\n",
                              static_cast<unsigned long>(receivedFrames));
            }
            lastStatus = millis();
        }

        if (liveData.canActive && millis() - lastValidFrame > 2000) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            liveData.canActive = false;
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =========================================================================
// CORE 1: UI REFRESH + BLE NOTIFY
// =========================================================================
void UIProcessingLoop(void *pvParameters) {
    unsigned long lastBleNotify = 0;
    unsigned long lastTick = millis();

    while (true) {
        unsigned long now = millis();
        lv_tick_inc(now - lastTick);
        lastTick = now;

        xSemaphoreTake(dataMutex, portMAX_DELAY);
        MotorMetrics snapshot = const_cast<MotorMetrics &>(liveData);
        xSemaphoreGive(dataMutex);

        update_dashboard(snapshot);

        if (now - lastBleNotify >= BLE_NOTIFY_INTERVAL_MS) {
            ble_notify_telemetry();
            lastBleNotify = now;
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =========================================================================
// ENTRY POINTS
// =========================================================================
void setup() {
    Serial.begin(115200);

     delay(2000);

    // LISTEN_ONLY mode keeps TX high/recessive so it cannot bring down the bus
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, 
        CAN_RX_PIN, 
        TWAI_MODE_LISTEN_ONLY
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK &&
        twai_start() == ESP_OK) {
        Serial.println("TWAI Driver running in LISTEN_ONLY mode.");
    } else {
        Serial.println("TWAI Initialization Failed!");
    }

    dataMutex = xSemaphoreCreateMutex();
    touchMutex = xSemaphoreCreateMutex();
    canLogQueue = xQueueCreate(CAN_LOG_QUEUE_LENGTH, sizeof(CanLogRecord));

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        Serial.println("I2C: Wire.begin failed");
    } else {
        Wire.setClock(400000);
    }

    if (!expander_init()) {
        Serial.println("EXPANDER: initialization failed, USB routing or panel controls may misbehave");
    }

    rtc_sync_system_time();

    if (CAN_ROUTE_ENABLED) {
        canDriverStarted = can_init();
        if (!canDriverStarted) {
            Serial.println("CAN: initialization failed; motor data will be unavailable");
        } else {
            xTaskCreatePinnedToCore(CANProcessingLoop, "CANTask", 4096, NULL, 1, &CANTaskHandle, 0);
        }
    }

    if (!hardware_display_init()) {
        Serial.println("DISPLAY: initialization failed");
    } else {
        set_rtc_write_callback(rtc_write_time);
        create_dashboard();
    }

    xTaskCreatePinnedToCore(TouchProcessingLoop, "TouchTask", 3072, NULL, 1, &TouchTaskHandle, 0);
    xTaskCreatePinnedToCore(UIProcessingLoop, "UITask", 8192, NULL, 1, &UITaskHandle, 1);

    ble_init(); // ble_notify_telemetry() is null-safe, so deferring this is safe

    // SDMMC starts last on a low-priority task so a slow card cannot block LVGL startup.
    xTaskCreatePinnedToCore(SDLoggerLoop, "SDLogger", 4096, NULL, 0, &SDLoggerTaskHandle, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}