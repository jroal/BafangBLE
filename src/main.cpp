#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
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
// CONFIGURATION - BAFANG UART SERIAL
// =========================================================================
static const gpio_num_t BAFANG_UART_RX_PIN = static_cast<gpio_num_t>(UART_RX_PIN);
static const gpio_num_t BAFANG_UART_TX_PIN = static_cast<gpio_num_t>(UART_TX_PIN);

struct UartModeConfig {
    uint32_t baud;
    bool invert;
    const char *name;
};

static const UartModeConfig UART_MODES[] = {
    {1200, false, "1200 Baud (RS485)"}
};
static const size_t NUM_UART_MODES = sizeof(UART_MODES) / sizeof(UART_MODES[0]);

static const unsigned long BLE_NOTIFY_INTERVAL_MS = 500;
static const unsigned long SD_LOG_FLUSH_INTERVAL_MS = 1000;

static constexpr uint8_t PCF85063A_I2C_ADDRESS = 0x51;
static constexpr uint8_t PCF85063A_CTRL1_REGISTER = 0x00;
static constexpr uint8_t PCF85063A_SECONDS_REGISTER = 0x04;
static constexpr int PCF85063A_YEAR_OFFSET = 1970;
static constexpr uint8_t CH422G_IO_WRITE_ADDRESS = 0x38;
static constexpr const char *UART_LOG_PATH = "/uart_capture.csv";
static constexpr const char *ERROR_LOG_PATH = "/error.txt";
static constexpr UBaseType_t UART_LOG_QUEUE_LENGTH = 1024;
static constexpr UBaseType_t ERROR_LOG_QUEUE_LENGTH = 32;
static constexpr unsigned long SD_LOG_RETRY_INTERVAL_MS = 5000;
static constexpr unsigned long SD_MOUNT_RETRY_INTERVAL_MS = 60000;

#define BLE_SERVICE_UUID        "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define BLE_TELEMETRY_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"

HardwareSerial BafangSerial(1); // Hardware UART1 controller

SemaphoreHandle_t dataMutex;
volatile MotorMetrics liveData;

#pragma pack(push, 1)
struct BleTelemetryPacket {
    float speedKmh;
    uint16_t cadenceRpm;
    uint16_t motorPowerWatts;
    uint8_t assistLevel;
    int8_t motorTempC;
    float batteryVoltage;
    float batteryCurrent;
    uint8_t batterySocPercent;
    uint8_t errorCode;
};
#pragma pack(pop)

TaskHandle_t UARTTaskHandle = NULL;
TaskHandle_t UITaskHandle = NULL;
TaskHandle_t SDLoggerTaskHandle = NULL;

struct UartLogRecord {
    time_t timestamp;
    uint32_t timestampMs;
    uint8_t length;
    uint8_t data[32];
};

struct ErrorLogRecord {
    time_t timestamp;
    uint32_t timestampMs;
    char message[128];
};

QueueHandle_t uartLogQueue = NULL;
QueueHandle_t errorLogQueue = NULL;
File uartLogFile;
File errorLogFile;
volatile bool sdCardMounted = false;
volatile bool sdLoggingEnabled = false;
volatile uint32_t uartLogDroppedBytes = 0;
volatile uint32_t errorLogDroppedMessages = 0;
bool uartDriverStarted = false;
bool sdSpiStarted = false;

static void log_error(const char *message) {
    if (errorLogQueue == NULL) {
        ++errorLogDroppedMessages;
        return;
    }

    ErrorLogRecord record = {};
    record.timestamp = time(nullptr);
    record.timestampMs = millis();
    snprintf(record.message, sizeof(record.message), "%s", message);
    if (xQueueSend(errorLogQueue, &record, 0) != pdTRUE) {
        ++errorLogDroppedMessages;
    }
}

static void format_log_timestamp(time_t timestamp, uint32_t uptimeMs, char *buffer, size_t bufferSize) {
    struct tm utcTime;
    if (timestamp > 0 && gmtime_r(&timestamp, &utcTime) != nullptr &&
        strftime(buffer, bufferSize, "%Y-%m-%d %H:%M:%S UTC", &utcTime) > 0) {
        return;
    }
    snprintf(buffer, bufferSize, "uptime %lu ms", static_cast<unsigned long>(uptimeMs));
}

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
        log_error("EXPANDER: failed to write output enable");
        return false;
    }
    Wire.beginTransmission(CH422G_IO_WRITE_ADDRESS);
    Wire.write(outputLevels);
    return Wire.endTransmission() == 0;
}

static uint8_t expander_usb_can_levels() {
    return 0xCFU;
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

static bool expander_init() {
    if (!expander_write_io(expander_usb_can_levels())) {
        log_error("EXPANDER: output level write failed");
        return false;
    }
    
    log_error("EXPANDER: Initialized successfully");
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
        log_error("RTC: PCF85063A read failed");
        return false;
    }

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
        log_error("RTC: invalid or unset PCF85063A time");
        return false;
    }

    year += PCF85063A_YEAR_OFFSET;
    if (second > 59 || minute > 59 || hour > 23 || month < 1 || month > 12 || day < 1) {
        log_error("RTC: out-of-range PCF85063A time");
        return false;
    }

    static const uint8_t daysPerMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    uint8_t monthLength = daysPerMonth[month - 1];
    if (month == 2 && (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0))) {
        ++monthLength;
    }
    if (day > monthLength) {
        log_error("RTC: invalid PCF85063A calendar date");
        return false;
    }

    int64_t epoch = days_from_civil(year, month, day) * 86400 + hour * 3600 + minute * 60 + second;
    timeval systemTime = {static_cast<time_t>(epoch), 0};
    if (settimeofday(&systemTime, nullptr) != 0) {
        log_error("RTC: settimeofday failed");
        return false;
    }
    return true;
}

static void rtc_write_time(int year, unsigned month, unsigned day, int hour, int minute) {
    Wire.beginTransmission(PCF85063A_I2C_ADDRESS);
    Wire.write(PCF85063A_CTRL1_REGISTER);
    Wire.write(static_cast<uint8_t>(0x00));
    Wire.endTransmission();

    Wire.beginTransmission(PCF85063A_I2C_ADDRESS);
    Wire.write(PCF85063A_SECONDS_REGISTER);
    Wire.write(encode_bcd(0));
    Wire.write(encode_bcd(minute));
    Wire.write(encode_bcd(hour));
    Wire.write(encode_bcd(static_cast<int>(day)));
    Wire.write(0x00);
    Wire.write(encode_bcd(static_cast<int>(month)));
    Wire.write(encode_bcd(year - PCF85063A_YEAR_OFFSET));
    if (Wire.endTransmission() != 0) {
        log_error("RTC: PCF85063A write failed");
        return;
    }

    int64_t epoch = days_from_civil(year, month, day) * 86400 + hour * 3600 + minute * 60;
    timeval systemTime = {static_cast<time_t>(epoch), 0};
    settimeofday(&systemTime, nullptr);
}

static bool hardware_display_init() {
    constexpr size_t BUFFER_LINES = 40;

    hardwarePanel.init();
    hardwarePanel.begin();
    uint8_t outputLevels = expander_usb_can_levels();
    if (!expander_write_io(outputLevels)) {
        log_error("EXPANDER: failed to restore USB routing after panel initialization");
    }
    hardwareLcd = hardwarePanel.getLcd();
    if (hardwareLcd == nullptr) {
        log_error("DISPLAY: LCD initialization failed");
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
        log_error("DISPLAY: frame buffer allocation failed");
        return false;
    }
    lv_display_set_buffers(hardwareDisplay, lvglFrameBuffer, lvglFrameBuffer2,
                            800 * BUFFER_LINES * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(hardwareDisplay, hardware_flush_cb);
    lv_display_set_default(hardwareDisplay);

    hardwareTouch = hardwarePanel.getLcdTouch();
    if (hardwareTouch == nullptr) {
        log_error("DISPLAY: touch initialization failed, screen will be view-only");
    } else {
        hardwareTouchIndev = lv_indev_create();
        lv_indev_set_type(hardwareTouchIndev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(hardwareTouchIndev, hardware_touch_read_cb);
    }
    return true;
}

static bool sd_log_init() {
    // Write the expander once: repeating the I2C write on every retry while no card is
    // present competes with the touch/panel traffic and can glitch the backlight.
    static bool expanderPrepared = false;
    if (!expanderPrepared) {
        uint8_t outputLevels = static_cast<uint8_t>(expander_usb_can_levels() & ~(1U << PIN_SD_CS_EXPANDER));
        if (!expander_write_io(outputLevels)) {
            log_error("SD: failed to preserve selected route through CH422G");
            return false;
        }
        expanderPrepared = true;
    }

    if (!sdSpiStarted) {
        SPI.setHwCs(false);
        SPI.begin(PIN_SD_CLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_DUMMY_CS);
        sdSpiStarted = true;
    }

    if (!SD.begin(PIN_SD_DUMMY_CS, SPI, 20000000, "/sdcard", 5, false)) {
        static bool mountFailureLogged = false;
        if (!mountFailureLogged) {
            log_error("SD: mount failed; error and UART logging unavailable");
            mountFailureLogged = true;
        }
        SD.end();
        return false;
    }

    sdCardMounted = true;
    errorLogFile = SD.open(ERROR_LOG_PATH, FILE_APPEND);
    if (!errorLogFile) {
        log_error("SD: failed to open error.log; queued errors retained for retry");
    } else {
        char timestamp[32];
        format_log_timestamp(time(nullptr), millis(), timestamp, sizeof(timestamp));
        errorLogFile.printf("[%s] SD error log started\n", timestamp);
        errorLogFile.flush();
    }

    return true;
}

static bool open_error_log() {
    if (errorLogFile) {
        return true;
    }
    errorLogFile = SD.open(ERROR_LOG_PATH, FILE_APPEND);
    return static_cast<bool>(errorLogFile);
}

static void queue_uart_log(const uint8_t *data, size_t len) {
    if (uartLogQueue == NULL || len == 0) {
        ++uartLogDroppedBytes;
        return;
    }

    UartLogRecord record;
    record.timestamp = time(nullptr);
    record.timestampMs = millis();
    record.length = (len > sizeof(record.data)) ? sizeof(record.data) : static_cast<uint8_t>(len);
    memcpy(record.data, data, record.length);

    if (xQueueSend(uartLogQueue, &record, 0) != pdTRUE) {
        ++uartLogDroppedBytes;
    }
}

static bool open_uart_log_file() {
    uartLogFile = SD.open(UART_LOG_PATH, FILE_APPEND);
    if (!uartLogFile) {
        log_error("SD: failed to open UART capture file");
        return false;
    } else {
        log_error("SD: UART capture file opened successfully");
    }

    char timestamp[32];
    format_log_timestamp(time(nullptr), millis(), timestamp, sizeof(timestamp));
    uartLogFile.printf("# capture started [%s]\n", timestamp);
    uartLogFile.println("timestamp_utc,timestamp_ms,length,bytes_hex");
    uartLogFile.flush();
    sdLoggingEnabled = true;
    return true;
}

static void SDLoggerLoop(void *) {
    if (errorLogQueue == NULL) {
        vTaskDelete(NULL);
        return;
    }
    if (uartLogQueue == NULL) {
        log_error("SD: failed to create UART capture queue");
    }

    unsigned long now = millis();
    unsigned long lastFlush = now;
    unsigned long lastMountAttempt = now - SD_MOUNT_RETRY_INTERVAL_MS;
    unsigned long lastErrorOpenAttempt = now - SD_LOG_RETRY_INTERVAL_MS;
    unsigned long lastUartOpenAttempt = now - SD_LOG_RETRY_INTERVAL_MS;
    UartLogRecord record;
    ErrorLogRecord errorRecord;

    while (true) {
        now = millis();
        if (!sdCardMounted && now - lastMountAttempt >= SD_MOUNT_RETRY_INTERVAL_MS) {
            lastMountAttempt = now;
            if (sd_log_init()) {
                lastFlush = now;
            }
        }

        if (sdCardMounted) {
            if (!errorLogFile && now - lastErrorOpenAttempt >= SD_LOG_RETRY_INTERVAL_MS) {
                lastErrorOpenAttempt = now;
                open_error_log();
            }

            if (errorLogFile && xQueueReceive(errorLogQueue, &errorRecord, 0) == pdTRUE) {
                char timestamp[32];
                format_log_timestamp(errorRecord.timestamp, errorRecord.timestampMs,
                                     timestamp, sizeof(timestamp));
                errorLogFile.printf("[%s] %s\n", timestamp, errorRecord.message);
                errorLogFile.flush();
            }
        }

        if (sdCardMounted && !sdLoggingEnabled && uartLogQueue != NULL &&
            uxQueueMessagesWaiting(uartLogQueue) > 0 &&
            now - lastUartOpenAttempt >= SD_LOG_RETRY_INTERVAL_MS) {
            lastUartOpenAttempt = now;
            open_uart_log_file();
        }

        if (sdLoggingEnabled && uartLogQueue != NULL) {
            if (xQueueReceive(uartLogQueue, &record, pdMS_TO_TICKS(100)) == pdTRUE) {
                char timestamp[32];
                format_log_timestamp(record.timestamp, record.timestampMs,
                                     timestamp, sizeof(timestamp));
                uartLogFile.printf("%s,%lu,%u,", timestamp, record.timestampMs, record.length);
                for (uint8_t i = 0; i < record.length; ++i) {
                    uartLogFile.printf("%02X", record.data[i]);
                }
                uartLogFile.println();
            }

            now = millis();
            if (now - lastFlush >= SD_LOG_FLUSH_INTERVAL_MS) {
                uartLogFile.flush();
                lastFlush = now;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
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
    BLEDevice::init("BafangBLE-UART");
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
        liveData.motorPowerWatts,
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
// CORE 0: BAFANG UART READING & DECODING
// =========================================================================
static bool apply_uart_mode(size_t modeIdx) {
    if (modeIdx >= NUM_UART_MODES) return false;
    
    BafangSerial.end();
    delay(20);
    
    const UartModeConfig &cfg = UART_MODES[modeIdx];
    BafangSerial.begin(cfg.baud, SERIAL_8N1, BAFANG_UART_RX_PIN, BAFANG_UART_TX_PIN, cfg.invert);
    
    char buf[128];
    snprintf(buf, sizeof(buf), "UART: Initialized Mode %u -> %s on RX Pin %d, TX Pin %d",
             (unsigned)modeIdx, cfg.name, (int)BAFANG_UART_RX_PIN, (int)BAFANG_UART_TX_PIN);
    log_error(buf);
    return true;
}

static bool uart_init() {
    // Start with the standard non-inverted RS485 UART mode.
    return apply_uart_mode(0);
}

// Controller reply to the display's 0x11 0x20 poll: 00 <speed> <speed+0x20>.
// Scale calibrated from a single point (raw 197 = 16.5 mph); refine with more data.
static constexpr float SPEED_KMH_PER_COUNT = 26.55f / 197.0f;
// Motor power reply (<n> <n>) scale from one point: peak raw 0x3C (60) at ~1500 W on 56.7 V.
static constexpr uint16_t MOTOR_WATTS_PER_COUNT = 25;
// Pack voltage from the 6-byte status frame <hi> <lo> 00 00 <sum> 01 (hi = 02): 56.7 V read as 0x02F4 (756).
static constexpr float VOLTS_PER_COUNT = 56.7f / 756.0f;

// Decodes controller replies on the display link (1200 baud). Replies carry no
// command echo, so the battery reply is identified by following the speed reply.
static bool parse_bafang_uart_packet(const uint8_t *buf, size_t len) {
    static bool expectBattery = false;
    static bool expectAssist = false;
    static bool expectMotorPower = false;

    if (len == 6 && buf[0] == 0x02 && buf[3] == 0x00 && buf[5] == 0x01 &&
        static_cast<uint8_t>(buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
        float volts = ((buf[0] << 8) | buf[1]) * VOLTS_PER_COUNT;
        if (volts > 30.0f && volts < 75.0f) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            liveData.batteryVoltage = volts;
            xSemaphoreGive(dataMutex);
        }
        return true;
    }

    // The 01/03 status flag precedes the motor power reply.
    if (len == 1 && (buf[0] == 0x01 || buf[0] == 0x03)) {
        expectMotorPower = true;
        return true;
    }

    if (expectMotorPower && len == 2 && buf[0] == buf[1]) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        liveData.motorPowerWatts = buf[0] * MOTOR_WATTS_PER_COUNT;
        xSemaphoreGive(dataMutex);
        expectMotorPower = false;
        return true;
    }

    if (len == 3 && buf[0] == 0x00 && buf[2] == static_cast<uint8_t>(buf[1] + 0x20)) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        liveData.speedKmh = buf[1] * SPEED_KMH_PER_COUNT;
        xSemaphoreGive(dataMutex);
        expectBattery = true;
        expectAssist = false;
        return true;
    }

    // Assist level reply: 00 <v> <v>, where v falls as the level rises. It follows the
    // battery reply; the odometer-like reply after it has the same shape, so gate on order.
    if (expectAssist && len == 3 && buf[0] == 0x00 && buf[1] == buf[2]) {
        uint8_t level = 0;
        switch (buf[1]) {
            case 0xFF: level = 0; break;
            case 0x81: level = 1; break;
            case 0x64: level = 2; break;
            case 0x4B: level = 3; break;
            case 0x38: level = 4; break;
            case 0x2D: level = 5; break;
            default: return false;
        }
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        liveData.assistLevel = level;
        xSemaphoreGive(dataMutex);
        expectAssist = false;
        return true;
    }

    if (expectBattery && len == 2 && buf[0] == buf[1] && buf[0] <= 100) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        liveData.batterySocPercent = buf[0];
        xSemaphoreGive(dataMutex);
        expectBattery = false;
        expectAssist = true;
        return true;
    }

    return false;
}

void UARTProcessingLoop(void *pvParameters) {
    while (!uartDriverStarted) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    log_error("UARTProcessingLoop active!");

    unsigned long lastValidFrame = 0;
    unsigned long lastModeSwitch = millis();
    size_t currentModeIdx = 0;
    bool modeLocked = false;

    uint8_t rxBuffer[64];
    size_t rxIndex = 0;
    unsigned long lastByteTime = 0;

    while (true) {
        // Read all available bytes from serial
        while (BafangSerial.available() > 0) {
            uint8_t byteIn = BafangSerial.read();

            if (rxIndex < sizeof(rxBuffer)) {
                rxBuffer[rxIndex++] = byteIn;
            }
            lastByteTime = millis();
        }

        // Frame boundary check: 20ms silence or buffer full
        if (rxIndex > 0 && (millis() - lastByteTime > 20 || rxIndex >= sizeof(rxBuffer))) {
            // Queue frame to SD card
            queue_uart_log(rxBuffer, rxIndex);

            // Valid frame received (more than 2 bytes)
            if (rxIndex >= 2 && parse_bafang_uart_packet(rxBuffer, rxIndex)) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                lastValidFrame = millis();
                liveData.canActive = true;
                xSemaphoreGive(dataMutex);

                if (!modeLocked) {
                    modeLocked = true;
                    char buf[128];
                    snprintf(buf, sizeof(buf), "[UART AUTO-SCAN] LOCKED onto Mode %u (%s)! Valid data streaming.",
                             (unsigned)currentModeIdx, UART_MODES[currentModeIdx].name);
                    log_error(buf);
                }

            }

            rxIndex = 0; // Reset buffer
        }

        // Auto-scan mode switcher: if no valid multi-byte packets received after 6 seconds, try next mode
        if (NUM_UART_MODES > 1 && !modeLocked && (millis() - lastModeSwitch > 6000)) {
            currentModeIdx = (currentModeIdx + 1) % NUM_UART_MODES;
            apply_uart_mode(currentModeIdx);
            lastModeSwitch = millis();
            rxIndex = 0;
        }

        // Timeout status indicator for UI
        if (liveData.canActive && millis() - lastValidFrame > 2000) {
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            liveData.canActive = false;
            xSemaphoreGive(dataMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
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
    delay(1000);

    errorLogQueue = xQueueCreate(ERROR_LOG_QUEUE_LENGTH, sizeof(ErrorLogRecord));
    uartLogQueue = xQueueCreate(UART_LOG_QUEUE_LENGTH, sizeof(UartLogRecord));
    dataMutex = xSemaphoreCreateMutex();
    touchMutex = xSemaphoreCreateMutex();

    if (!Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL)) {
        log_error("I2C: Wire.begin failed");
    } else {
        Wire.setClock(400000);
    }

    if (!expander_init()) {
        log_error("EXPANDER: initialization failed, USB routing or panel controls may misbehave");
    }

    rtc_sync_system_time();

    uartDriverStarted = uart_init();
    if (uartDriverStarted) {
        xTaskCreatePinnedToCore(UARTProcessingLoop, "UARTTask", 4096, NULL, 1, &UARTTaskHandle, 0);
    } else {
        log_error("UART: initialization failed; motor data will be unavailable");
    }

    if (!hardware_display_init()) {
        log_error("DISPLAY: initialization failed");
    } else {
        set_rtc_write_callback(rtc_write_time);
        create_dashboard();
    }

    xTaskCreatePinnedToCore(TouchProcessingLoop, "TouchTask", 3072, NULL, 1, &TouchTaskHandle, 0);
    xTaskCreatePinnedToCore(UIProcessingLoop, "UITask", 8192, NULL, 1, &UITaskHandle, 1);

    ble_init();
    xTaskCreatePinnedToCore(SDLoggerLoop, "SDLogger", 4096, NULL, 0, &SDLoggerTaskHandle, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
