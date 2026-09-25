#include <Arduino.h>
#include <driver/i2c.h>
#include <driver/twai.h>    // ESP32 native CAN driver (Bafang M620 uses CAN)
#include <SD.h>
#include <SPI.h>
#include <ESP_Panel_Library.h>
#include <esp_heap_caps.h>
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

// =========================================================================
// CONFIGURATION - verify against your wiring / captured bus traffic
// =========================================================================
// TODO: confirm these against the actual CAN transceiver wiring on your board.
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_4;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_6;
static const uint32_t CAN_BITRATE = 500000; // TODO: verify actual M620 CAN bitrate (500 kbps assumed)

// TODO: these CAN identifiers and byte layouts are placeholders. Bafang has not
// published the M620 CAN protocol; capture real traffic (e.g. with a CAN sniffer)
// and replace the IDs/scaling below with verified values before relying on this data.
static const uint32_t CAN_ID_MOTOR_STATUS = 0x300; // speed, cadence, assist level, error code
static const uint32_t CAN_ID_MOTOR_POWER = 0x301;  // power, torque, motor temp
static const uint32_t CAN_ID_BATTERY_STATUS = 0x310; // voltage, current, state of charge

static const unsigned long BLE_NOTIFY_INTERVAL_MS = 500;
static const unsigned long SD_LOG_FLUSH_INTERVAL_MS = 1000;

// Waveshare ESP32-S3-Touch-LCD-4.3B onboard TF-card SPI wiring.
static const int SD_SPI_SCK_PIN = 12;
static const int SD_SPI_MISO_PIN = 13;
static const int SD_SPI_MOSI_PIN = 11;
static const uint8_t CH422G_IO_WRITE_ADDRESS = 0x38;
static constexpr const char *CAN_LOG_PATH = "/can_capture.csv";

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
bool sdLoggingEnabled = false;

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

// Brings up the CH422G expander's I2C bus and releases TP_RST/LCD_RST (driven high here)
// before the touch/LCD peripherals are touched - GT911 stays unresponsive if this runs late.
static bool expander_init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = ESP_PANEL_LCD_TOUCH_I2C_IO_SDA;
    conf.scl_io_num = ESP_PANEL_LCD_TOUCH_I2C_IO_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    if (i2c_param_config(I2C_NUM_0, &conf) != ESP_OK || i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0) != ESP_OK) {
        Serial.println("EXPANDER: I2C bus initialization failed");
        return false;
    }

    uint8_t outputLevels = 0xFF;
    if (i2c_master_write_to_device(I2C_NUM_0, CH422G_IO_WRITE_ADDRESS, &outputLevels,
                                   sizeof(outputLevels), pdMS_TO_TICKS(10)) != ESP_OK) {
        Serial.println("EXPANDER: output level write failed");
        return false;
    }
    return true;
}

static bool hardware_display_init() {
    constexpr size_t BUFFER_LINES = 40;

    hardwarePanel.init();
    hardwarePanel.begin();
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
    // The board's TF CS is GPIO10, which is also the LCD RGB data4 pin.
    // Do not claim it while the display is active.
    return false;
}

static void queue_can_log(const twai_message_t &message) {
    if (!sdLoggingEnabled) {
        return;
    }

    CanLogRecord record = {millis(), message};
    xQueueSend(canLogQueue, &record, 0);
}

static void SDLoggerLoop(void *) {
    canLogQueue = xQueueCreate(256, sizeof(CanLogRecord));
    if (canLogQueue == NULL || !sd_log_init()) {
        vTaskDelete(NULL);
        return;
    }

    unsigned long lastFlush = millis();
    CanLogRecord record;

    while (true) {
        if (xQueueReceive(canLogQueue, &record, pdMS_TO_TICKS(100)) == pdTRUE) {
            const twai_message_t &message = record.message;
            canLogFile.printf(
                "%lu,0x%03lX,%u,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
                record.timestampMs,
                static_cast<unsigned long>(message.identifier),
                message.extd ? 1 : 0,
                message.data_length_code,
                message.data[0], message.data[1], message.data[2], message.data[3],
                message.data[4], message.data[5], message.data[6], message.data[7]);
        }

        if (millis() - lastFlush >= SD_LOG_FLUSH_INTERVAL_MS) {
            canLogFile.flush();
            lastFlush = millis();
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
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
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

// TODO: byte offsets/scaling below are placeholders pending verified M620 CAN spec.
static void decode_motor_status(const twai_message_t &msg) {
    liveData.speedKmh = msg.data[0] / 10.0f;   // assumed 0.1 km/h units
    liveData.cadenceRpm = msg.data[1];
    liveData.assistLevel = msg.data[2] & 0x0F;
    liveData.errorCode = msg.data[3];
}

static void decode_motor_power(const twai_message_t &msg) {
    liveData.powerWatts = (uint16_t)(msg.data[0] << 8 | msg.data[1]);
    liveData.motorTempC = (int8_t)msg.data[2] - 40; // assumed offset like automotive CAN temp encoding
}

static void decode_battery_status(const twai_message_t &msg) {
    liveData.batteryVoltage = ((msg.data[0] << 8) | msg.data[1]) / 100.0f; // assumed 0.01V units
    int16_t currentRaw = (int16_t)((msg.data[2] << 8) | msg.data[3]);
    liveData.batteryCurrent = currentRaw / 100.0f; // assumed 0.01A units
    liveData.batterySocPercent = msg.data[4];
}

void CANProcessingLoop(void *pvParameters) {
    unsigned long lastValidFrame = 0;

    while (true) {
        twai_message_t rx_msg;
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
            queue_can_log(rx_msg);
            xSemaphoreTake(dataMutex, portMAX_DELAY);
            lastValidFrame = millis();
            liveData.canActive = true;

            switch (rx_msg.identifier) {
                case CAN_ID_MOTOR_STATUS:
                    decode_motor_status(rx_msg);
                    break;
                case CAN_ID_MOTOR_POWER:
                    decode_motor_power(rx_msg);
                    break;
                case CAN_ID_BATTERY_STATUS:
                    decode_battery_status(rx_msg);
                    break;
                default:
                    break;
            }
            xSemaphoreGive(dataMutex);
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

    dataMutex = xSemaphoreCreateMutex();
    touchMutex = xSemaphoreCreateMutex();

    if (!expander_init()) {
        Serial.println("EXPANDER: initialization failed, touch/backlight/SD may misbehave");
    }

    if (!can_init()) {
        Serial.println("CAN: initialization failed, motor data will be unavailable");
    }
    ble_init();

    if (!hardware_display_init()) {
        Serial.println("DISPLAY: initialization failed");
    } else {
        create_dashboard();
    }

    xTaskCreatePinnedToCore(CANProcessingLoop, "CANTask", 4096, NULL, 1, &CANTaskHandle, 0);
    xTaskCreatePinnedToCore(TouchProcessingLoop, "TouchTask", 3072, NULL, 1, &TouchTaskHandle, 0);
    xTaskCreatePinnedToCore(UIProcessingLoop, "UITask", 8192, NULL, 1, &UITaskHandle, 1);
    // Runs its own SD.begin() - kept off the boot path since a slow/stuck card must not
    // delay UITask startup (previously froze the whole boot with no display ever drawn).
    xTaskCreatePinnedToCore(SDLoggerLoop, "SDLogger", 4096, NULL, 1, &SDLoggerTaskHandle, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}