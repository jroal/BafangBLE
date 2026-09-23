#include <Arduino.h>
#include <driver/twai.h>    // ESP32 native CAN driver (Bafang M620 uses CAN)
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

// =========================================================================
// DISPLAY / LVGL
// =========================================================================
ESP_Panel hardwarePanel;
ESP_PanelLcd *hardwareLcd = nullptr;
lv_display_t *hardwareDisplay = nullptr;
static uint8_t *lvglFrameBuffer = nullptr;
static uint8_t *lvglFrameBuffer2 = nullptr;

static void hardware_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *pixelMap) {
    if (hardwareLcd != nullptr) {
        hardwareLcd->drawBitmap(area->x1, area->y1, area->x2 + 1, area->y2 + 1, pixelMap);
    }
    lv_display_flush_ready(display);
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
    return true;
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

    while (true) {
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        MotorMetrics snapshot = const_cast<MotorMetrics &>(liveData);
        xSemaphoreGive(dataMutex);

        update_dashboard(snapshot);

        unsigned long now = millis();
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
    xTaskCreatePinnedToCore(UIProcessingLoop, "UITask", 8192, NULL, 1, &UITaskHandle, 1);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}