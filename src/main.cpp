#include <Arduino.h>

#include <TFT_eSPI.h>

#include <PubSubClient.h>

// #include <Wire.h>
#include <WiFi.h>
// #include <FS.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <BLEDevice.h>

#include "lvgl.h"

#include "constants.h"

TFT_eSPI tft = TFT_eSPI();

lv_disp_draw_buf_t draw_buf;
lv_color_t lv_buf[TFT_WIDTH * TFT_HEIGHT / 10];
lv_disp_drv_t driver;
lv_obj_t *label, *label2, *textarea, *meter1, *meter2;
lv_obj_t *label_meter1, *label_meter2;
lv_style_t meter_style, main_style;

lv_meter_scale_t *scale1, *scale1x, *scale2, *scale2x;
lv_meter_indicator_t *indic1, *indic2;

WiFiClient wifiClient;
PubSubClient mqtt_client(wifiClient);

BLEScan* scan;

bool ble_found = false;

SemaphoreHandle_t gui_mu;

class MutexLock {
    public:
        MutexLock(SemaphoreHandle_t mu) : mu_(mu) {
            xSemaphoreTake(mu_, portMAX_DELAY);
        }
        ~MutexLock() { xSemaphoreGive(mu_); }

    private:
        SemaphoreHandle_t mu_;
};

void read_gvh(BLEAdvertisedDevice& device);

class AdvertisedDeviceCallback : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice device) {
        if (!device.haveName()) return;
        if (device.getName().substr(0, 7) == "GVH5102" ||
            device.getName().substr(0, 7) == "GVH5101") {
            ble_found = true;
            read_gvh(device);
        }
    }
};

void init_tft() {
    tft.init();
    tft.setRotation(1);
}

void flush_display(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color) {
    tft.pushImage(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (uint16_t *)color);
    lv_disp_flush_ready(disp);
}

void init_gui() {
    lv_init();
    lv_disp_draw_buf_init(&draw_buf, lv_buf, NULL, sizeof(lv_buf) / sizeof(lv_color_t));

    lv_disp_drv_init(&driver);
    driver.flush_cb = flush_display;
    driver.draw_buf = &draw_buf;
    driver.hor_res = TFT_HEIGHT;
    driver.ver_res = TFT_WIDTH;
    driver.antialiasing = 0;
    lv_disp_drv_register(&driver);

    lv_style_init(&main_style);
    lv_style_set_bg_color(&main_style, lv_color_black());
    lv_style_set_text_color(&main_style, lv_color_white());
    lv_obj_add_style(lv_scr_act(), &main_style, 0);

    label = lv_label_create(lv_scr_act());
    lv_obj_set_x(label, 0);
    lv_obj_set_y(label, 0);
    lv_obj_set_size(label, 120, 16);

    label2 = lv_label_create(lv_scr_act());
    lv_obj_set_x(label2, 120);
    lv_obj_set_y(label2, 0);
    lv_obj_set_size(label, 120, 16);

    lv_obj_add_style(label, &main_style, 0);
    lv_obj_add_style(label2, &main_style, 0);

/*
    textarea = lv_textarea_create(lv_scr_act());
    lv_obj_set_x(textarea, 0);
    lv_obj_set_y(textarea, 16);
    lv_obj_set_size(textarea, 240, 119);
    lv_obj_set_style_text_font(textarea, &lv_font_montserrat_14, 0);
    */

    meter1 = lv_meter_create(lv_scr_act());
    lv_obj_set_size(meter1, 110, 110);
    lv_obj_set_pos(meter1, 5, 20);
    scale1 = lv_meter_add_scale(meter1);
    lv_meter_set_scale_range(meter1, scale1, 200, 300, 270, 135);
    lv_meter_set_scale_ticks(meter1, scale1, 0, 0, 0, lv_palette_main(LV_PALETTE_GREY));
    indic1 = lv_meter_add_needle_line(meter1, scale1, 4, lv_palette_main(LV_PALETTE_RED), 0);
    scale1x = lv_meter_add_scale(meter1);
    lv_meter_set_scale_ticks(meter1, scale1x, 21, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter1, scale1x, 4, 4, 15, lv_color_black(), 10);
    lv_meter_set_scale_range(meter1, scale1x, 20, 30, 270, 135);
    label_meter1 = lv_label_create(lv_scr_act());
    lv_obj_set_pos(label_meter1, 35, 105);
    lv_obj_set_size(label_meter1, 50, 20);
    lv_label_set_text(label_meter1, "");

    meter2 = lv_meter_create(lv_scr_act());
    lv_obj_set_pos(meter2, 125, 20);
    lv_obj_set_size(meter2, 110, 110);
    scale2 = lv_meter_add_scale(meter2);
    lv_meter_set_scale_ticks(meter2, scale2, 0, 0, 0, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_range(meter2, scale2, 0, 1000, 270, 135);
    indic2 = lv_meter_add_needle_line(meter2, scale2, 4, lv_palette_main(LV_PALETTE_RED), 0);
    scale2x = lv_meter_add_scale(meter2);
    lv_meter_set_scale_ticks(meter2, scale2x, 21, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter2, scale2x, 5, 4, 15, lv_color_black(), 10);
    lv_meter_set_scale_range(meter2, scale2x, 0, 100, 270, 135);
    label_meter2 = lv_label_create(lv_scr_act());
    lv_obj_set_pos(label_meter2, 155, 105);
    lv_obj_set_size(label_meter2, 50, 20);
    lv_label_set_text(label_meter2, "");

    lv_style_init(&meter_style);
    lv_style_set_bg_color(&meter_style, lv_color_make(200, 200, 200));
    lv_style_set_text_color(&meter_style, lv_color_black());
    lv_style_set_border_width(&meter_style, 3);
    lv_style_set_pad_left(&meter_style, 0);
    lv_style_set_pad_right(&meter_style, 0);
    lv_style_set_pad_top(&meter_style, 0);
    lv_style_set_pad_bottom(&meter_style, 0);
    lv_obj_add_style(meter1, &meter_style, 0);
    lv_obj_add_style(meter2, &meter_style, 0);
    lv_obj_add_style(label_meter1, &meter_style, 0);
    lv_obj_add_style(label_meter2, &meter_style, 0);

    gui_mu = xSemaphoreCreateMutex();
}

void set_label1(const char* text) {
    MutexLock lock(gui_mu);
    lv_label_set_text(label, text);
}

void set_label2(const char* text) {
    MutexLock lock(gui_mu);
    lv_label_set_text(label2, text);
}

void WifiConnectedCallback(WiFiEvent_t event, WiFiEventInfo_t info) {
    set_label2("Connected!");
}

void WifiGotIpCallback(WiFiEvent_t event, WiFiEventInfo_t info) {
    set_label2(WiFi.localIP().toString().c_str());
}

void WifiDisconnectedCallback(WiFiEvent_t event, WiFiEventInfo_t info) {
    set_label2("Disconnected!");
}

void init_wifi() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("esp32");
    WiFi.onEvent(WifiConnectedCallback, SYSTEM_EVENT_STA_CONNECTED);
    WiFi.onEvent(WifiGotIpCallback, SYSTEM_EVENT_STA_GOT_IP);
    WiFi.onEvent(WifiDisconnectedCallback, SYSTEM_EVENT_STA_DISCONNECTED);
    WiFi.begin(WIFI_SSID, WIFI_PASS); 
    set_label2("Connecting...");
}

void init_ble() {
    BLEDevice::init("esp32");
    scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallback());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(10);
}

void init_fs() {
    /*
    SPIFFS.begin();
    fs::File file = SPIFFS.open("/test.txt", "r");
    char buf[1024];
    int len = file.read((uint8_t *)buf, sizeof(buf));
    buf[len] = 0;
    tft.setCursor(0, 20);
    tft.println(buf);
    file.close();
    */
}

void init_mqtt() {
    mqtt_client.setServer(MQTT_SERVER, 1883);
}

TaskHandle_t task1, task2;

void task1_run(void* arg);
void task2_run(void* arg);

void setup() {
    Serial.begin(115200);
    init_tft();
    init_gui();
    init_wifi();
    init_mqtt();
    init_ble();

    if (xTaskCreate(task1_run, "task1", 4096, nullptr, 1, &task1) != pdPASS) {
        lv_label_set_text(label, "Fail to create task1");
    }
    if (xTaskCreate(task2_run, "task2", 4096, nullptr, 1, &task2) != pdPASS) {
        lv_label_set_text(label2, "Fail to create task2");
    }
}

void check_mqtt() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt_client.connected()) return;

    if (!mqtt_client.connect("esp32", "pubsub", "pubsub")) {
        MutexLock lock(gui_mu);
        lv_label_set_text_fmt(label2, "Fail to connect to MQTT! rc=%d", mqtt_client.state());
        return;
    }
    set_label2("MQTT OK!");
}

// std::map<std::string, std::string> ble_info;

void read_gvh(BLEAdvertisedDevice& device) {
    Serial.printf("Name: %s\n", device.getName().c_str());
    Serial.printf("Address: %s\n", device.getAddress().toString().c_str());
    Serial.printf("RSSI: %d\n", device.getRSSI());
    const std::vector<std::string>* data = device.getManufacturerData();

    float temp = 0, hum = 0;
    int bat = 0;
    bool got_data = false;
    for (const std::string& item : *data) {
        if (item.length() != 8) continue;
        uint32_t raw = (item[4] << 16) | (item[5] << 8) | item[6];
        temp = raw / 1000 / 10.0;
        hum = raw % 1000 / 10.0;
        bat = item[7];
        Serial.printf("Temp: %.1f Hum: %.1f Bat: %d%%\n", temp, hum, bat);
        got_data = true;
    }
    if (!got_data) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "%s Temp: %.1fC Hum: %.1f%% Bat: %d%%",
             device.getName().c_str(),
             temp, hum, bat);
    // ble_info[device.getName()] = buf;
    if (device.getName() == TARGET_DEVICE_NAME) {
        MutexLock lock(gui_mu);
        /*
        lv_textarea_set_text(textarea, "");
        bool first = true;
        for (const auto &entry : ble_info) {
            if (!first) lv_textarea_add_char(textarea, '\n');
            first = false;
            lv_textarea_add_text(textarea, entry.second.c_str());
        }
        */
        lv_meter_set_indicator_value(meter1, indic1, temp * 10);
        lv_label_set_text_fmt(label_meter1, "%.1fC", temp);
        lv_meter_set_indicator_value(meter2, indic2, hum * 10);
        lv_label_set_text_fmt(label_meter2, "%.1f%%", hum);
    }

    snprintf(buf, sizeof(buf), "sensor/%s", device.getName().c_str());
    buf[sizeof(buf) - 1] = 0;
    char payload[128];
    snprintf(payload, sizeof(payload), "TEMP=%.1f&HUM=%.1f&BAT=%d", temp, hum, bat);
    payload[sizeof(payload) - 1] = 0;
    mqtt_client.publish(buf, payload);
/*
    BLEClient *client = BLEDevice::createClient();
    if (!client->connect(&ble_device)) {
        lv_textarea_set_text(label, "Fail to connect!");
    }
    Serial.println("Connected to device");

    //std::map<std::string, BLERemoteService*>* services = client->getServices();
    //for (const auto& entry : *services) {
    BLERemoteService* service = client->getService(SERVICE_UUID);
    if (service == nullptr) {
        Serial.println("Fail to get service");
    } else {
        std::map<uint16_t, BLERemoteCharacteristic *> chars;
        service->getCharacteristics(&chars);
        for (const auto &entry : chars)
        {
            Serial.printf("Char %x UUID: %s Value: ", entry.first, entry.second->getUUID().toString().c_str());
            if (entry.second->canRead()) {
                std::string value = entry.second->readValue();
                for (int i = 0; i < value.length(); ++i) {
                    Serial.printf("%02x ", value[i]);
                }
            } else {
                Serial.print("N/A");
            }
            Serial.println();
        }
    }
    Serial.println("Disconnecting");
    client->disconnect();
    Serial.println("Deleting client");
    delete client;
    */
}

int loop_counter = 0;

void loop() {
    {
        MutexLock lock(gui_mu);
        lv_timer_handler();
    }
    delay(100);
}

void task1_run(void* arg) {
    while (true) {
        ++loop_counter;
        ble_found = false;
        {
            MutexLock lock(gui_mu);
            lv_label_set_text_fmt(label, "Scanning, #%d", loop_counter);
        }
        scan->start(/*duration=*/60, /*is_continue=*/false);
        scan->stop();

        if (ble_found) {
            set_label1("Found");
        } else {
            set_label1("Not found");
        }
        scan->clearResults();
        delay(1000);
    }
}

void task2_run(void* arg) {
    while (true) {
        check_mqtt();
        mqtt_client.loop();
        mqtt_client.publish("rssi", String(WiFi.RSSI()).c_str());
        delay(1000);
    }
}
