////////////////////////////////////
//* Project for Arduino ESP32    *//
////////////////////////////////////
#include "utils.h"

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <FS.h>

#include <freertos/FreeRTOS.h>
#include <esp_wifi.h>

#include <math.h>

#include <ESPFMfGK.h>
#include <ADS1115_WE.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

const char *TAG = "MAIN";
const word fmng_port = 8080;
bool is_fmng_enabled = false;

#define SD_SPI_CS               (5)
#define BTN_INPUT               (27)
#define DISPLAY_OLED
#define FIRMWARE_VERSION_STR    "HAW version 2.00"
#define CSV_NAMING_RULE         "/haw_%02d.CSV"
#define CSV_HEADER              "time[s],hx711a[i24],hx711b[i24],disp[V],hx711a[N],hx711b[N],disp[mm]"
#define WIFI_AP_MAX_CLIENTS     (4)
#define NUM_HX711_CH            (2)
#define ESP32_SLOW_CLOCK        (80)
#define ESP32_FAST_CLOCK        (240)

SPIClass sdspi(VSPI);
ADS1115_WE adc = ADS1115_WE();
ESPFMfGK fmng(fmng_port);
Loadcell st_hx711[] = { Loadcell(32, 36, 1.0, 0.0), Loadcell(33, 39, 1.0, 0.0),};
#ifdef DISPLAY_OLED
#include "oled1602.h"
OLED1602 lcd = OLED1602();
#else
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#endif
typedef enum {
    BUTTON_NONE = 0,
    BUTTON_SHORT,
    BUTTON_LONG,
} BUTTON_EVENT_T;

//////////////// Calibration Value //////////////////
static double CALIB_HX711A_AX = 0.00022828; // Horizontal
static double CALIB_HX711A_B  = 2.47313;
static double CALIB_HX711B_AX = 0.00042649; // Vertical
static double CALIB_HX711B_B  = 2118.3;
static double CALIB_DISP_AX = 31.15;
static double CALIB_DISP_B  = 0.000;

void logger_main_task(void *pvParameters);
void logger_button_task(void *pvParameters);
void logger_setup(void);
void fmng_setup(void);
void fmng_loop(void);
static inline float get_float_time_sec(void) { return (float)xTaskGetTickCount() / configTICK_RATE_HZ; }

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    Serial.setDebugOutput(true);

    LittleFS.begin();
    ESP_LOGI(TAG, "LittleFS started");

    Wire.begin();
    Wire.setClock(400E3);

#ifdef DISPLAY_OLED
    lcd.begin();
    lcd.setContrast(0x02);
    lcd.lcdOn();
#else
    lcd.begin(20, 4);
    lcd.backlight();
#endif

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("To Enter FM Mode");
    lcd.setCursor(0, 1);
    lcd.print("LongPress Button");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    int is_logger_mode = 10;
    pinMode(BTN_INPUT, INPUT);
    for (uint8_t timeout = 0; timeout < 10; timeout++)
    {
        if (digitalRead(BTN_INPUT) == HIGH) {
            break;
        }
        ESP_LOGI(TAG, "Waiting button... %d", is_logger_mode);
        is_logger_mode--;
        delay(100);
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    
    if (is_logger_mode > 0) {
        ESP_LOGI(TAG, "Enter Logger Mode");
        logger_setup();
    } else {
        ESP_LOGI(TAG, "Enter FileManager Mode");
        is_fmng_enabled = true;
        fmng_setup();
    }
}

void loop()
{
    fmng_loop();
}

void fmng_loop()
{
    if (!is_fmng_enabled) return;
    
    fmng.handleClient();

    // toggle lcd per 2 seconds
    static float last_time = 0;
    if (get_float_time_sec() - last_time > 2) {
        last_time = get_float_time_sec();
        static bool toggle = false;
        toggle = !toggle;
        if (toggle) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("192.168.4.1:8080");
            lcd.setCursor(0, 1);
            lcd.print("FileManager Mode");
        } else {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("SSID: HAWLogger ");
            lcd.setCursor(0, 1);
            lcd.print("PASS: 0123456789");
        }
    }
}

void fmng_add_fs(void) {
    // Add SD File System
    if (SD.begin(SD_SPI_CS, sdspi)) {
        if (!fmng.AddFS(SD, "SD Card", false)) {
            ESP_LOGI(TAG, "Adding SD failed.");
        }
        ESP_LOGI(TAG, "Adding SD success.");
    } else {
        ESP_LOGE(TAG, "SD File System not inited.");
    }
    // Add LittleFS
    if (fmng.AddFS(LittleFS, "LittleFS", false)) {
        ESP_LOGI(TAG, "Adding LittleFS success.");
    } else {
        ESP_LOGI(TAG, "Adding LittleFS failed.");
    }
}

uint32_t fmng_check_fflags(fs::FS &fs, String filename, uint32_t flags) {
    // Checks if target file name is valid for action. This will simply allow everything by returning the queried flag
    if (flags & ESPFMfGK::flagIsValidAction) { return flags & (~ESPFMfGK::flagIsValidAction);  }
    // Checks if target file name is valid for action.
    if (flags & ESPFMfGK::flagIsValidTargetFilename) { return flags & (~ESPFMfGK::flagIsValidTargetFilename);}
    // Default actions
    int32_t defaultflags = ESPFMfGK::flagCanDelete | ESPFMfGK::flagCanRename | ESPFMfGK::flagCanGZip |
                           ESPFMfGK::flagCanDownload | ESPFMfGK::flagCanUpload | ESPFMfGK::flagCanEdit |
                           ESPFMfGK::flagAllowPreview;
    return defaultflags;
}

void fmng_initialize(void) {
    // See above.
    fmng.checkFileFlags = fmng_check_fflags;
    fmng.WebPageTitle = "FileManager";
    fmng.BackgroundColor = "white";
    fmng.textareaCharset = "accept-charset=\"utf-8\"";

    if (fmng.begin()) {
        ESP_LOGI(TAG, "Open Filemanager with http://192.168.4.1:%d/", fmng_port);
    } else {
        ESP_LOGI(TAG, "Filemanager: did not start");
    }
}

void fmng_setup() {
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP("HAWLogger", "0123456789", 1, 0, WIFI_AP_MAX_CLIENTS);
    WiFi.disconnect(true);
    
    esp_wifi_stop();
    esp_wifi_deinit();
    wifi_init_config_t ap_config = WIFI_INIT_CONFIG_DEFAULT();
    ap_config.ampdu_rx_enable = 0;
    esp_wifi_init(&ap_config);
    esp_wifi_start();
    
    IPAddress ip(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress dns(192, 168, 4, 1);
    WiFi.softAPConfig(ip, gateway, subnet); 

    fmng_add_fs();
    fmng_initialize();
}

void logger_setup(void)
{
    esp_wifi_stop();
    esp_wifi_deinit();

    setCpuFrequencyMhz(ESP32_SLOW_CLOCK);
    Serial.updateBaudRate(115200);

    ESP_LOGI(TAG, "enter logger_setup()");

    QueueHandle_t xButtonQueue = xQueueCreate(1, sizeof(BUTTON_EVENT_T));
    xTaskCreate(logger_main_task, "Main", configMINIMAL_STACK_SIZE + 2048, (void *)xButtonQueue, 1, NULL);
    xTaskCreate(logger_button_task, "Button", configMINIMAL_STACK_SIZE + 32, (void *)xButtonQueue, 1, NULL);
    xTaskCreate(loadcell_task, "hx711[0]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[0], 2, NULL);
    xTaskCreate(loadcell_task, "hx711[1]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[1], 2, NULL);
    // xTaskCreate(loadcell_task, "hx711[2]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[2], 2, NULL);
    // xTaskCreate(loadcell_task, "hx711[3]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[3], 2, NULL);
}

void logger_sd_make_filename(char *fn_str, int fn_str_len)
{
    int8_t ret = -1;
    char str_buf[16];
    for (int8_t index = 0; index < 100; index++)
    {
        sprintf(str_buf, CSV_NAMING_RULE, index);
        if (SD.exists(str_buf)) ret = index;
    }
    memset(fn_str, 0x00, fn_str_len);
    sprintf(fn_str, CSV_NAMING_RULE, ret + 1);
}

void logger_button_task(void *pvParameters) {
    QueueHandle_t xButtonQueue = (QueueHandle_t)pvParameters;
    uint8_t btn_store = 0x00;
    pinMode(BTN_INPUT, INPUT);
    for(;;) {
        btn_store = btn_store << 1 | (!digitalRead(BTN_INPUT) & 0x01);
        if ((btn_store & 0b00000111) == 0b00000010 || (btn_store & 0b00001111) == 0b00000110)
        {
            BUTTON_EVENT_T val = BUTTON_SHORT;
            xQueueSend(xButtonQueue, &val, 100/portTICK_PERIOD_MS);
        }
        if ((btn_store & 0b00111111) == 0b00111110)
        {
            BUTTON_EVENT_T val = BUTTON_LONG;
            xQueueSend(xButtonQueue, &val, 100/portTICK_PERIOD_MS);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void logger_main_task(void *pvParameters) // This is a task.
{
    QueueHandle_t xButtonQueue = (QueueHandle_t)pvParameters;
    
    volatile uint8_t window_num = 0;

    int32_t l_raw_hx711[NUM_HX711_CH] = {};
    float l_phy_hx711[NUM_HX711_CH] = {};
    float local_disp = 0;
    float phy_displace = 0;

    static File csvFile;

    bool is_recording = false;

    // Naming rule is CSV_NAMING_RULE
    const int sd_fn_len = 16;
    char sd_fn[sd_fn_len + 1];
    memset(sd_fn, 0x00, sd_fn_len + 1);

    static char dtostrf_buf[17];
    const int sd_buf_size = 256;
    static char sd_buf[sd_buf_size + 1];

    // Check ADmodule ADS1115
    if (!adc.init())
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ADmodule ADS1115");
        lcd.setCursor(0, 1);
        lcd.print("not connected!!!");
        vTaskSuspend(NULL);
    }
    adc.setVoltageRange_mV(ADS1115_RANGE_4096);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    adc.setConvRate(ADS1115_128_SPS);

    // Check SD Card
    if (!SD.begin(SD_SPI_CS, sdspi))
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SD CARD ERROR!!!");
        lcd.setCursor(0, 1);
        lcd.print("CheckSD & REBOOT");
        vTaskSuspend(NULL);
    }

    // Check config.json file in SD Card
    File configFile1 = SD.open("/config.json");
    if (!configFile1)
    {
        // create template json file
        configFile1 = SD.open("/config.json", FILE_WRITE);
        if (configFile1)
        {
            StaticJsonDocument<256> doc;
            doc["CALIB_HX711A_AX"] = CALIB_HX711A_AX;
            doc["CALIB_HX711A_B"] = CALIB_HX711A_B;
            doc["CALIB_HX711B_AX"] = CALIB_HX711B_AX;
            doc["CALIB_HX711B_B"] = CALIB_HX711B_B;
            doc["CALIB_DISP_AX"] = CALIB_DISP_AX;
            doc["CALIB_DISP_B"] = CALIB_DISP_B;
            serializeJson(doc, configFile1);
            configFile1.close();
        }
    }

    // Read calibration value from config.json
    File configFile2 = SD.open("/config.json");
    if (configFile2){
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, configFile2);
        if (error) {
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("config.json ERROR");
            lcd.setCursor(0, 1);
            lcd.print("CheckSD & REBOOT");
            vTaskSuspend(NULL);
        }
        CALIB_HX711A_AX = doc["CALIB_HX711A_AX"].as<double>();
        CALIB_HX711A_B  = doc["CALIB_HX711A_B"].as<double>();
        CALIB_HX711B_AX = doc["CALIB_HX711B_AX"].as<double>();
        CALIB_HX711B_B  = doc["CALIB_HX711B_B"].as<double>();
        CALIB_DISP_AX   = doc["CALIB_DISP_AX"].as<double>();
        CALIB_DISP_B    = doc["CALIB_DISP_B"].as<double>();
        configFile2.close();
    }

    // Set Calibration Value to HX711
    st_hx711[0].setCalibA(CALIB_HX711A_AX);
    st_hx711[0].setCalibB(CALIB_HX711A_B);
    st_hx711[1].setCalibA(CALIB_HX711B_AX);
    st_hx711[1].setCalibB(CALIB_HX711B_B);

    for (;;) // A Task shall never return or exit.
    {
        // HX711データ取得
        for (int i = 0; i < NUM_HX711_CH; i++) {
            st_hx711[i].getRawPhyValue(&l_raw_hx711[i], &l_phy_hx711[i]);
        }
        
        // 変位計ADCデータ取得
        adc.setCompareChannels(ADS1115_COMP_0_GND);
        local_disp = adc.getResult_V();
        phy_displace = local_disp * CALIB_DISP_AX + CALIB_DISP_B;

        // ボタン短押し・長押し処理
        BUTTON_EVENT_T event = BUTTON_NONE;
        xQueueReceive(xButtonQueue, &event, 0);
        if (event == BUTTON_SHORT) {
            if (++window_num > 1) window_num = 0;
        }
        if (event == BUTTON_LONG) {
            // Start or Stop Recording
            if (!is_recording)
            {
                setCpuFrequencyMhz(ESP32_FAST_CLOCK);
                Serial.updateBaudRate(115200);

                logger_sd_make_filename(sd_fn, sd_fn_len);
                // ヘッダーを書き込む
                csvFile = SD.open(sd_fn, FILE_WRITE, true);
                if (csvFile)
                {
                    csvFile.println(CSV_HEADER); // write contents
                    csvFile.close();
                }
                is_recording = true;
            }
            else
            {
                setCpuFrequencyMhz(ESP32_SLOW_CLOCK);
                Serial.updateBaudRate(115200);

                is_recording = false;
                memset(sd_fn, 0x00, sd_fn_len + 1);
            }
        }

        // 液晶表示処理
        switch (window_num) {
            default:
            case 0:
                lcd.setCursor(0, 0);
                lcd.print(FIRMWARE_VERSION_STR);lcd.print("  "); 
                if (is_recording) {
                    lcd.setCursor(0, 1);
                    lcd.print("REC");
                    static uint8_t progress = 0;
                    switch (progress) {
                        default: lcd.print("*");  break;
                        case 0:  lcd.print("|");  break;
                        case 1:  lcd.print("/");  break;
                        case 2:  lcd.print("-");  break;
                        case 3:  lcd.print("\\"); break;
                    }
                    if (++progress > 3) progress = 0;
                    lcd.print(" "); lcd.print(sd_fn);
                }
                else lcd.setCursor(0, 1); lcd.print("Data Logger Mode");
                break;
            case 1:
                lcd.setCursor(0, 0); lcd.print(dtostrf(get_float_time_sec(), 7, 1, dtostrf_buf)); lcd.print("s");
                lcd.setCursor(8, 0); lcd.print(dtostrf(phy_displace, 6, 1, dtostrf_buf)); lcd.print("mm");
                lcd.setCursor(0, 1); lcd.print("A"); lcd.print(dtostrf(l_phy_hx711[0], 7, 1, dtostrf_buf));
                lcd.setCursor(8, 1); lcd.print("B"); lcd.print(dtostrf(l_phy_hx711[1], 7, 1, dtostrf_buf));
                break;
        }

        if (true) {
            ESP_LOGI(TAG, "{time:%f, disp:%f, hxA:0x%08lx, hxB:0x%08lx, phd:%f, pha:%f, phb:%f}", get_float_time_sec(), local_disp, l_raw_hx711[0], l_raw_hx711[1], phy_displace, l_phy_hx711[0], l_phy_hx711[1]);
        }

        // SDカード保存処理
        if (is_recording) {
            csvFile = SD.open(sd_fn, FILE_APPEND);
            if (csvFile)
            {
                int p = 0;
                memset(sd_buf, 0x00, sd_buf_size + 1);
                dtostrf(get_float_time_sec(), -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                p += sprintf(&sd_buf[p], "0x%08lX", l_raw_hx711[0]) - 2; sd_buf[p++] = ',';
                p += sprintf(&sd_buf[p], "0x%08lX", l_raw_hx711[1]) - 2; sd_buf[p++] = ',';
                dtostrf(phy_displace, -1, 3, dtostrf_buf);   p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(l_phy_hx711[0], -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(l_phy_hx711[1], -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(phy_displace, -1, 3, dtostrf_buf);   p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
                csvFile.println(sd_buf);
                csvFile.close();
            }
        }
        vTaskDelay(48 / portTICK_PERIOD_MS);
    }
}
