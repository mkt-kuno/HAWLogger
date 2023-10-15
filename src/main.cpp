////////////////////////////////////
//* Project for Arduino Mega2560 *//
////////////////////////////////////
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <ADS1115_WE.h>

#include "utils.h"

#define DISPLAY_OLED
#ifdef DISPLAY_OLED
#include "oled1602.h"
OLED1602 lcd = OLED1602();
#else
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#endif

const char *TAG = "MAIN";

#define FIRMWARE_VERSION_STR "HAW ver0.1"
#define CSV_NAMING_RULE "/haw_%02d.CSV"
#define CSV_HEADER "time[s],hx711a[i24],hx711b[i24],hx711c[i24],disp[V],hx711a[N],hx711b[N]],hx711c[N],disp[mm]"

SPIClass sdspi(VSPI);
ADS1115_WE adc = ADS1115_WE();

#define ESP32_SLOW_CLOCK 80
#define ESP32_FAST_CLOCK 240

//////////////// Calibration Value //////////////////
#define NUM_HX711_CH 4

Loadcell st_hx711[] = {
    Loadcell(32, 36, 0.001, 0.0),
    Loadcell(33, 39, 0.001, 0.0),
    Loadcell(25, 34, 0.001, 0.0),
    Loadcell(26, 35, 0.001, 0.0),
};

#define CALIB_DISP_AX (20.884)
#define CALIB_DISP_B (0.000)

void MainTask(void *pvParameters);

// the setup function runs once when you press reset or power the board
void setup(void)
{
    setCpuFrequencyMhz(ESP32_SLOW_CLOCK);

    Serial.begin(115200);
    while (!Serial) {}
    Serial.setDebugOutput(true);
    ESP_LOGI(TAG, "setup, UART start");

    Wire.begin();
    Wire.setClock(400E3);

    xTaskCreate(MainTask, "Main", configMINIMAL_STACK_SIZE + 2048, NULL, 1, NULL);
    xTaskCreate(Loadcell_Task, "hx711[0]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[0], 2, NULL);
    xTaskCreate(Loadcell_Task, "hx711[1]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[1], 2, NULL);
    xTaskCreate(Loadcell_Task, "hx711[2]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[2], 2, NULL);
    xTaskCreate(Loadcell_Task, "hx711[3]", configMINIMAL_STACK_SIZE + 128, (void *)&st_hx711[3], 2, NULL);
}

void loop()
{
    // Empty. Things are done in Tasks.
}

void sd_make_filename(char *fn_str, int fn_str_len)
{
    int8_t ret = -1;
    char str_buf[16];
    for (int8_t index = 0; index < 100; index++)
    {
        sprintf(str_buf, CSV_NAMING_RULE, index);
        if (SD.exists(str_buf))
            ret = index;
    }
    memset(fn_str, 0x00, fn_str_len);
    sprintf(fn_str, CSV_NAMING_RULE, ret + 1);
}

void MainTask(void *pvParameters) // This is a task.
{
    (void)pvParameters;
    const uint8_t BTN_INPUT = 27;
    uint8_t btn_store = 0x00;
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

#ifdef DISPLAY_OLED
    lcd.begin();
    lcd.setContrast(0x02);
    lcd.lcdOn();
#else
    lcd.begin(20, 4);
    lcd.backlight();
#endif

    if (!adc.init())
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ADmodule ADS1115");
        lcd.setCursor(0, 1);
        lcd.print("not connected!!!");
        for (;;)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        };
    }
    adc.setVoltageRange_mV(ADS1115_RANGE_4096);
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    adc.setConvRate(ADS1115_128_SPS);

    if (!SD.begin(5, sdspi))
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SD CARD ERROR!!!");
        lcd.setCursor(0, 1);
        lcd.print("CheckSD & REBOOT");
        for (;;)
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        };
    }

    pinMode(BTN_INPUT, INPUT_PULLUP);

    for (;;) // A Task shall never return or exit.
    {
        // HX711データ取得
        for (int i = 0; i < NUM_HX711_CH ;i++) {
            st_hx711[i].getRawPhyValue(&l_raw_hx711[i], &l_phy_hx711[i]);
        }
        
        // 変位計ADCデータ取得
        adc.setCompareChannels(ADS1115_COMP_0_GND);
        local_disp = adc.getResult_V();
        phy_displace = local_disp * CALIB_DISP_AX + CALIB_DISP_B;

        // ボタン短押し・長押し処理
        btn_store = btn_store << 1 | (!digitalRead(BTN_INPUT) & 0x01);
        if ((btn_store & 0b00000111) == 0b00000010 || (btn_store & 0b00001111) == 0b00000110)
        {
            window_num++;
            if (window_num > 2)
            {
                window_num = 0;
            }
        }
        if ((btn_store & 0b00111111) == 0b00111110)
        {
            // Start or Stop Recording
            if (!is_recording)
            {
                setCpuFrequencyMhz(ESP32_FAST_CLOCK);

                sd_make_filename(sd_fn, sd_fn_len);
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

                is_recording = false;
                memset(sd_fn, 0x00, sd_fn_len + 1);
            }
        }

        // 液晶表示処理
        switch (window_num)
        {
        default:
        case 0:
            lcd.setCursor(0, 0);
            if (!is_recording)
            {
                lcd.print("NOREC ");
                lcd.print(FIRMWARE_VERSION_STR);
            }
            else
            {
                lcd.print("REC");
                static uint8_t progress = 0;
                switch (progress)
                {
                default:
                    lcd.print("*");
                    break;
                case 0:
                    lcd.print("|");
                    break;
                case 1:
                    lcd.print("/");
                    break;
                case 2:
                    lcd.print("-");
                    break;
                case 3:
                    lcd.print("\\");
                    break;
                }
                progress++;
                if (progress > 3)
                    progress = 0;
                lcd.print(" ");
                lcd.print(sd_fn);
            }
            lcd.setCursor(0, 1); lcd.print(dtostrf((float)xTaskGetTickCount() / configTICK_RATE_HZ, 6, 1, dtostrf_buf)); lcd.print("s");
            lcd.setCursor(8, 1); lcd.print(dtostrf(phy_displace, 6, 2, dtostrf_buf)); lcd.print("mm");
            break;
        case 1:
            lcd.setCursor(0, 0); lcd.print("hxA:"); lcd.print(dtostrf(l_phy_hx711[0], 9, 1, dtostrf_buf)); lcd.print("[N]");
            lcd.setCursor(0, 1); lcd.print("hxB:"); lcd.print(dtostrf(l_phy_hx711[1], 9, 1, dtostrf_buf)); lcd.print("[N]");
            break;
        case 2:
            lcd.setCursor(0, 0); lcd.print("hxC:"); lcd.print(dtostrf(l_phy_hx711[2], 9, 1, dtostrf_buf)); lcd.print("[N]");
            lcd.setCursor(0, 1); lcd.print("hxD:"); lcd.print(dtostrf(l_phy_hx711[3], 9, 1, dtostrf_buf)); lcd.print("[N]");
            break;
        }

        if (true) {
            ESP_LOGI(TAG, "{disp:%f, hxA:0x%08lx, hxB:0x%08lx, hxC:0x%08lx, hxD:0x%08lx}",  local_disp, l_raw_hx711[0], l_raw_hx711[1],l_raw_hx711[2],l_raw_hx711[3]);
        }

        // SDカード保存処理
        if (is_recording)
        {
            csvFile = SD.open(sd_fn, FILE_APPEND);
            if (csvFile)
            {
                int p = 0;
                memset(sd_buf, 0x00, sd_buf_size + 1);
                dtostrf((float)xTaskGetTickCount() / configTICK_RATE_HZ, -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                p += sprintf(&sd_buf[p], "0x%08lX", l_raw_hx711[0]) - 2; sd_buf[p++] = ',';
                p += sprintf(&sd_buf[p], "0x%08lX", l_raw_hx711[1]) - 2; sd_buf[p++] = ',';
                p += sprintf(&sd_buf[p], "0x%08lX", l_raw_hx711[2]) - 2; sd_buf[p++] = ',';
                p += sprintf(&sd_buf[p], "0x%08lX", l_raw_hx711[3]) - 2; sd_buf[p++] = ',';
                dtostrf(phy_displace, -1, 3, dtostrf_buf);   p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(l_phy_hx711[0], -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(l_phy_hx711[1], -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(l_phy_hx711[2], -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(l_phy_hx711[3], -1, 3, dtostrf_buf); p += sprintf(&sd_buf[p], "%s", dtostrf_buf); sd_buf[p++] = ',';
                dtostrf(phy_displace, -1, 3, dtostrf_buf);   p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
                csvFile.println(sd_buf);
                csvFile.close();
            }
        }
        vTaskDelay(48 / portTICK_PERIOD_MS);
    }
}
