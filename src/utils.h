#ifndef __HAWLOGGER_UTILS_H__
#define __HAWLOGGER_UTILS_H__

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

class Calibration
{
private:
    double calib_a = 1.0;
    double calib_b = 0.0;

public:
    void setCalibA(double value)
    {
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        this->calib_a = value;
        portEXIT_CRITICAL(&mux);
    }
    void setCalibB(double value)
    {
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        this->calib_b = value;
        portEXIT_CRITICAL(&mux);
    }
    double getCalibA(void)
    {
        double ret;
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        ret = this->calib_a;
        portEXIT_CRITICAL(&mux);
        return ret;
    }
    double getCalibB(void)
    {
        double ret;
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        ret = this->calib_b;
        portEXIT_CRITICAL(&mux);
        return ret;
    }
protected:
    float calc(float value)
    {
        float ret;
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        ret = (float)(this->calib_a * (double)value + this->calib_b);
        portEXIT_CRITICAL(&mux);
        return ret;
    }
};

class HX711 : Calibration
{
public:
    HX711(uint8_t sck = -1, uint8_t dout = -1, double calib_a = 1.0, double calib_b = 0.0)
    {
        this->sck = sck;
        this->dout = dout;
        this->setCalibA(calib_a);
        this->setCalibB(calib_b);
    };
    void init(void)
    {
        pinMode(this->sck, OUTPUT);
        pinMode(this->dout, INPUT);
        digitalWrite(this->sck, 1);
        delayMicroseconds(60);
        digitalWrite(this->sck, 0);
    }

    int32_t read(void){
        int32_t ret = 0;
        // DOUT->LOW　つまり Data Readyになるまで待つ
        while (!digitalRead(this->dout) == LOW)
        {
            // まだだった場合10ms程度待つ
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        // パルスの作成中にRTOSに処理持ってかれたらマズイのでRTOSを止める
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        {
            // 1ビットずつ読み込む
            for (char i = 0; i < 24; i++)
            {
                this->pulse();
                ret = (ret << 1) | (digitalRead(this->dout));
            }
            // 次回変換先＆ゲイン指定、詳しくはデータシート
            this->pulse(); // one pulse -> channel A amp x128
        }
        portEXIT_CRITICAL(&mux);
        // このままだと負の値、最上位ビットが24ビット目にあるが、
        // 32ビットに格納する必要があるので、左に8ビットシフトする
        return (int32_t)(ret << 8);
    }

    void task(void) {
        // 86ms間隔なので 60msくらい待つ
        vTaskDelay(60 / portTICK_PERIOD_MS);
        // 頃合いになったら読み込む
        int32_t itemp = this->read();
        float dtemp = this->calc(itemp);

        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        {
            // タスク間で共有している値なので、RTOSを止めて読み書きする
            this->raw_value = itemp;
            this->phy_value = dtemp;
        }
        portEXIT_CRITICAL(&mux);
    }

    void getRawPhyValue(int32_t *ivalue, float *fvalue) {
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        *ivalue = this->raw_value;
        *fvalue = this->phy_value;
        portEXIT_CRITICAL(&mux);
    }

    int32_t getRawValue(void)
    {
        int32_t ret;
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        ret = this->raw_value;
        portEXIT_CRITICAL(&mux);
        return ret;
    }
    float getPysicalValue(void)
    {
        float ret;
        portMUX_TYPE mux = SPINLOCK_INITIALIZER;
        portENTER_CRITICAL(&mux);
        ret = this->phy_value;
        portEXIT_CRITICAL(&mux);
        return ret;
    }
    void task(void *pvParameters)
    {
        this->init();
        for (;;) { this->task(); }
    }

private:
    uint8_t sck;
    uint8_t dout;
    int32_t raw_value;
    float phy_value;
    
    void pulse(void)
    {
        digitalWrite(this->sck, 1);
        delayMicroseconds(1);
        digitalWrite(this->sck, 0);
        delayMicroseconds(1);
    }
};

static void HX711_Task(void *pvParameters)
{
    HX711 *phx711 = (HX711 *)pvParameters;
    phx711->init();
    for (;;) { phx711->task(); }
}

#endif /*__HAWLOGGER_UTILS_H__*/