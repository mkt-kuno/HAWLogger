#ifndef __HAWLOGGER_UTILS_H__
#define __HAWLOGGER_UTILS_H__

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <HX711.h>

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

class Loadcell : Calibration
{
public:
    Loadcell(uint8_t sck = -1, uint8_t dout = -1, double calib_a = 1.0, double calib_b = 0.0)
    {
        this->sck = sck;
        this->dout = dout;
        this->setCalibA(calib_a);
        this->setCalibB(calib_b);
    };
    void init(void)
    {
        this->hx711.begin(this->dout, this->sck);
    };
    
    int32_t read(void){
        int32_t ret = 0;
        // Data Readyになるまで10msずつ待つ
        this->hx711.wait_ready(10);
        return this->hx711.read();
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
    HX711 hx711;
    uint8_t sck;
    uint8_t dout;
    int32_t raw_value;
    float phy_value;
};

static void Loadcell_Task(void *pvParameters)
{
    Loadcell *lc = (Loadcell *)pvParameters;
    lc->init();
    for (;;) { lc->task(); }
}

#endif /*__HAWLOGGER_UTILS_H__*/