/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include <driver/dac.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#include <driver/gpio.h>
#include "soc/syscon_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"


static const char* TAG = "ADC";

#define I2S_NUM         (0)
#define PI 3.14159265

static const i2s_config_t i2s_config = {
     .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX | I2S_MODE_DAC_BUILT_IN,
     .sample_rate = 4000,
     .bits_per_sample = 8, /* must be 8 for built-in DAC */
     .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
     .communication_format = I2S_COMM_FORMAT_I2S_MSB ,
     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
     .dma_buf_count = 2,
     .dma_buf_len = 128
};

void dac_init() {
    char samples[256];
    float sin_float;
    // Setup the i2s dma
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);   //install and start i2s driver
    // Connect dac to i2s
    i2s_set_pin(I2S_NUM, NULL); //for internal DAC

    // Create Data
    // Make sure data is between 0 and 1V
    for(uint i = 0; i < 256; i++) {
        sin_float = sin(i / 128.0 * 2 * PI);
        sin_float *= 35;
        sin_float += 36;

        samples[i] = (char) sin_float;
    }
    // Write data to the buffer. This is timing dependant, so be carefull with 
    // changing the configuration
    i2s_write_bytes(I2S_NUM, (char*) &samples, 256, portMAX_DELAY);
    i2s_write_bytes(I2S_NUM, (char*) &samples, 256, portMAX_DELAY);
 
    ESP_LOGI(TAG, "Done initialising DAC");
 
    // Disable both DAC and ADC
    i2s_stop(I2S_NUM);

}

void adc_measure() {
    // Restart the DAC
    i2s_start(I2S_NUM);
    ESP_LOGI(TAG, "Started DAC");

    // Setup ADC
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_INPUT);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_0db);
    ESP_LOGI(TAG, "Set up ADC");

    //Adc Controler is Rtc module,not ulp coprocessor
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 1, SENS_MEAS1_START_FORCE_S); //force pad mux and force start
    //Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 0, SENS_FORCE_XPD_SAR_S); //force XPD_SAR=0, use XPD_FSM
    //Disable Amp Bit1=0:Fsm  Bit1=1(Bit0=0:PownDown Bit10=1:Powerup)
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_AMP, 0x2, SENS_FORCE_XPD_AMP_S); //force XPD_AMP=0
    //Open the ADC1 Data port Not ulp coprocessor
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 1, SENS_SAR1_EN_PAD_FORCE_S); //open the ADC1 data port
    //Select channel
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << ADC1_CHANNEL_5), SENS_SAR1_EN_PAD_S); //pad enable
    SET_PERI_REG_BITS(SENS_SAR_MEAS_CTRL_REG, 0xfff, 0x0, SENS_AMP_RST_FB_FSM_S);  //[11:8]:short ref ground, [7:4]:short ref, [3:0]:rst fb
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT1, 0x1, SENS_SAR_AMP_WAIT1_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT1_REG, SENS_SAR_AMP_WAIT2, 0x1, SENS_SAR_AMP_WAIT2_S);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_SAR_AMP_WAIT3, 0x1, SENS_SAR_AMP_WAIT3_S);
    while (GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR1_REG, 0x7, SENS_MEAS_STATUS_S) != 0); //wait det_fsm==0

    int adc_values[512];
    for (uint i=0; i < 512; i++) {
        // Trigger ADC
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 0, SENS_MEAS1_START_SAR_S); //start force 0
        SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, 1, 1, SENS_MEAS1_START_SAR_S); //start force 1
        // Wait for adc to finish
        while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0) {}; //read done
        // Ready out sample
        adc_values[i] = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
    }

    char msg[6];
    for (uint i=0; i < 512; i++) {
        msg[0] = 0x55;
        msg[1] = 0xAA;
        msg[2] = 0x03;    
        msg[3] = (char) (adc_values[i] >> 8);
        msg[4] = (char) (adc_values[i]);
        msg[5] = msg[3] + msg[4] + 0xff + 0x03;
        fwrite(msg, 1, sizeof(msg), stdout);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Done ADC Measurement");

    // Halt the dac
    i2s_stop(I2S_NUM);

}

void adc1task(void* arg)
{
    dac_init();
    while(1){
        adc_measure();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void app_main()
{
    // 12 bit version of the adc introduces the bug
    // adc1_config_width(ADC_WIDTH_12Bit);
    // Uncomment to see difference in behaviour with 12 bit version
    adc1_config_width(ADC_WIDTH_11Bit);

    xTaskCreate(adc1task, "adc1task", 1024*3, NULL, 10, NULL);
}

