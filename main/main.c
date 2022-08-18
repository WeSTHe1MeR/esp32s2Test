/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include <math.h>
#include "esp_dsp.h"
#include "fft.h"
#include "driver/i2s.h"
#include "esp_err.h"

#define AMPLITUDE      2000000          // Depending on your audio source level, you may need to alter this value. Can be used as a 'sensitivity' control.
#define NUM_BANDS       8            // To change this, you will need to change the bunch of if statements describing the mapping from bins to bands
#define NOISE          900000           // Used as a crude noise filter, values below this are ignored
//-----------------------------
#define TIMES              1024
#define GET_UNIT(x)        ((x>>3) & 0x1)
#define ADC_RESULT_BYTE     2
#define ADC_CONV_LIMIT_EN   0
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1

static uint16_t result[TIMES] = {0};
static const char *TAG = "ADC DMA";

#define CONFIG_EXAMPLE_SAMPLE_RATE 40000
#define CONFIG_EXAMPLE_I2S_CH 0
void init_microphone(void)
{
    // Set the I2S configuration as PDM and 16bits per sample
i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_RX,
    .sample_rate = CONFIG_EXAMPLE_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
    .dma_buf_count = 10,
    .dma_buf_len = 1024,
    .use_apll = true,
    .intr_alloc_flags = 0  // Interrupt level 1, default 0
    };

    // Set the pinout configuration (set using menuconfig)
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = I2S_PIN_NO_CHANGE,
        .ws_io_num = I2S_PIN_NO_CHANGE,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = GPIO_NUM_1,
    };

    // Call driver installation function before any I2S R/W operation.
    ESP_ERROR_CHECK( i2s_driver_install(CONFIG_EXAMPLE_I2S_CH, &i2s_config, 0, NULL) );
    ESP_ERROR_CHECK( i2s_set_pin(CONFIG_EXAMPLE_I2S_CH, &pin_config) );
     //ESP_ERROR_CHECK( i2s_set_clk(CONFIG_EXAMPLE_I2S_CH, CONFIG_EXAMPLE_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO) );
}

void app_main(void)
{
    int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    int oldBarHeights[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    esp_err_t ret=0;
    uint32_t ret_num = 0;
    uint32_t ret_num2 = 0;
    
    init_microphone();
    while(1) {
        ret = i2s_read(CONFIG_EXAMPLE_I2S_CH, (char *)result, TIMES, &ret_num, 100);

        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE) {
            if (ret == ESP_ERR_INVALID_STATE) {
                /**
                 * @note 1
                 * Issue:
                 * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
                 * fast for the task to handle. In this condition, some conversion results lost.
                 *
                 * Reason:
                 * When this error occurs, you will usually see the task watchdog timeout issue also.
                 * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
                 * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
                 * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
                 *
                 * Solution:
                 * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
                 */
            }
            for (int i = 0; i<NUM_BANDS; i++){
                bandValues[i] = 0;
            }
            fft_config_t *real_fft_plan = fft_init(ret_num, FFT_REAL, FFT_FORWARD, NULL, NULL);

            //ESP_LOGI("TASK:", "ret is %x, ret_num is %d", ret, ret_num);
            // for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE) {
            //     adc_digi_output_data_t *p = (void*)&result[i];
            //     if (ADC_CONV_MODE == ADC_CONV_SINGLE_UNIT_1) {
            //         //ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %f", 1, p->type1.channel, y_cf[i]);
            //     }
            // }
            for (int k = 0 ; k < real_fft_plan->size ; k++){
                real_fft_plan->input[k] =0xffff&result[k];
                //ESP_LOGI(TAG, "Value: %d", result[k]);
                //ret_num2+=result[k];
            }
            //if(ret_num2==0)ESP_LOGI(TAG, "Zeros");
            fft_execute(real_fft_plan);
            //printf("DC component : %f\n", real_fft_plan->output[0]);  // DC is at [0]
            for (int k = 1 ; k < real_fft_plan->size / 2 ; k++){
              if (real_fft_plan->output[2*k]> NOISE) {                    // Add a crude noise filter
                if (k<=3 )           bandValues[0]  += (int)real_fft_plan->output[2*k];
                if (k>3   && k<=6  ) bandValues[1]  += (int)real_fft_plan->output[2*k];
                if (k>6   && k<=13 ) bandValues[2]  += (int)real_fft_plan->output[2*k];
                if (k>13  && k<=27 ) bandValues[3]  += (int)real_fft_plan->output[2*k];
                if (k>27  && k<=55 ) bandValues[4]  += (int)real_fft_plan->output[2*k];
                if (k>55  && k<=112) bandValues[5]  += (int)real_fft_plan->output[2*k];
                if (k>112 && k<=229) bandValues[6]  += (int)real_fft_plan->output[2*k];
                if (k>229          ) bandValues[7]  += (int)real_fft_plan->output[2*k];
                printf("%d-th freq : %f+j%f\n", k, real_fft_plan->output[2*k], real_fft_plan->output[2*k+1]);
              }
                
            }
            //printf("Middle component : %f\n", real_fft_plan->output[1]);  // N/2 is real and stored at [1]
            //printf("\e[1;1H\e[2J");
            for (uint8_t band = 0; band < NUM_BANDS; band++) {

                // Scale the bars for the display
                int barHeight = bandValues[band] / AMPLITUDE;
                // Small amount of averaging between frames
                barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;
                // Draw bars
                //printf("Bar %d, H %d\n",band , barHeight);
                 printf("%d",band);
                 for (int i =0; i<barHeight; i++){
                 printf("-");
                 }
                 printf("\n");
                // Save oldBarHeights for averaging later
                oldBarHeights[band] = barHeight;
            }
            // Don't forget to clean up at the end to free all the memory that was allocated
            fft_destroy(real_fft_plan);
            //See `note 1`
            vTaskDelay(1);
			//fft_tmp_func(ret_num);

			
        } else if (ret == ESP_ERR_TIMEOUT) {
            /**
             * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
             * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
             */
            ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
            vTaskDelay(1000);
        }

    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
}