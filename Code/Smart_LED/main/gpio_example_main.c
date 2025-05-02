/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/sdm.h"
#include <math.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "soc/soc_caps.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define EXAMPLE_ADC1_CHAN0      ADC_CHANNEL_7

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (0) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

#define LEDC_RED_IO                   (0)
#define LEDC_GREEN_IO                 (4)
#define LEDC_BLUE_IO                  (5)

#define GPIO_OUTPUT_IO_0    32
#define GPIO_OUTPUT_IO_1    33
#define GPIO_OUTPUT_IO_2    25
#define GPIO_OUTPUT_IO_3    26

#define LEDC_CHANNEL_RED              (LEDC_CHANNEL_0)
#define LEDC_CHANNEL_GREEN            (LEDC_CHANNEL_1)
#define LEDC_CHANNEL_BLUE             (LEDC_CHANNEL_2)

#define RGB_TO_DUTY(x)  (x * (1 << LEDC_DUTY_RES) / 8192)




#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

static const char* TAG = "EXAMPLE";
static int adc_raw[2][10];
static int voltage[2][10];
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

int R, G, B;
bool action = false;
void out_Led(int a);
int sensor_value;
// Structure to store R, G, B channels
typedef struct {
    uint32_t red_channel;
    uint32_t green_channel;
    uint32_t blue_channel;
} rgb_channel_config_t;

static rgb_channel_config_t rgb_led_1_channels = {
    .red_channel = LEDC_CHANNEL_RED,
    .green_channel = LEDC_CHANNEL_GREEN,
    .blue_channel = LEDC_CHANNEL_BLUE,
};

static void example_ledc_init(void){
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
        ledc_channel.channel = LEDC_CHANNEL_RED;
        ledc_channel.gpio_num = LEDC_RED_IO;
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        ledc_channel.channel = LEDC_CHANNEL_GREEN;
        ledc_channel.gpio_num = LEDC_GREEN_IO;
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        ledc_channel.channel = LEDC_CHANNEL_BLUE;
        ledc_channel.gpio_num = LEDC_BLUE_IO;
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static void rgb_set_duty_and_update(rgb_channel_config_t rgb_channels,
    uint32_t target_r_duty, uint32_t target_g_duty, uint32_t target_b_duty){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, rgb_channels.red_channel, target_r_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, rgb_channels.green_channel, target_g_duty));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, rgb_channels.blue_channel, target_b_duty));

    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, rgb_channels.red_channel));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, rgb_channels.green_channel));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, rgb_channels.blue_channel));
}
void app_main (void){
    example_ledc_init();
    gpio_set_direction (GPIO_OUTPUT_IO_2, GPIO_MODE_INPUT );
    gpio_set_direction (GPIO_OUTPUT_IO_1, GPIO_MODE_INPUT );
    gpio_set_direction (GPIO_OUTPUT_IO_0, GPIO_MODE_INPUT );
    gpio_set_direction (GPIO_OUTPUT_IO_3, GPIO_MODE_INPUT );
    
    int max, min;
    int value;
    int last;
    last = gpio_get_level (GPIO_OUTPUT_IO_1);
    value = 0;
    max = 20; min = 0;

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
    .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN0, &config));


    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_chan0_handle = NULL;
    bool do_calibration1_chan0 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC1_CHAN0, EXAMPLE_ADC_ATTEN, &adc1_cali_chan0_handle);
   
    while (1){
        int buttonState = gpio_get_level (GPIO_OUTPUT_IO_2);
        int sensorState = gpio_get_level (GPIO_OUTPUT_IO_3);
        //printf("Button State: %d\n", buttonState);
        while(buttonState == 0){
            int buttonState = gpio_get_level (GPIO_OUTPUT_IO_2);
            while(buttonState == 1){
                action = !action;
                //printf("action: %d\n", action);
                goto exit;
            }
        }
        exit:
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
            sensor_value = voltage[0][0];
            //printf("sensor_value: %d\n",sensor_value);
        }
        int state = gpio_get_level (GPIO_OUTPUT_IO_1);
        if(state != last){
            if(gpio_get_level (GPIO_OUTPUT_IO_0) != state){
                value ++;
            }
            else{
                value --;
            }
        }
        last = state;
        if(value > max){
            value = 1;    
        }  
        if(value <= min){
            value = 20;
        }  
        printf("value: %d\n",value); 
        vTaskDelay(pdMS_TO_TICKS(10));
        if(((sensor_value > 2000) || (action == true))&&(sensorState == 1)){
            out_Led(value);
        }
        else{
            R = 8192, G = 8192; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
        }
    } 
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
    example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
}

void out_Led(int a){
    switch(a){
        case 1:
            R = 6528, G = 6528; B = 6528;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 2:
            R = 3264, G = 3264; B = 3264;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 3:
            R = 1632, G = 1632; B = 1632;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 4:
            R = 0, G = 0; B = 0;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 5:
            R = 8192, G = 8192; B = 6528;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 6:
            R = 8192, G = 8192; B = 3264;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 7:
            R = 8192, G = 8192; B = 1632;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 8:
            R = 8192, G = 8192; B = 0;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 9:
            R = 8192, G = 6528; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 10:
            R = 8192, G = 3264; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 11:
            R = 8192, G = 1632; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 12:
            R = 8192, G = 0; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 13:
            R = 6528, G = 8192; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 14:
            R = 3264, G = 8192; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 15:
            R = 1632, G = 8192; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 16:
            R = 0, G = 8192; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 17:
            R = 6528, G = 6528; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 18:
            R = 3264, G = 3264; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 19:
            R = 1632, G = 1632; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        case 20:
            R = 0, G = 0; B = 8192;
            rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));
            break;
        default:
        R = 8192, G = 8192; B = 8192;
        rgb_set_duty_and_update(rgb_led_1_channels, RGB_TO_DUTY(R), RGB_TO_DUTY(G), RGB_TO_DUTY(B));

    }
}

    
/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle){
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

    