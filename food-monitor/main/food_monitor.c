#include "stdio.h"
#include "stdlib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#define LED_1_PIN GPIO_NUM_25
#define LED_2_PIN GPIO_NUM_27
#define LED_3_PIN GPIO_NUM_26

#define BUTTON_1_PIN GPIO_NUM_21
#define BUTTON_2_PIN GPIO_NUM_19
#define BUTTON_3_PIN GPIO_NUM_18
#define WEIGHT_SENSOR_ZERO_PIN BUTTON_1_PIN
#define WEIGHT_SENSOR_CALIBRATE_PIN BUTTON_2_PIN

#define FOOD_DISPENSER_DIR_PIN GPIO_NUM_14
#define FOOD_DISPENSER_STEP_PIN GPIO_NUM_13
#define FOOD_DISPENSER_MANUAL_PIN BUTTON_3_PIN
#define FOOD_DISPENSER_PIN_SEL  ((1ULL<<FOOD_DISPENSER_DIR_PIN) | (1ULL<<FOOD_DISPENSER_STEP_PIN))

#define HX711_DT_PIN GPIO_NUM_22
#define HX711_SCK_PIN GPIO_NUM_23

#define LEDS_PIN_SEL ((1ULL<<LED_1_PIN) | (1ULL<<LED_2_PIN) | (1ULL<<LED_3_PIN))
#define BUTTONS_PIN_SEL ((1ULL<<BUTTON_1_PIN) | (1ULL<<BUTTON_2_PIN) | (1ULL<<BUTTON_3_PIN))

#define BUTTON_CHECK_PERIOD 500

typedef enum {
    HX711_GAIN_128,
	HX711_GAIN_32,
	HX711_GAIN_64
} HX711_GAIN_T;

static TaskHandle_t zero_task_handle;
static TaskHandle_t dispense_food_handle;
static TaskHandle_t calibrate_task_handle;
static TaskHandle_t get_weight_readings_task_handle;
static TickType_t weight_sensor_zero_offset_next;
static TickType_t weight_sensor_calibrate_next;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

static uint8_t calibration_weight = 130; // 17HS4023 stepper motor weights 130 g 
static uint8_t weight = 0;
static uint32_t zero = 8281192;
static int32_t offset = 0;
static float scale = -0.010125;
static float min_weight = 5;
static float max_weight = 90;
static HX711_GAIN_T default_gain = HX711_GAIN_128;
static bool calibration_complete = true;
const char * TAG = "food_monitor";

static void IRAM_ATTR weight_sensor_zero_offset_isr(void *arg) {
    TickType_t current = xTaskGetTickCountFromISR();

    if (current > weight_sensor_zero_offset_next) {
        vTaskNotifyGiveFromISR(zero_task_handle, NULL);

        weight_sensor_zero_offset_next = current + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);
    }
}

static void IRAM_ATTR weight_sensor_calibrate_isr(void *arg) {
    TickType_t current = xTaskGetTickCountFromISR();

    if (current > weight_sensor_calibrate_next) {
        vTaskNotifyGiveFromISR(calibrate_task_handle, NULL);

        weight_sensor_calibrate_next = current + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);
    }
}

static uint32_t weight_sensor_get_raw_data(HX711_GAIN_T gain) {
    uint32_t raw_data = 0;
    uint8_t gain_clock_cycles = 0;

    taskENTER_CRITICAL(&spinlock);
    gpio_set_level(HX711_SCK_PIN, false);
    ets_delay_us(1);

    while (gpio_get_level(HX711_DT_PIN));
    
    for (uint8_t index = 0; index < 24; index++) {
        gpio_set_level(HX711_SCK_PIN, true);
        ets_delay_us(1);
        raw_data = raw_data << 1;
        gpio_set_level(HX711_SCK_PIN, false);
        ets_delay_us(1);
        
        if (gpio_get_level(HX711_DT_PIN)) {
            raw_data++;
        }
    }
    
    switch (gain) {
        case HX711_GAIN_128:
			gain_clock_cycles = 1;
			break;
			
		case HX711_GAIN_32:
			gain_clock_cycles = 2;
			break;
			
		case HX711_GAIN_64:
			gain_clock_cycles = 3;
			break;
    }

    for (uint8_t index = 0; index < gain_clock_cycles; index++) {
        gpio_set_level(HX711_SCK_PIN, true);
        ets_delay_us(1);
        gpio_set_level(HX711_SCK_PIN, false);
        ets_delay_us(1);
    }

    taskEXIT_CRITICAL(&spinlock);

    raw_data = raw_data ^ 0x800000;	
    return raw_data;
}

static uint32_t weight_sensor_get_average_raw_data(uint8_t times, HX711_GAIN_T gain) {
    uint32_t raw_data = 0;

    for (uint8_t index = 0; index < times; index++) {
        raw_data += weight_sensor_get_raw_data(gain);
    }    

    return raw_data / times;
}

static void weight_sensor_zero_offset_task(void *arg) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ESP_LOGI(TAG, "Setting zero offset for weight sensor...");

        zero = weight_sensor_get_average_raw_data(10, default_gain);

        ESP_LOGI(TAG, "Zero value for weight sensor: %d...", zero);
    }         
}

static float weight_sensor_get_value(void) {
    int32_t data = (weight_sensor_get_average_raw_data(3, default_gain) - zero);
	return ((float)(data)) * scale;
}

static void weight_sensor_calibrate_task(void *arg) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);    
        
        ESP_LOGI(TAG, "Calibrating...");
        uint32_t raw_data = weight_sensor_get_average_raw_data(10, default_gain);

        offset = raw_data - zero;
        scale = ((float)calibration_weight) / ((float)offset);
        uint32_t w = (int32_t)weight_sensor_get_value();

        if (w > calibration_weight - 2 && w < calibration_weight + 2) {
            calibration_complete = true;
            vTaskResume(get_weight_readings_task_handle);
        } else {
            ESP_LOGE(TAG, "Calibration process failed...");
        }

        ESP_LOGI(TAG, "Scale value: %f", scale);
    }  
}

static void weight_sensor_readings_task(void *arg) {
    for(;;) {
        if (calibration_complete) {
            weight = (int32_t)weight_sensor_get_value();    
            printf("Weight: %d\n", weight);
            vTaskDelay(20000 / portTICK_PERIOD_MS);
        } else {
            printf("Waiting for calibration...\n");
            vTaskDelay(20000 / portTICK_PERIOD_MS);
        }
    }
}

static void weight_sensor_trigger_low_food_task(void *arg) {
    for(;;) {
        ESP_LOGI(TAG, "Checking if there is enough food...");
        if (weight < min_weight && calibration_complete) {
            xTaskNotifyGive(dispense_food_handle);
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

static void food_dispenser_move() {
    for (uint8_t index = 0; index < 200; index++) {
        gpio_set_level(FOOD_DISPENSER_STEP_PIN, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(FOOD_DISPENSER_STEP_PIN, 0); 
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

static void food_dispenser_task(void *arg) {
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        ESP_LOGI(TAG, "Giving food to cats...");

        if(weight > max_weight) {
            ESP_LOGI(TAG, "Too much food for your cats!");
        } else {
            food_dispenser_move();
        }
    }         
}

static void food_dispenser_init(void) {
    gpio_config_t config_outputs = {    
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = FOOD_DISPENSER_PIN_SEL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config_t config_buttons = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << FOOD_DISPENSER_MANUAL_PIN)
    };

    gpio_config(&config_outputs);
    gpio_config(&config_buttons);

    xTaskCreate(food_dispenser_task, "food_dispenser_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &dispense_food_handle);
}

static void configure_leds(void) {
    gpio_config_t config_leds = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = LEDS_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };

    gpio_config(&config_leds);
}

static void configure_hx711(void) {
    gpio_config_t config_buttons = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = ((1ULL << WEIGHT_SENSOR_ZERO_PIN) | (1ULL << WEIGHT_SENSOR_CALIBRATE_PIN))
    };

    gpio_config_t config_sck = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = (1ULL << HX711_SCK_PIN)
    };

    gpio_config_t config_dt = {
        .pin_bit_mask = (1ULL << HX711_DT_PIN),
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .mode = GPIO_MODE_INPUT
    };

    gpio_config(&config_buttons);
    gpio_config(&config_sck);
    gpio_config(&config_dt);

    gpio_isr_handler_add(WEIGHT_SENSOR_ZERO_PIN, weight_sensor_zero_offset_isr, NULL);
    gpio_isr_handler_add(WEIGHT_SENSOR_CALIBRATE_PIN, weight_sensor_calibrate_isr, NULL);
    
    xTaskCreate(weight_sensor_zero_offset_task, "zero_offset_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &zero_task_handle);
    xTaskCreate(weight_sensor_calibrate_task, "calibrate_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, &calibrate_task_handle);
    xTaskCreate(weight_sensor_readings_task, "readings_task", configMINIMAL_STACK_SIZE * 5, NULL, 8, &get_weight_readings_task_handle);
    xTaskCreate(weight_sensor_trigger_low_food_task, "trigger_low_food_task", configMINIMAL_STACK_SIZE * 5, NULL, 5, NULL);
}

void app_main(void) {
    TickType_t current = xTaskGetTickCount() + (BUTTON_CHECK_PERIOD / portTICK_PERIOD_MS);    
    weight_sensor_zero_offset_next = current;

    gpio_install_isr_service(0);
    food_dispenser_init();
    configure_leds();
    configure_hx711();
    

    while (true) {
        gpio_set_level(LED_1_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(LED_1_PIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
