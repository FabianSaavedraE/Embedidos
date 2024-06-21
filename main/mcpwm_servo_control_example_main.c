#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "esp_attr.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define SERVO_MIN_PULSEWIDTH 500 // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180 // Maximum angle in degree upto which servo can rotate

#define SERVO_PIN 12 // GPIO where the servo is connected

static void mcpwm_example_gpio_initialize(void) {
    printf("Initializing MCPWM servo control GPIO...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_PIN); // Set GPIO 12 as PWM0A
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
    uint32_t cal_pulsewidth = 0;
    cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
    return cal_pulsewidth;
}

static void mcpwm_example_servo_control(void *arg) {
    uint32_t angle;
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // Frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     // Duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // Duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    // Configure PWM0A with above settings

    while (1) {
        for (int count = 0; count <= SERVO_MAX_DEGREE; count++) {
            angle = servo_per_degree_init(count);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(20 / portTICK_PERIOD_MS); // Add delay, since it takes time for servo to rotate
        }
        for (int count = SERVO_MAX_DEGREE; count >= 0; count--) {
            angle = servo_per_degree_init(count);
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle);
            vTaskDelay(20 / portTICK_PERIOD_MS); // Add delay, since it takes time for servo to rotate
        }
    }
}

void app_main(void) {
    printf("Testing servo motor...\n");
    mcpwm_example_gpio_initialize();
    xTaskCreate(mcpwm_example_servo_control, "mcpwm_example_servo_control", 4096, NULL, 5, NULL);
}