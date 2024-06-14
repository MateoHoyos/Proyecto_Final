/**
 * @file main.c
 * @brief Control PWM y PID para un sistema de dirección y elevación.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "control_pid.h"

// Pines al que se conecta la señal PWM
#define PWM_Cn1 0   ///< Dirección
#define PWM_Cn2 1   ///< Elevación
#define PWM_Cn4 2   ///< Alas
#define PWM_Cn6 3   ///< Switch control

// Pines al que se conecta la señal PWM saliente
#define PWM_OUT1 4  ///< Dirección d
#define PWM_OUT2 5  ///< Dirección t
#define PWM_OUT3 6  ///< Elevación
#define PWM_OUT4 7  ///< Ala derecha
#define PWM_OUT5 8  ///< Ala izquierda

/**
 * @brief Mide el ciclo de trabajo de la señal PWM en el pin especificado.
 * 
 * @param gpio Pin GPIO al cual se conecta la señal PWM.
 * @return Ciclo de trabajo en porcentaje.
 */
float measure_duty_cycle(uint gpio) {
    absolute_time_t t1, t2, t3, t4;
    uint32_t high_time, low_time;
    float duty_cycle;

    // Espera a que la señal sea alta
    while (!gpio_get(gpio));

    // Marca el tiempo de inicio del pulso alto
    t1 = get_absolute_time();
    
    // Espera a que la señal sea baja
    while (gpio_get(gpio));
    
    // Marca el tiempo de fin del pulso alto y comienzo del pulso bajo
    t2 = get_absolute_time();

    // Espera a que la señal sea alta nuevamente
    while (!gpio_get(gpio));
    
    // Marca el tiempo de fin del pulso bajo
    t3 = get_absolute_time();

    // Calcula el tiempo en alto y en bajo
    high_time = absolute_time_diff_us(t1, t2);
    low_time = absolute_time_diff_us(t2, t3);

    // Calcula el ciclo de trabajo
    duty_cycle = (float)high_time / (high_time + low_time) * 100.0f;

    return duty_cycle;
}

/**
 * @brief Configura el pin GPIO para generar una señal PWM con el ciclo de trabajo especificado.
 * 
 * @param gpio Pin GPIO al cual se generará la señal PWM.
 * @param duty_cycle Ciclo de trabajo en porcentaje.
 */
void setup_pwm(uint gpio, float duty_cycle) {
    switch (gpio) {
        case 7:
            if(duty_cycle < 7.0) {
                duty_cycle = 7.0;
            } else if(duty_cycle > 10.5) {
                duty_cycle = 10.5;
            }
            break;
        case 8:
            if(duty_cycle < 6.0) {
                duty_cycle = 6.0;
            } else if(duty_cycle > 9.5) {
                duty_cycle = 9.5;
            }
            break;
        default:
            break;
    }
    
    // Configura el pin PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Obtén el slice del PWM para el pin
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Configura el PWM con un divisor ajustado para obtener la frecuencia deseada
    float clkdiv = 38.146;  // Divisor calculado para 50 Hz
    pwm_set_clkdiv(slice_num, clkdiv);

    // Configura el nivel de salida para el duty cycle
    uint16_t level = duty_cycle * (float)(1 << 16) / 100.0f;
    pwm_set_gpio_level(gpio, level);

    // Habilita el PWM
    pwm_set_enabled(slice_num, true);
}

/**
 * @brief Función principal. Configura los pines, inicializa los periféricos y ejecuta el bucle principal.
 * 
 * @return Código de estado del programa.
 */
int main() {
    stdio_init_all();
    
    // Configura los pines como entrada
    gpio_init(PWM_Cn1);
    gpio_set_dir(PWM_Cn1, GPIO_IN);
    gpio_init(PWM_Cn2);
    gpio_set_dir(PWM_Cn2, GPIO_IN);
    gpio_init(PWM_Cn4);
    gpio_set_dir(PWM_Cn4, GPIO_IN);
    gpio_init(PWM_Cn6);
    gpio_set_dir(PWM_Cn6, GPIO_IN);
    
    // Configura los pines de salida PWM
    gpio_set_function(PWM_OUT1, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT2, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT3, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT4, GPIO_FUNC_PWM);
    gpio_set_function(PWM_OUT5, GPIO_FUNC_PWM);

    // Inicializa periféricos adicionales
    i2c_init_gy();
    gy85_init();
    
    KalmanFilter kalman_filter;
    kalman_init(&kalman_filter, 0.01, 0.1, 0); // Inicializa el filtro de Kalman

    PIDController pid_controller;
    pid_controller_init(&pid_controller, 1.0, 0.1, 0.05, 0); // Inicializa el controlador PID

    // Bucle principal
    while (1) {
        float duty_cycle1 = measure_duty_cycle(PWM_Cn1);
        float duty_cycle2 = measure_duty_cycle(PWM_Cn2);
        float duty_cycle3 = measure_duty_cycle(PWM_Cn4);
        float duty_cycle4 = measure_duty_cycle(PWM_Cn6);
        duty_cycle3 = (8.3 + (8.3 - duty_cycle3));
        printf("Ciclo de trabajo: %.2f\n", duty_cycle1);
        setup_pwm(PWM_OUT1, duty_cycle1 - 0.6);
        setup_pwm(PWM_OUT2, (8.3 + (8.3 - duty_cycle1)));
        setup_pwm(PWM_OUT3, duty_cycle2 + 1.0);

        if (duty_cycle4 < 9.0 && (duty_cycle3 < 8.5 && duty_cycle3 > 8.1)) {
            int16_t accX, accY, accZ;
            float pitch, filtered_pitch, control_signal;

            read_accelerometer(&accX, &accY, &accZ);
            calculate_pitch(accX, accY, accZ, &pitch);

            // Aplica el filtro de Kalman al ángulo de pitch
            filtered_pitch = kalman_update(&kalman_filter, pitch + 3.0);

            // Calcula la señal de control usando el controlador PID
            control_signal = pid_controller_update(&pid_controller, filtered_pitch, 0.1); // Suponiendo dt = 0.1s

            // Ajusta el ángulo del servo motor basado en la señal de control con límites personalizados
            setup_pwm(PWM_OUT4, (control_signal / 10.0) + 9.0);
            setup_pwm(PWM_OUT5, (control_signal / 10.0) + 7.5);

            printf("Raw Pitch: %.2f, Filtered Pitch: %.2f, Control Signal: %.2f\n", pitch, filtered_pitch, control_signal);
        } else {
            setup_pwm(PWM_OUT4, duty_cycle3 + 0.5);
            setup_pwm(PWM_OUT5, duty_cycle3 - 0.9);
        }
        
        sleep_ms(80);  // Espera antes de medir nuevamente
    }

    return 0;
}
