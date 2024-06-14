#include "control_pid.h"

#define GY85_ADDR 0x53 // Dirección del acelerómetro en la GY-85
#define PI 3.14159265358979323846
#define SERVO_PIN 1

// Estructura para el filtro de Kalman
// typedef struct {
//     float q; // Variancia del proceso
//     float r; // Variancia de la medida
//     float x; // Valor estimado
//     float p; // Estimación del error
//     float k; // Ganancia de Kalman
// } KalmanFilter;

// // Estructura para el controlador PID
// typedef struct {
//     float kp;  // Ganancia proporcional
//     float ki;  // Ganancia integral
//     float kd;  // Ganancia derivativa
//     float integral; // Acumulador integral
//     float previous_error; // Error anterior
//     float setpoint; // Punto de referencia deseado
// } PIDController;

// Inicializa el filtro de Kalman
void kalman_init(KalmanFilter *filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0;
    filter->k = 0.0;
}

// Aplica el filtro de Kalman a un valor nuevo
float kalman_update(KalmanFilter *filter, float measurement) {
    // Predicción
    filter->p += filter->q;

    // Actualización
    filter->k = filter->p / (filter->p + filter->r);
    filter->x += filter->k * (measurement - filter->x);
    filter->p *= (1 - filter->k);

    return filter->x;
}

// Inicializa el controlador PID
void pid_controller_init(PIDController *controller, float kp, float ki, float kd, float setpoint) {
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->integral = 0.0;
    controller->previous_error = 0.0;
    controller->setpoint = setpoint;
}

// Calcula la salida del controlador PID
float pid_controller_update(PIDController *controller, float measured_value, float dt) {
    float error = controller->setpoint - measured_value;
    controller->integral += error * dt;
    float derivative = (error - controller->previous_error) / dt;
    float control_output = controller->kp * error + controller->ki * controller->integral + controller->kd * derivative;
    controller->previous_error = error;
    return control_output;
}

// Inicializa el I2C
void i2c_init_gy() {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);
}

// Escribe en un registro del dispositivo
void write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[] = {reg, value};
    i2c_write_blocking(i2c0, GY85_ADDR, buf, 2, false);
}

// Lee datos del dispositivo
void read_registers(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(i2c0, GY85_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, GY85_ADDR, buf, len, false);
}

// Inicializa el GY-85
void gy85_init() {
    write_register(0x2D, 0x08); // Pone el acelerómetro en modo de medida
}

// Lee los valores del acelerómetro
void read_accelerometer(int16_t *accX, int16_t *accY, int16_t *accZ) {
    uint8_t buf[6];
    read_registers(0x32, buf, 6);
    *accX = (buf[1] << 8) | buf[0];
    *accY = (buf[3] << 8) | buf[2];
    *accZ = (buf[5] << 8) | buf[4];
}

// Calcula los ángulos de inclinación
void calculate_pitch(int16_t accX, int16_t accY, int16_t accZ, float *pitch) {
    *pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
}

// Configura el PWM para el servo motor
// void pwm_init_s() {
//     gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
//     uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
//     pwm_set_wrap(slice_num, 20000); // Periodo de 20ms (50Hz)
//     pwm_set_clkdiv(slice_num, 64.0); // División del reloj para obtener la frecuencia deseada
//     pwm_set_enabled(slice_num, true);
// }
// void setup_pwm(uint gpio, float duty_cycle) {
//     switch (gpio)
//     {
//     case 0:
//         if(duty_cycle<6.0){
//         duty_cycle=6.0;
//         }
//     else if(duty_cycle>9.5){
//         duty_cycle=9.5;
//         }
//         break;
//     case 1:
//         if(duty_cycle<7.0){
//         duty_cycle=7.0;
//         }
//     else if(duty_cycle>10.5){
//         duty_cycle=10.5;
//         }
//     default:
//         break;
//     }
    
//     // Configura el pin PWM
//     gpio_set_function(gpio, GPIO_FUNC_PWM);

//     // Obtén el slice del PWM para el pin
//     uint slice_num = pwm_gpio_to_slice_num(gpio);

//     // Configura el PWM con un divisor ajustado para obtener la frecuencia deseada
//     float clkdiv = 38.146;  // Divisor calculado para 50 Hz
//     pwm_set_clkdiv(slice_num, clkdiv);

//     // Configura el nivel de salida para el duty cycle
//     uint16_t level = duty_cycle * (float)(1 << 16) / 100.0f;
//     pwm_set_gpio_level(gpio, level);

//     // Habilita el PWM
//     pwm_set_enabled(slice_num, true);
// }
// Ajusta el ángulo del servo motor (0-180 grados) con límites personalizados
// void set_servo_angle(float angle, float min_angle, float max_angle) {
//     if (angle < min_angle) {
//         angle = min_angle;
//     } else if (angle > max_angle) {
//         angle = max_angle;
//     }
//     uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
//     uint16_t duty_cycle = (uint16_t)((angle * 100 / 9) + 500); // Convierte el ángulo a ciclo de trabajo
//     pwm_set_gpio_level(SERVO_PIN, duty_cycle);
// }

// int main() {
//     stdio_init_all();
//     i2c_init_gy();
//     gy85_init();
//     //pwm_init_s();

//     KalmanFilter kalman_filter;
//     kalman_init(&kalman_filter, 0.01, 0.1, 0); // Inicializa el filtro de Kalman

//     PIDController pid_controller;
//     pid_controller_init(&pid_controller, 1.0, 0.1, 0.05, 0); // Inicializa el controlador PID

//     float min_angle = 90; // Ángulo mínimo permitido para el servo
//     float max_angle = 110; // Ángulo máximo permitido para el servo

//     while (1) {
//         int16_t accX, accY, accZ;
//         float pitch, filtered_pitch, control_signal;

//         read_accelerometer(&accX, &accY, &accZ);
//         calculate_pitch(accX, accY, accZ, &pitch);

//         // Aplica el filtro de Kalman al ángulo de pitch
//         filtered_pitch = kalman_update(&kalman_filter, pitch);

//         // Calcula la señal de control usando el controlador PID
//         control_signal = pid_controller_update(&pid_controller, filtered_pitch, 0.1); // Suponiendo dt = 0.1s

//         // Ajusta el ángulo del servo motor basado en la señal de control con límites personalizados
//         //set_servo_angle(95, min_angle, max_angle);
//         setup_pwm(1,(control_signal/10.0)+9);
//         setup_pwm(0,(control_signal/10.0)+7.5);


//         printf("Raw Pitch: %.2f, Filtered Pitch: %.2f, Control Signal: %.2f\n", pitch, filtered_pitch, control_signal);

//         sleep_ms(200);
//     }

//     return 0;
// }

