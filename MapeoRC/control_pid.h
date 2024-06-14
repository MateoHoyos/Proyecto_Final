#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>
#define GY85_ADDR 0x53 // Dirección del acelerómetro en la GY-85
#define PI 3.14159265358979323846

// Estructura para el filtro de Kalman
typedef struct {
    float q; // Variancia del proceso
    float r; // Variancia de la medida
    float x; // Valor estimado
    float p; // Estimación del error
    float k; // Ganancia de Kalman
} KalmanFilter;

// Estructura para el controlador PID
typedef struct {
    float kp;  // Ganancia proporcional
    float ki;  // Ganancia integral
    float kd;  // Ganancia derivativa
    float integral; // Acumulador integral
    float previous_error; // Error anterior
    float setpoint; // Punto de referencia deseado
} PIDController;

// Funciones de inicialización
void kalman_init(KalmanFilter *filter, float q, float r, float initial_value);
void pid_controller_init(PIDController *controller, float kp, float ki, float kd, float setpoint);
void i2c_init_gy();
void gy85_init();
void pwm_init_s();

// Funciones de actualización
float kalman_update(KalmanFilter *filter, float measurement);
float pid_controller_update(PIDController *controller, float measured_value, float dt);

// Funciones de manejo de sensores
void write_register(uint8_t reg, uint8_t value);
void read_registers(uint8_t reg, uint8_t *buf, uint8_t len);
void read_accelerometer(int16_t *accX, int16_t *accY, int16_t *accZ);
void calculate_pitch(int16_t accX, int16_t accY, int16_t accZ, float *pitch);



#endif // CONTROL_PID_H