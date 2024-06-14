/**
 * @file control_pid.h
 * @brief Declaraciones de funciones y estructuras para el control PID y filtro de Kalman.
 */

#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include <math.h>

#define GY85_ADDR 0x53 ///< Dirección del acelerómetro en la GY-85
#define PI 3.14159265358979323846 ///< Valor de PI

/**
 * @brief Estructura para el filtro de Kalman.
 */
typedef struct {
    float q; ///< Variancia del proceso
    float r; ///< Variancia de la medida
    float x; ///< Valor estimado
    float p; ///< Estimación del error
    float k; ///< Ganancia de Kalman
} KalmanFilter;

/**
 * @brief Estructura para el controlador PID.
 */
typedef struct {
    float kp;  ///< Ganancia proporcional
    float ki;  ///< Ganancia integral
    float kd;  ///< Ganancia derivativa
    float integral; ///< Acumulador integral
    float previous_error; ///< Error anterior
    float setpoint; ///< Punto de referencia deseado
} PIDController;

/**
 * @brief Inicializa el filtro de Kalman.
 * 
 * @param filter Puntero a la estructura del filtro de Kalman.
 * @param q Variancia del proceso.
 * @param r Variancia de la medida.
 * @param initial_value Valor inicial estimado.
 */
void kalman_init(KalmanFilter *filter, float q, float r, float initial_value);

/**
 * @brief Inicializa el controlador PID.
 * 
 * @param controller Puntero a la estructura del controlador PID.
 * @param kp Ganancia proporcional.
 * @param ki Ganancia integral.
 * @param kd Ganancia derivativa.
 * @param setpoint Punto de referencia deseado.
 */
void pid_controller_init(PIDController *controller, float kp, float ki, float kd, float setpoint);

/**
 * @brief Inicializa la interfaz I2C.
 */
void i2c_init_gy();

/**
 * @brief Inicializa el sensor GY-85.
 */
void gy85_init();

/**
 * @brief Inicializa el PWM.
 */
void pwm_init_s();

/**
 * @brief Actualiza el filtro de Kalman con una nueva medida.
 * 
 * @param filter Puntero a la estructura del filtro de Kalman.
 * @param measurement Nueva medida.
 * @return Valor estimado actualizado.
 */
float kalman_update(KalmanFilter *filter, float measurement);

/**
 * @brief Actualiza el controlador PID con un nuevo valor medido.
 * 
 * @param controller Puntero a la estructura del controlador PID.
 * @param measured_value Valor medido.
 * @param dt Intervalo de tiempo desde la última actualización.
 * @return Señal de control calculada.
 */
float pid_controller_update(PIDController *controller, float measured_value, float dt);

/**
 * @brief Escribe un valor en un registro del sensor.
 * 
 * @param reg Dirección del registro.
 * @param value Valor a escribir en el registro.
 */
void write_register(uint8_t reg, uint8_t value);

/**
 * @brief Lee varios registros del sensor.
 * 
 * @param reg Dirección del primer registro a leer.
 * @param buf Buffer donde se almacenarán los datos leídos.
 * @param len Número de registros a leer.
 */
void read_registers(uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief Lee los valores del acelerómetro.
 * 
 * @param accX Puntero donde se almacenará el valor del eje X.
 * @param accY Puntero donde se almacenará el valor del eje Y.
 * @param accZ Puntero donde se almacenará el valor del eje Z.
 */
void read_accelerometer(int16_t *accX, int16_t *accY, int16_t *accZ);

/**
 * @brief Calcula el ángulo de pitch basado en los valores del acelerómetro.
 * 
 * @param accX Valor del eje X.
 * @param accY Valor del eje Y.
 * @param accZ Valor del eje Z.
 * @param pitch Puntero donde se almacenará el ángulo de pitch calculado.
 */
void calculate_pitch(int16_t accX, int16_t accY, int16_t accZ, float *pitch);

#endif // CONTROL_PID_H
