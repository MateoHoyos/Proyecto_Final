/**
 * @file control_pid.c
 * @brief Implementación de funciones para el control PID y el filtro de Kalman.
 */

#include "control_pid.h"

#define GY85_ADDR 0x53 ///< Dirección del acelerómetro en la GY-85
#define PI 3.14159265358979323846 ///< Valor de PI
#define SERVO_PIN 1 ///< Pin del servo

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



/**
 * @brief Inicializa el filtro de Kalman.
 * 
 * @param filter Puntero a la estructura del filtro de Kalman.
 * @param q Variancia del proceso.
 * @param r Variancia de la medida.
 * @param initial_value Valor inicial estimado.
 */
void kalman_init(KalmanFilter *filter, float q, float r, float initial_value) {
    filter->q = q;
    filter->r = r;
    filter->x = initial_value;
    filter->p = 1.0;
    filter->k = 0.0;
}

/**
 * @brief Aplica el filtro de Kalman a un valor nuevo.
 * 
 * @param filter Puntero a la estructura del filtro de Kalman.
 * @param measurement Nueva medida.
 * @return Valor estimado actualizado.
 */
float kalman_update(KalmanFilter *filter, float measurement) {
    // Predicción
    filter->p += filter->q;

    // Actualización
    filter->k = filter->p / (filter->p + filter->r);
    filter->x += filter->k * (measurement - filter->x);
    filter->p *= (1 - filter->k);

    return filter->x;
}

/**
 * @brief Inicializa el controlador PID.
 * 
 * @param controller Puntero a la estructura del controlador PID.
 * @param kp Ganancia proporcional.
 * @param ki Ganancia integral.
 * @param kd Ganancia derivativa.
 * @param setpoint Punto de referencia deseado.
 */
void pid_controller_init(PIDController *controller, float kp, float ki, float kd, float setpoint) {
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->integral = 0.0;
    controller->previous_error = 0.0;
    controller->setpoint = setpoint;
}

/**
 * @brief Calcula la salida del controlador PID.
 * 
 * @param controller Puntero a la estructura del controlador PID.
 * @param measured_value Valor medido.
 * @param dt Intervalo de tiempo desde la última actualización.
 * @return Señal de control calculada.
 */
float pid_controller_update(PIDController *controller, float measured_value, float dt) {
    float error = controller->setpoint - measured_value;
    controller->integral += error * dt;
    float derivative = (error - controller->previous_error) / dt;
    float control_output = controller->kp * error + controller->ki * controller->integral + controller->kd * derivative;
    controller->previous_error = error;
    return control_output;
}

/**
 * @brief Inicializa la interfaz I2C.
 */
void i2c_init_gy() {
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);
}

/**
 * @brief Escribe en un registro del dispositivo.
 * 
 * @param reg Dirección del registro.
 * @param value Valor a escribir en el registro.
 */
void write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[] = {reg, value};
    i2c_write_blocking(i2c0, GY85_ADDR, buf, 2, false);
}

/**
 * @brief Lee datos del dispositivo.
 * 
 * @param reg Dirección del primer registro a leer.
 * @param buf Buffer donde se almacenarán los datos leídos.
 * @param len Número de registros a leer.
 */
void read_registers(uint8_t reg, uint8_t *buf, uint8_t len) {
    i2c_write_blocking(i2c0, GY85_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, GY85_ADDR, buf, len, false);
}

/**
 * @brief Inicializa el sensor GY-85.
 */
void gy85_init() {
    write_register(0x2D, 0x08); // Pone el acelerómetro en modo de medida
}

/**
 * @brief Lee los valores del acelerómetro.
 * 
 * @param accX Puntero donde se almacenará el valor del eje X.
 * @param accY Puntero donde se almacenará el valor del eje Y.
 * @param accZ Puntero donde se almacenará el valor del eje Z.
 */
void read_accelerometer(int16_t *accX, int16_t *accY, int16_t *accZ) {
    uint8_t buf[6];
    read_registers(0x32, buf, 6);
    *accX = (buf[1] << 8) | buf[0];
    *accY = (buf[3] << 8) | buf[2];
    *accZ = (buf[5] << 8) | buf[4];
}

/**
 * @brief Calcula los ángulos de inclinación.
 * 
 * @param accX Valor del eje X.
 * @param accY Valor del eje Y.
 * @param accZ Valor del eje Z.
 * @param pitch Puntero donde se almacenará el ángulo de pitch calculado.
 */
void calculate_pitch(int16_t accX, int16_t accY, int16_t accZ, float *pitch) {
    *pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
}
