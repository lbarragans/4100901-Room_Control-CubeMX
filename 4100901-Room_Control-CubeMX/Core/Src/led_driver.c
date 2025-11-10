/**
  ******************************************************************************
  * @file    led_driver.c
  * @brief   Implementación de la librería para control de LEDs (LD2 y LED_DOOR)
  ******************************************************************************
  */

#include "led_driver.h"

/**
 * @brief  Inicializa el LED apagándolo al inicio.
 */
void led_init(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief  Enciende el LED especificado.
 */
void led_on(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

/**
 * @brief  Apaga el LED especificado.
 */
void led_off(led_handle_t *led) {
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

/**
 * @brief  Cambia el estado del LED (encendido ↔ apagado).
 */
void led_toggle(led_handle_t *led) {
    HAL_GPIO_TogglePin(led->port, led->pin);
}
