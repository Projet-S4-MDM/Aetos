#ifndef __CONFIG_HPP__
#define __CONFIG_HPP__

#include "Arduino.h"

constexpr gpio_num_t PIN_PWM_1 = GPIO_NUM_4;
constexpr gpio_num_t PIN_PWM_2 = GPIO_NUM_5;
constexpr gpio_num_t PIN_PWM_3 = GPIO_NUM_6;
constexpr gpio_num_t PIN_PWM_4 = GPIO_NUM_7;


constexpr gpio_num_t PIN_ENCODER_A1 = GPIO_NUM_15;
constexpr gpio_num_t PIN_ENCODER_B1 = GPIO_NUM_16;

constexpr gpio_num_t PIN_ENCODER_A4 = GPIO_NUM_17;
constexpr gpio_num_t PIN_ENCODER_B4 = GPIO_NUM_18;

constexpr gpio_num_t PIN_ENCODER_A3 = GPIO_NUM_36;
constexpr gpio_num_t PIN_ENCODER_B3 = GPIO_NUM_35;

constexpr gpio_num_t PIN_ENCODER_A2 = GPIO_NUM_38;
constexpr gpio_num_t PIN_ENCODER_B2 = GPIO_NUM_37;

#endif