/*
 * stepper.h
 *
 *  Created on: Mar 11, 2026
 *      Author: mrsoh
 */

#ifndef SRC_STEPPER_H_
#define SRC_STEPPER_H_

#include "main.h"

/* ── Public API ────────────────────────────────────────────────────────────── */

/**
 * Call once from USER CODE BEGIN 2 in main.c
 * Engages all brakes and arms UART receive interrupt.
 */
void stepper_init(void);

/**
 * Call from HAL_TIM_PWM_PulseFinishedCallback in main.c:
 *
 *   void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
 *       stepper_pulse_callback(htim);
 *   }
 */
void stepper_pulse_callback(TIM_HandleTypeDef *htim);

/**
 * Call from HAL_UART_RxCpltCallback in main.c:
 *
 *   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 *       stepper_uart_callback(huart);
 *   }
 */
void stepper_uart_callback(UART_HandleTypeDef *huart);

/* ── Command format (4 bytes from Raspberry Pi over UART2 at 115200) ───────
 *
 *  cmd[0] = motor_id  : 1=TopLeft(brake), 2=TopRight(brake),
 *                       3=BottomRight,    4=BottomLeft
 *  cmd[1] = direction : 0=CW, 1=CCW
 *  cmd[2] = speed     : actual steps/sec = cmd[2] * 25
 *                         0  -> stop
 *                         8  ->  200 steps/sec  (~7.5  RPM)
 *                        32  ->  800 steps/sec  (~30   RPM)
 *                        64  -> 1600 steps/sec  (~60   RPM)
 *                       128  -> 3200 steps/sec  (~120  RPM)
 *  cmd[3] = steps     : pulses to move = cmd[3] * 16
 *                         0   -> run indefinitely until stopped
 *                       100  -> 1600 pulses = 1 revolution
 *                       200  -> 3200 pulses = 2 revolutions
 *
 * Sample commands:
 *   {1, 0,  8, 100} -> Motor1 CW,   200 steps/sec, 1 rev
 *   {2, 1, 32, 100} -> Motor2 CCW,  800 steps/sec, 1 rev
 *   {3, 0, 64, 200} -> Motor3 CW,  1600 steps/sec, 2 rev
 *   {4, 1, 16,  50} -> Motor4 CCW,  400 steps/sec, 0.5 rev
 *   {2, 0,  0,   0} -> Motor2 STOP + engage brake
 * ────────────────────────────────────────────────────────────────────────── */

/* ── CubeMX Timer Settings ──────────────────────────────────────────────────
 *
 *  TIM2, TIM3, TIM4  (APB1 @ 84MHz):   PSC = 83,  Period = 4999, Pulse = 2499
 *  TIM1              (APB2 @ 168MHz):   PSC = 167, Period = 4999, Pulse = 2499
 *  Mode: PWM Mode 1, Output Compare Preload: Enable
 *
 *  Your OLD config (PSC=17999, Period=9999) = 0.47Hz = unusable for stepping!
 * ────────────────────────────────────────────────────────────────────────── */
void stepper_test(void);
void stepper_test2(void);
void stepper_test3(void);


#endif /* SRC_STEPPER_H_ */
