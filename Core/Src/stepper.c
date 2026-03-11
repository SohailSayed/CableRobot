#include "stepper.h"
#include <string.h>

/* ── External handles (defined in main.c by CubeMX) ───────────────────────── */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

/* ── Config ────────────────────────────────────────────────────────────────── */
#define NUM_MOTORS        4
#define PULSES_PER_REV    1600      // DM556 DIP switch: 1600 pulses/rev
#define TIMER_CLOCK_HZ    1000000UL // 1MHz after prescaler (PSC=83 on APB1 timers)

/* Direction polarity — swap DIR_CW/DIR_CCW if a motor runs backwards */
#define DIR_CW            GPIO_PIN_RESET
#define DIR_CCW           GPIO_PIN_SET

/* Brake polarity:
 *   Power-off brake = engaged when coil is unpowered.
 *   Your GPIO drives an NPN transistor switching 24V to the brake coil.
 *   GPIO HIGH -> transistor ON  -> 24V to coil -> BRAKE RELEASED
 *   GPIO LOW  -> transistor OFF -> coil off    -> BRAKE ENGAGED        */
#define BRAKE_RELEASE     GPIO_PIN_SET
#define BRAKE_ENGAGE      GPIO_PIN_RESET

/* How long to wait after releasing brake before sending pulses (ms) */
#define BRAKE_RELEASE_DELAY_MS  50

/* ── Motor descriptor ──────────────────────────────────────────────────────── */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           tim_channel;
    GPIO_TypeDef      *dir_port;
    uint16_t           dir_pin;
    GPIO_TypeDef      *brake_port;   // NULL if motor has no brake
    uint16_t           brake_pin;
    volatile uint32_t  steps_remaining;
    volatile uint8_t   running;
} Motor_t;

/* ── Motor table ───────────────────────────────────────────────────────────── *
 * Index 0 = Motor ID 1 (BottomLeft,  no brake)                               *
 * Index 1 = Motor ID 2 (TopRight,   has brake)                               *
 * Index 2 = Motor ID 3 (BottomRight, no brake)                               *
 * Index 3 = Motor ID 4 (TopLeft,  has brake)                               *
 * ─────────────────────────────────────────────────────────────────────────── */
static Motor_t motors[NUM_MOTORS] = {
    { &htim2, TIM_CHANNEL_1, GPIOC, GPIO_PIN_8,  NULL,  0,          0, 0 },
    { &htim4, TIM_CHANNEL_1, GPIOA, GPIO_PIN_10, GPIOA, GPIO_PIN_6, 0, 0 },
    { &htim1, TIM_CHANNEL_2, GPIOC, GPIO_PIN_4,  NULL,  0,          0, 0 },
    { &htim3, TIM_CHANNEL_4, GPIOC, GPIO_PIN_6,  GPIOA, GPIO_PIN_5, 0, 0 },
};

/* ── UART receive state ────────────────────────────────────────────────────── */
#define CMD_LEN   4
static uint8_t rx_byte;
static uint8_t cmd_buf[CMD_LEN];
static uint8_t cmd_idx = 0;

/* ── Private functions ─────────────────────────────────────────────────────── */

/**
 * Set timer period to achieve the desired pulse frequency.
 * With a 1MHz timer clock: period = (1,000,000 / steps_per_sec) - 1
 */
static void motor_set_speed(Motor_t *m, uint32_t steps_per_sec)
{
    if (steps_per_sec == 0) return;

    uint32_t period = (TIMER_CLOCK_HZ / steps_per_sec) - 1;
    if (period < 3)     period = 3;      // min ~250kHz, well above DM556 200kHz max anyway
    if (period > 65535) period = 65535;  // 16-bit timer limit

    __HAL_TIM_SET_AUTORELOAD(m->htim, period);
    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, period / 2); // 50% duty
}

/**
 * Stop a motor immediately and engage its brake (if any).
 */
static void motor_stop_internal(Motor_t *m)
{
	HAL_TIM_PWM_Stop(m->htim, m->tim_channel);
	HAL_TIM_Base_Stop_IT(m->htim);
    m->running = 0;
    m->steps_remaining = 0;

    if (m->brake_port != NULL) {
        HAL_GPIO_WritePin(m->brake_port, m->brake_pin, BRAKE_ENGAGE);
    }
}

/* ── Public: move a motor ──────────────────────────────────────────────────── */
static void motor_move(uint8_t motor_id, uint8_t direction,
                       uint32_t steps_per_sec, uint32_t steps)
{
    if (motor_id < 1 || motor_id > NUM_MOTORS) return;
    Motor_t *m = &motors[motor_id - 1];

    /* Stop any current motion cleanly */
    motor_stop_internal(m);

    /* Release brake before moving */
    if (m->brake_port != NULL) {
        HAL_GPIO_WritePin(m->brake_port, m->brake_pin, BRAKE_RELEASE);
        HAL_Delay(BRAKE_RELEASE_DELAY_MS);
    }

    /* Set direction — must be stable >5us before first pulse (DM556 spec) */
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin,
                      direction ? DIR_CCW : DIR_CW);
    HAL_Delay(1);

    /* Configure speed */
    motor_set_speed(m, steps_per_sec);

    /* Arm step counter (0 = run until explicitly stopped) */
    m->steps_remaining = steps;
    m->running = 1;

    HAL_TIM_PWM_Start(m->htim, m->tim_channel);  // PWM output no IT
    __HAL_TIM_ENABLE_IT(m->htim, TIM_IT_UPDATE);  // count periods instead
    HAL_TIM_Base_Start_IT(m->htim);
}

/* ── Public: stop a motor ──────────────────────────────────────────────────── */
static void motor_stop(uint8_t motor_id)
{
    if (motor_id < 1 || motor_id > NUM_MOTORS) return;
    motor_stop_internal(&motors[motor_id - 1]);
}

/* ── Command parser ────────────────────────────────────────────────────────── *
 *  cmd[0] = motor_id  (1–4)                                                   *
 *  cmd[1] = direction (0=CW, 1=CCW)                                           *
 *  cmd[2] = speed     (steps/sec = cmd[2] * 25;  0 = stop)                   *
 *  cmd[3] = steps     (pulses    = cmd[3] * 16;  0 = indefinite)             *
 * ─────────────────────────────────────────────────────────────────────────── */
static void parse_command(uint8_t *cmd)
{
    uint8_t  motor_id  = cmd[0];
    uint8_t  direction = cmd[1];
    uint32_t speed     = (uint32_t)cmd[2] * 25;
    uint32_t steps     = (uint32_t)cmd[3] * 16;

    if (motor_id < 1 || motor_id > NUM_MOTORS) return;

    if (speed == 0) {
        motor_stop(motor_id);
    } else {
        motor_move(motor_id, direction, speed, steps);
    }
}

/* ── Public API implementations ────────────────────────────────────────────── */

void stepper_init(void)
{
    /* Engage all brakes at startup */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_6, BRAKE_ENGAGE);

    /* Start byte-by-byte UART receive */
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void stepper_pulse_callback(TIM_HandleTypeDef *htim)
{
    for (int i = 0; i < NUM_MOTORS; i++) {
        Motor_t *m = &motors[i];
        if (m->htim->Instance == htim->Instance && m->running) {
            if (m->steps_remaining > 0) {
                m->steps_remaining--;
                if (m->steps_remaining == 0) {
                    HAL_TIM_PWM_Stop_IT(m->htim, m->tim_channel);
                    m->running = 0;
                    if (m->brake_port != NULL) {
                        HAL_GPIO_WritePin(m->brake_port, m->brake_pin, BRAKE_ENGAGE);
                    }
                }
            }
            /* steps_remaining == 0 from the start means run indefinitely */
        }
    }
}

void stepper_uart_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        cmd_buf[cmd_idx++] = rx_byte;
        if (cmd_idx >= CMD_LEN) {
            parse_command(cmd_buf);
            cmd_idx = 0;
        }
        HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
    }
}

void stepper_test(void)
{
    uint8_t cmd[4] = {2, 1, 50, 100}; // Motor1, CW, 800 steps/sec, 1 rev
    parse_command(cmd);
}

