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
#define TIMER_CLOCK_HZ    1000000UL // 1MHz after prescaler (PSC=83, all timers 84MHz input)

#define DIR_CW            GPIO_PIN_SET
#define DIR_CCW           GPIO_PIN_RESET

/* Brake polarity:
 *   Power-off brake = engaged when coil is unpowered.
 *   GPIO HIGH -> transistor ON  -> 24V to coil -> BRAKE RELEASED
 *   GPIO LOW  -> transistor OFF -> coil off    -> BRAKE ENGAGED        */
#define BRAKE_RELEASE     GPIO_PIN_SET
#define BRAKE_ENGAGE      GPIO_PIN_RESET

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
 * Index 0 = Motor ID 1 (BottomLeft,  no brake)  TIM2_CH1  PUL=PA0  DIR=PC8  *
 * Index 1 = Motor ID 2 (TopRight,   has brake)  TIM4_CH1  PUL=PB6  DIR=PA10 *
 * Index 2 = Motor ID 3 (BottomRight, no brake)  TIM1_CH2  PUL=PA9  DIR=PC4  *
 * Index 3 = Motor ID 4 (TopLeft,    has brake)  TIM3_CH4  PUL=PB1  DIR=PC6  *
 *                                                                             *
 * Wiring: PUL+/DIR+ tied to GND, PUL-/DIR- driven by STM32 (3.3V swing)     *
 * ─────────────────────────────────────────────────────────────────────────── */
static Motor_t motors[NUM_MOTORS] = {
    { &htim2, TIM_CHANNEL_1, GPIOC, GPIO_PIN_8,  NULL,  0,          0, 0 },
    { &htim4, TIM_CHANNEL_1, GPIOA, GPIO_PIN_10, GPIOA, GPIO_PIN_6, 0, 0 },
    { &htim1, TIM_CHANNEL_2, GPIOC, GPIO_PIN_4,  NULL,  0,          0, 0 },
    { &htim3, TIM_CHANNEL_4, GPIOC, GPIO_PIN_6,  GPIOA, GPIO_PIN_5, 0, 0 },
};

/* ── Packet encoding helpers ───────────────────────────────────────────────── */
#define SPD(x)   ((uint16_t)((x) / 10))   // encode steps/sec for packet
#define HI(x)    ((uint8_t)((x) >> 8))    // high byte of uint16
#define LO(x)    ((uint8_t)((x) & 0xFF))  // low byte of uint16

/* ── UART packet format ────────────────────────────────────────────────────── *
 *  buf[0]          = num_motors (1-4)                                         *
 *  per motor i (6 bytes each):                                                *
 *    [0]  motor_id          (1-4)                                             *
 *    [1]  direction         (0=CW, 1=CCW)                                     *
 *    [2]  speed high byte   \  uint16, steps/sec = combined * 10              *
 *    [3]  speed low byte    /  0 = stop motor                                 *
 *    [4]  steps high byte   \  uint16, raw pulses                             *
 *    [5]  steps low byte    /  0 = run indefinitely                           *
 *                                                                             *
 *  Total packet size = 1 + num_motors * 6                                     *
 *  Max speed:  65535 * 10 = 655350 steps/sec                                  *
 *  Max steps:  65535 pulses = ~41 revs at 1600 pulses/rev                     *
 * ─────────────────────────────────────────────────────────────────────────── */
#define MAX_MOTORS_PER_CMD  4
#define SINGLE_CMD_LEN      6

static uint8_t rx_byte;
static uint8_t cmd_buf[1 + MAX_MOTORS_PER_CMD * SINGLE_CMD_LEN];
static uint8_t cmd_idx     = 0;
static uint8_t expected_len = 0;

/* ── Private functions ─────────────────────────────────────────────────────── */

static void motor_set_speed(Motor_t *m, uint32_t steps_per_sec)
{
    if (steps_per_sec == 0) return;

    uint32_t period = (TIMER_CLOCK_HZ / steps_per_sec) - 1;
    if (period < 3)     period = 3;
    if (period > 65535) period = 65535;

    __HAL_TIM_SET_AUTORELOAD(m->htim, period);
    __HAL_TIM_SET_COMPARE(m->htim, m->tim_channel, period / 2);
}

static void motor_stop_internal(Motor_t *m)
{
    HAL_TIM_PWM_Stop(m->htim, m->tim_channel);
    HAL_TIM_Base_Stop_IT(m->htim);
    m->running = 0;
    m->steps_remaining = 0;

    if (m->brake_port != NULL)
        HAL_GPIO_WritePin(m->brake_port, m->brake_pin, BRAKE_ENGAGE);
}

static void parse_multi_command(uint8_t *buf, uint8_t num)
{
    // Pass 1: stop only commanded motors and release their brakes
    for (int i = 0; i < num; i++) {
        uint8_t *c = &buf[1 + i * SINGLE_CMD_LEN];
        uint8_t motor_id = c[0];
        if (motor_id < 1 || motor_id > NUM_MOTORS) continue;
        Motor_t *m = &motors[motor_id - 1];
        motor_stop_internal(m);
        if (m->brake_port != NULL)
            HAL_GPIO_WritePin(m->brake_port, m->brake_pin, BRAKE_RELEASE);
    }

    HAL_Delay(BRAKE_RELEASE_DELAY_MS);

    // Pass 2: configure + start all commanded motors simultaneously
    for (int i = 0; i < num; i++) {
        uint8_t *c = &buf[1 + i * SINGLE_CMD_LEN];
        uint8_t  motor_id  = c[0];
        uint8_t  direction = c[1];
        uint32_t speed     = ((uint32_t)c[2] << 8 | c[3]) * 10;  // steps/sec
        uint32_t steps     = ((uint32_t)c[4] << 8 | c[5]);        // raw pulses, 0 = indefinite
        if (motor_id < 1 || motor_id > NUM_MOTORS) continue;

        Motor_t *m = &motors[motor_id - 1];

        if (speed == 0) {
            motor_stop_internal(m);
            continue;
        }

        HAL_GPIO_WritePin(m->dir_port, m->dir_pin, direction ? DIR_CCW : DIR_CW);
        m->steps_remaining = steps;
        m->running = 1;

        HAL_TIM_PWM_Start(m->htim, m->tim_channel);
        __HAL_TIM_ENABLE_IT(m->htim, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(m->htim);

        motor_set_speed(m, speed);
    }
}

/* ── Public API ────────────────────────────────────────────────────────────── */

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
                    if (m->brake_port != NULL)
                        HAL_GPIO_WritePin(m->brake_port, m->brake_pin, BRAKE_ENGAGE);
                }
            }
            /* steps_remaining == 0 from the start means run indefinitely */
        }
    }
}

void stepper_uart_callback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) return;

    cmd_buf[cmd_idx++] = rx_byte;

    if (cmd_idx == 1) {
        uint8_t num = cmd_buf[0];
        if (num < 1 || num > MAX_MOTORS_PER_CMD) {
            cmd_idx = 0;
        } else {
            expected_len = 1 + num * SINGLE_CMD_LEN;
        }
    } else if (cmd_idx == expected_len) {
        parse_multi_command(cmd_buf, cmd_buf[0]);
        cmd_idx = 0;
        expected_len = 0;
    }

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* ── Test functions ────────────────────────────────────────────────────────── *
 * Speed encoding:  uint16 = steps_per_sec / 10  (e.g. 800 steps/sec = 0x0050)*
 * Steps encoding:  uint16 = raw pulses           (e.g. 1600 pulses   = 0x0640)*
 * ─────────────────────────────────────────────────────────────────────────── */

void stepper_test(void)
{
    // Motor 3: CW, 800 steps/sec, 1600 pulses (1 rev)
    uint8_t buf[] = {1,
        2, 0, HI(SPD(2000)), LO(SPD(2000)), HI(1600), LO(1600),
    };
    parse_multi_command(buf, buf[0]);
}

void stepper_test2(void)
{
    // Motor 3: CW, 200 steps/sec, 3200 pulses (2 revs)
    uint8_t buf[] = {1,
        3, 0, HI(SPD(200)), LO(SPD(200)), HI(3200), LO(3200),
    };
    parse_multi_command(buf, buf[0]);
}

void stepper_test3(void)
{
    // Motor 3: CW, 20000 steps/sec, indefinite
    uint16_t speed = SPD(800);
    uint16_t steps = 1600;
    uint8_t buf[] = {1,
        3, 0, HI(speed), LO(speed), HI(steps), LO(steps),
    };
    parse_multi_command(buf, buf[0]);
}
