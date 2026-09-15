/**
 * @file    board_config.h
 * @brief   Pin map, task priorities and platform limits for the robot.
 *
 * Every board-dependent number used by the obstacle subsystem lives in
 * this file. A wiring change must never require editing a .c file.
 *
 * Target : Raspberry Pi Pico W on Cytron Robo Pico
 * RTOS   : micro T-Kernel 3.0
 * Style  : Barr-C:2018
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/*=====================================================================*/
/* GPIO assignment (Robo Pico datasheet Rev 1.0)                       */
/*=====================================================================*/

/*
 * Robo Pico allocation used by the whole team. Keep this comment in
 * sync with the team wiring sheet - it is the integration contract.
 *
 *   GP8  / GP9   M1A / M1B      motor driver      (Buddy 2)
 *   GP10 / GP11  M2A / M2B      motor driver      (Buddy 2)
 *   GP12..GP15   SERVO 1..4     servo headers     (Buddy 5 uses 4)
 *   GP4  / GP5   GROVE 3        I2C0              (Buddy 4, IMU)
 *   GP16 / GP17  GROVE 4        free digital pair (Buddy 5, sonar)
 *   GP26/27/28   GROVE 6, 7     ADC0..2           (Buddy 3, IR)
 *   GP18         NeoPixel, GP20/21 buttons, GP22 buzzer (on board)
 */

/** Servo signal pin. Robo Pico SERVO 4 header. */
#define BOARD_SERVO_GPIO            (15U)

/** HC-SR04 TRIG. Robo Pico GROVE 4, first pin. */
#define BOARD_SONAR_TRIG_GPIO       (16U)

/**
 * HC-SR04 ECHO. Robo Pico GROVE 4, second pin.
 *
 * WARNING: RP2040 GPIO is NOT 5 V tolerant (abs max 3.6 V). Use an
 * HC-SR04P / RCWL-1601 (3.3 V part) powered from the Grove 3V3 rail,
 * or a divider / level shifter on ECHO. See README section "Wiring".
 */
#define BOARD_SONAR_ECHO_GPIO       (17U)

/*=====================================================================*/
/* micro T-Kernel task priorities (1 = highest)                        */
/*=====================================================================*/

/*
 * One table for the whole team. Agree these in week 1, otherwise every
 * buddy picks itskpri = 10 and the schedule becomes undefined.
 *
 * Rule used here: shorter period and harder deadline => higher
 * priority (rate-monotonic ordering).
 */
#define BOARD_PRI_MOTION_CTRL       (7U)    /* 20 ms PID loop        */
#define BOARD_PRI_OBSTACLE          (8U)    /* 60 ms range guard     */
#define BOARD_PRI_LINE_FOLLOW       (9U)    /* 10..20 ms IR sampling */
#define BOARD_PRI_IMU               (10U)   /* 10 ms sampling        */
#define BOARD_PRI_VEHICLE_CTRL      (11U)   /* event driven          */
#define BOARD_PRI_TELEMETRY         (14U)   /* 200 ms, soft deadline */

/*=====================================================================*/
/* Chassis geometry (millimetres). Measure your own car and edit.      */
/*=====================================================================*/

/** Half the widest part of the chassis, wheels included. */
#define BOARD_HALF_WIDTH_MM         (75U)

/** Distance from the sonar face to the front bumper. */
#define BOARD_SONAR_TO_BUMPER_MM    (30U)

/** Nominal line-following speed, used for the latency budget. */
#define BOARD_CRUISE_SPEED_MM_S     (300U)

#endif /* BOARD_CONFIG_H */
