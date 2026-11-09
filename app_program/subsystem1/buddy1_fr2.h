
#ifndef BUDDY1_FR2_H
#define BUDDY1_FR2_H

#include <stdint.h>

/*
 * Buddy 1 - FR2 MQTT command protocol.
 *
 * Provisional binary wire format:
 *
 * Byte 0: protocol version (1)
 * Byte 1: command ID
 * Byte 2: sequence number
 *
 * This is separate from the 8-byte
 * barcode movement mapping payload.
 */

#define B1_FR2_VERSION       1u
#define B1_FR2_MESSAGE_BYTES 3u
#define B1_FR2_QUEUE_SIZE    4u

#define B1_FR2_START 1u
#define B1_FR2_STOP  2u
#define B1_FR2_RESET 3u
/* nyx, 2026-11-08: start with a line search (robot.ps1 search): drive
 * straight until the IRs meet a line, line up along it, then follow it */
#define B1_FR2_SEARCH 4u

typedef struct
{
    uint8_t command;
    uint8_t sequence;
} B1_FR2_Command;

/*
 * Called from one consumer task:
 * VehicleController.
 *
 * Returns:
 *  0: one validated command retrieved
 * -1: queue empty or invalid output pointer
 *
 * This does not call the MQTT/lwIP stack.
 */
int32_t b1_fr2_pop(B1_FR2_Command *output);

/*
 * Number of commands waiting for the controller.
 * A snapshot only; the value may change.
 */
uint32_t b1_fr2_pending(void);

#endif
