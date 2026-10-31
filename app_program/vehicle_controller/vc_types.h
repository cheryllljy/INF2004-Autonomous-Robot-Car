/*
 *	vc_types.h
 *	VehicleController -- shared state, command and fault representation.
 *
 *	Sized for Change Requirement 1: every field uses only the bits its
 *	valid range needs, and the static asserts below fail the build if a
 *	new value no longer fits.  Change this file only by team agreement.
 *
 *	Field         Valid range                 Bits  Used by
 *	------------  --------------------------  ----  -------------------------
 *	state         vc_state_t, 0..5            3     B1 telemetry controller_state
 *	command       vc_cmd_t, 0..7              3     B1 command byte (IDs 0..7)
 *	fault causes  one bit per source          8     flg_vc bits 0..7,
 *	                                                B1 telemetry fault_flags
 *	pending turn  b3_nav_command_t, 0..4      3     turn waiting for the next junction
 *	symbol        Code 39 character, ASCII    7     Buddy 3 navigation event
 *	nav sequence  wraps at 256                8     detects a missed barcode
 *	junctions     wraps at 8                  3     detects a new junction
 *
 *	All of it travels between tasks as ONE 32-bit status word (layout at
 *	VC_SW_*), built in a local and published with one aligned store, so
 *	readers never see half an update (CR1 race-safety; the M0+ has no
 *	LDREX/STREX, so no read-modify-write is ever done on it).
 */

#ifndef VC_TYPES_H
#define VC_TYPES_H

#include <stdint.h>
#include "../subsystem1/buddy1_cr1.h"	/* B1_FAULT_* bit positions */
#include "../subsystem3/buddy3.h"	/* b3_nav_command_t */

/* ---------------------------------------------------------------------
 *  Controller state (3 bits)
 * --------------------------------------------------------------------- */
typedef enum {
	VC_STATE_IDLE = 0,	/* stopped, waiting for START */
	VC_STATE_LINE_FOLLOW,	/* Buddy 3 steers, Buddy 2 drives */
	VC_STATE_TURNING,	/* executing a barcode command */
	VC_STATE_OBSTACLE,	/* stopped for Buddy 5's scan / bypass */
	VC_STATE_STOPPED,	/* STOP command; START resumes */
	VC_STATE_ESTOP,		/* latched fault; only RESET leaves it */
	VC_STATE_COUNT
} vc_state_t;
#define VC_STATE_BITS		3U

/* ---------------------------------------------------------------------
 *  Remote commands (3 bits), from Buddy 1's MQTT handler after
 *  b1_command_unpack().  Buddy 3 only reports the barcode LETTER; the
 *  MQTT side maps the letter to one of the turn commands below and sends
 *  it back.  The VehicleController keeps it until the next junction.
 *  All 8 IDs are used: a new command needs a 4th bit.
 * --------------------------------------------------------------------- */
typedef enum {
	VC_CMD_NONE = 0,
	VC_CMD_START,
	VC_CMD_STOP,
	VC_CMD_RESET,		/* clear latched faults, back to IDLE */
	VC_CMD_TURN_LEFT,	/* at the next junction */
	VC_CMD_TURN_RIGHT,
	VC_CMD_GO_STRAIGHT,
	VC_CMD_U_TURN,
	VC_CMD_COUNT
} vc_cmd_t;
#define VC_CMD_BITS		3U

/* ---------------------------------------------------------------------
 *  Fault cause bits (8 bits).  Latched in flg_vc; any task or ISR sets
 *  its own bit; only the VehicleController clears them, on RESET.
 *  Bits 0..4 reuse Buddy 1's provisional B1_FAULT_* positions so the
 *  telemetry fault_flags field needs no translation.
 * --------------------------------------------------------------------- */
#define VC_FAULT_MOTOR_STALL	B1_FAULT_MOTOR_STALL	/* Buddy 2 */
#define VC_FAULT_ENCODER	B1_FAULT_ENCODER	/* Buddy 2 */
#define VC_FAULT_IMU		B1_FAULT_IMU		/* Buddy 4 */
#define VC_FAULT_NO_ECHO	B1_FAULT_NO_ECHO	/* Buddy 5 (sets it itself) */
#define VC_FAULT_WIFI_LOSS	B1_FAULT_WIFI_LOSS	/* Buddy 1 */
#define VC_FAULT_LINE		(1U << 5)	/* Buddy 3: task error, event queue full */
#define VC_FAULT_SPARE		(1U << 6)	/* free for the team */
#define VC_FAULT_TASK		(1U << 7)	/* VehicleController / kernel call */
#define VC_FAULT_BITS		8U
#define VC_FAULT_MASK		((1U << VC_FAULT_BITS) - 1U)

#define VC_NAV_BITS		3U
#define VC_SYMBOL_BITS		7U
#define VC_SEQ_BITS		8U
#define VC_JUNCTION_BITS	3U

/* ---------------------------------------------------------------------
 *  Status word, least-significant bit first
 *	bits  0..2   state
 *	bits  3..10  fault causes
 *	bits 11..13  pending turn (b3_nav_command_t)
 *	bits 14..20  last barcode symbol (ASCII, 0 = none yet)
 *	bits 21..28  last barcode sequence (wraps)
 *	bits 29..31  junctions reached (wraps)
 * --------------------------------------------------------------------- */
#define VC_SW_STATE_SHIFT	0U
#define VC_SW_FAULT_SHIFT	(VC_SW_STATE_SHIFT + VC_STATE_BITS)
#define VC_SW_NAV_SHIFT		(VC_SW_FAULT_SHIFT + VC_FAULT_BITS)
#define VC_SW_SYMBOL_SHIFT	(VC_SW_NAV_SHIFT + VC_NAV_BITS)
#define VC_SW_SEQ_SHIFT		(VC_SW_SYMBOL_SHIFT + VC_SYMBOL_BITS)
#define VC_SW_JUNCTION_SHIFT	(VC_SW_SEQ_SHIFT + VC_SEQ_BITS)
#define VC_SW_USED_BITS		(VC_SW_JUNCTION_SHIFT + VC_JUNCTION_BITS)

#define VC_FIELD(word, shift, bits) \
	(((word) >> (shift)) & ((1UL << (bits)) - 1UL))

static inline uint32_t vc_status_pack(vc_state_t state, uint32_t faults,
				      b3_nav_command_t nav, char symbol,
				      uint32_t sequence, uint32_t junctions)
{
	return ((uint32_t)state << VC_SW_STATE_SHIFT) |
	       ((faults & VC_FAULT_MASK) << VC_SW_FAULT_SHIFT) |
	       ((uint32_t)nav << VC_SW_NAV_SHIFT) |
	       (((uint32_t)(uint8_t)symbol & 0x7FU) << VC_SW_SYMBOL_SHIFT) |
	       ((sequence & ((1U << VC_SEQ_BITS) - 1U)) << VC_SW_SEQ_SHIFT) |
	       ((junctions & ((1U << VC_JUNCTION_BITS) - 1U)) << VC_SW_JUNCTION_SHIFT);
}

#define VC_SW_STATE(w)	((vc_state_t)VC_FIELD((w), VC_SW_STATE_SHIFT, VC_STATE_BITS))
#define VC_SW_FAULTS(w)	((uint8_t)VC_FIELD((w), VC_SW_FAULT_SHIFT, VC_FAULT_BITS))
#define VC_SW_NAV(w)	((b3_nav_command_t)VC_FIELD((w), VC_SW_NAV_SHIFT, VC_NAV_BITS))
#define VC_SW_SYMBOL(w)	((char)VC_FIELD((w), VC_SW_SYMBOL_SHIFT, VC_SYMBOL_BITS))
#define VC_SW_SEQ(w)	((uint8_t)VC_FIELD((w), VC_SW_SEQ_SHIFT, VC_SEQ_BITS))
#define VC_SW_JUNCTIONS(w) ((uint8_t)VC_FIELD((w), VC_SW_JUNCTION_SHIFT, VC_JUNCTION_BITS))

/* ---------------------------------------------------------------------
 *  CR1 checks: adding a state, command or cause that no longer fits its
 *  bits stops the build instead of silently corrupting the packing.
 * --------------------------------------------------------------------- */
_Static_assert(VC_STATE_COUNT <= (1U << VC_STATE_BITS),
	       "vc_state_t no longer fits 3 bits (B1 controller_state)");
_Static_assert(VC_CMD_COUNT <= (1U << VC_CMD_BITS),
	       "vc_cmd_t no longer fits 3 bits (B1 command byte)");
_Static_assert(B3_NAV_UTURN < (1U << VC_NAV_BITS),
	       "b3_nav_command_t no longer fits 3 bits");
_Static_assert((VC_FAULT_MOTOR_STALL + VC_FAULT_ENCODER + VC_FAULT_IMU +
		VC_FAULT_NO_ECHO + VC_FAULT_WIFI_LOSS + VC_FAULT_LINE +
		VC_FAULT_SPARE + VC_FAULT_TASK) == VC_FAULT_MASK,
	       "fault cause bits must be distinct and fill 8 bits");
_Static_assert(VC_SW_USED_BITS <= 32U, "status word overflow");

#endif /* VC_TYPES_H */
