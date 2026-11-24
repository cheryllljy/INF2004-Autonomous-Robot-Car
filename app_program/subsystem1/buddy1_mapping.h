
/*
 * buddy1_mapping.h
 *
 * INF2004 Autonomous Robot Car
 * Buddy 1 - Runtime Barcode Movement Mapping
 *
 * Responsibilities:
 * - Decode mappings received through MQTT
 * - Validate four barcode-to-movement pairs
 * - Provide mappings to VehicleController
 * - Support safe cross-task communication
 */

#ifndef BUDDY1_MAPPING_H
#define BUDDY1_MAPPING_H

#include <stdint.h>

/* Four mappings, two bytes per mapping. */
#define B1_MAPPING_COUNT 4u
#define B1_MAPPING_BYTES 8u

/*
 * Movement identifiers shared with VehicleController.
 */
typedef enum
{
    B1_MOVE_FORWARD = 0,
    B1_MOVE_LEFT = 1,
    B1_MOVE_RIGHT = 2,
    B1_MOVE_UTURN = 3
} B1_Movement;

/*
 * One barcode character and its movement.
 */
typedef struct
{
    char character;
    B1_Movement movement;
} B1_MappingEntry;

/*
 * Four runtime barcode-to-movement pairs.
 */
typedef struct
{
    B1_MappingEntry entries[B1_MAPPING_COUNT];
} B1_Mapping;

/*
 * Decode and validate a received MQTT payload.
 *
 * Payload:
 * [character, movement] repeated four times
 *
 * Returns:
 *  0 = valid mapping
 * -1 = invalid mapping
 */
int32_t b1_mapping_decode(const uint8_t *payload, uint32_t length,
                          B1_Mapping *output);

/*
 * Find a character's movement in a mapping.
 *
 * Returns:
 *  0 = mapping found
 * -1 = character not mapped or invalid input
 */
int32_t b1_mapping_lookup(const B1_Mapping *mapping, char character,
                          B1_Movement *movement);

/*
 * Publish a validated mapping into the shared
 * mapping store.
 *
 * Called by Buddy 1 after receiving MQTT data.
 *
 * Returns:
 *  0 = mapping stored
 * -1 = invalid input or store unavailable
 *
 * Implementation must synchronize access
 * between the MQTT task and VehicleController.
 */
int32_t b1_mapping_store_latest(const B1_Mapping *mapping);

/*
 * Retrieve a snapshot of the latest mapping.
 *
 * Called by VehicleController.
 *
 * Returns:
 *  0 = valid mapping copied into output
 * -1 = no mapping available or invalid input
 *
 * The returned data is a private copy.
 * VehicleController must not access internal
 * MQTT storage directly.
 */
int32_t b1_mapping_get_latest(B1_Mapping *output);

/*
 * Look up a movement using the latest
 * received and stored mapping.
 *
 * Returns:
 *  0 = movement found
 * -1 = mapping unavailable or character
 *      not present in the mapping
 */
int32_t b1_mapping_get_movement(char character, B1_Movement *movement);

/*
 * Initialize the shared mapping semaphore.
 * Called once during subsystem initialization.
 */
int32_t b1_mapping_store_init(void);

#endif /* BUDDY1_MAPPING_H */
