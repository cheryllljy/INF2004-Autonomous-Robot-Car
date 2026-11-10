
/*
 * buddy1_mapping.c
 *
 * INF2004 Autonomous Robot Car
 * Buddy 1 - Runtime Barcode Movement Mapping
 *
 * Features:
 * - Decode four barcode-to-movement pairs
 * - Validate uppercase A-Z
 * - Reject duplicate characters and movements
 * - Store the latest validated MQTT mapping
 * - Provide race-free snapshots to VehicleController
 *
 * Synchronization uses a micro T-Kernel semaphore.
 * No barcode-to-movement assignments are hardcoded.
 */

#include "buddy1_mapping.h"
#include "../app.h"

/*
 * optional built-in mapping, for running without the
 * MQTT broker. 0 = none (the original design: the car only knows a
 * mapping once one arrives over MQTT, and until then every letter means
 * "go straight"). 1 = start with the table below. A mapping that arrives
 * over MQTT still replaces it.
 *
 * Same rules as the MQTT message: 4 different letters A-Z, and each of
 * the 4 movements used once. Change the letters / movements here.
 */
#define B1_DEFAULT_MAPPING_ENABLE 0

#if (B1_DEFAULT_MAPPING_ENABLE != 0)
static const B1_Mapping default_mapping = { {
    { 'A', B1_MOVE_LEFT },
    { 'B', B1_MOVE_RIGHT },
    { 'C', B1_MOVE_FORWARD },
    { 'D', B1_MOVE_UTURN },
} };
#endif

/* Private shared mapping state */
static B1_Mapping shared_mapping;
static ID mapping_sem = 0;
static int32_t mapping_valid = 0;


/*
 * Initialize the shared mapping store.
 *
 * Must be called from subsystem1_init()
 * before networking and controller tasks use it.
 *
 * Returns 0 on success, -1 on failure.
 */
int32_t b1_mapping_store_init(void)
{
    T_CSEM csem = { 0 };

    if (mapping_sem > 0)
    {
        return 0;
    }

    csem.sematr = TA_TFIFO;
    csem.isemcnt = 1;
    csem.maxsem = 1;

    mapping_sem = tk_cre_sem(&csem);

    if (mapping_sem <= 0)
    {
        mapping_sem = 0;
        return -1;
    }

    mapping_valid = 0;

#if (B1_DEFAULT_MAPPING_ENABLE != 0)
    /* same checks as an MQTT mapping. */
    if (b1_mapping_store_latest(&default_mapping) != 0)
    {
        return -1;
    }
#endif

    return 0;
}


/*
 * Decode an 8-byte MQTT mapping payload.
 *
 * Byte pairs:
 * [ASCII A-Z, movement 0-3] repeated four times.
 */
int32_t b1_mapping_decode(const uint8_t *payload, uint32_t length,
                          B1_Mapping *output)
{
    B1_Mapping temporary;
    uint32_t i;
    uint32_t j;

    if (0 == payload || 0 == output)
    {
        return -1;
    }

    if (B1_MAPPING_BYTES != length)
    {
        return -1;
    }

    for (i = 0; i < B1_MAPPING_COUNT; i++)
    {
        uint8_t character = payload[i * 2u];
        uint8_t movement = payload[i * 2u + 1u];

        if (character < (uint8_t)'A' || character > (uint8_t)'Z')
        {
            return -1;
        }

        if (movement > (uint8_t)B1_MOVE_UTURN)
        {
            return -1;
        }

        temporary.entries[i].character = (char)character;

        temporary.entries[i].movement = (B1_Movement)movement;
    }

    /* Ensure four different letters and movements. */
    for (i = 0; i < B1_MAPPING_COUNT; i++)
    {
        for (j = i + 1u; j < B1_MAPPING_COUNT; j++)
        {
            if (temporary.entries[i].character ==
                    temporary.entries[j].character ||
                temporary.entries[i].movement == temporary.entries[j].movement)
            {
                return -1;
            }
        }
    }

    /* Update output only if all checks pass. */
    *output = temporary;

    return 0;
}


/*
 * Look up one movement from a supplied mapping.
 *
 * Does not access shared state.
 */
int32_t b1_mapping_lookup(const B1_Mapping *mapping, char character,
                          B1_Movement *movement)
{
    uint32_t i;

    if (0 == mapping || 0 == movement)
    {
        return -1;
    }

    if (character < 'A' || character > 'Z')
    {
        return -1;
    }

    for (i = 0; i < B1_MAPPING_COUNT; i++)
    {
        if (mapping->entries[i].character == character)
        {
            *movement = mapping->entries[i].movement;
            return 0;
        }
    }

    return -1;
}


/*
 * Store a complete validated mapping.
 *
 * Called from Buddy 1's MQTT network task.
 *
 * Uses non-blocking semaphore acquisition:
 * if the store is busy, return -1 rather
 * than block the network-owning task.
 */
int32_t b1_mapping_store_latest(const B1_Mapping *mapping)
{
    uint32_t i;
    uint32_t j;

    if (0 == mapping || mapping_sem <= 0)
    {
        return -1;
    }

    /* Validate before acquiring the semaphore. */
    for (i = 0; i < B1_MAPPING_COUNT; i++)
    {
        if (mapping->entries[i].character < 'A' ||
            mapping->entries[i].character > 'Z' ||
            (uint32_t)mapping->entries[i].movement > (uint32_t)B1_MOVE_UTURN)
        {
            return -1;
        }

        for (j = i + 1u; j < B1_MAPPING_COUNT; j++)
        {
            if (mapping->entries[i].character ==
                    mapping->entries[j].character ||
                mapping->entries[i].movement == mapping->entries[j].movement)
            {
                return -1;
            }
        }
    }

    if (tk_wai_sem(mapping_sem, 1, TMO_POL) != E_OK)
    {
        return -1;
    }

    /* Protected whole-structure update. */
    shared_mapping = *mapping;
    mapping_valid = 1;

    tk_sig_sem(mapping_sem, 1);

    return 0;
}


/*
 * Retrieve a complete snapshot of the mapping.
 *
 * Returns:
 *  0 = mapping copied successfully
 * -1 = unavailable, invalid argument or busy
 *
 * Never exposes a pointer to shared state.
 */
int32_t b1_mapping_get_latest(B1_Mapping *output)
{
    if (0 == output || mapping_sem <= 0)
    {
        return -1;
    }

    if (tk_wai_sem(mapping_sem, 1, TMO_POL) != E_OK)
    {
        return -1;
    }

    if (!mapping_valid)
    {
        tk_sig_sem(mapping_sem, 1);
        return -1;
    }

    /* Copy the complete mapping while protected. */
    *output = shared_mapping;

    tk_sig_sem(mapping_sem, 1);

    return 0;
}


/*
 * VehicleController convenience interface.
 *
 * Retrieve a safe snapshot and look up the
 * movement for the scanned barcode.
 */
int32_t b1_mapping_get_movement(char character, B1_Movement *movement)
{
    B1_Mapping snapshot;

    if (0 == movement)
    {
        return -1;
    }

    if (b1_mapping_get_latest(&snapshot) != 0)
    {
        return -1;
    }

    return b1_mapping_lookup(&snapshot, character, movement);
}
