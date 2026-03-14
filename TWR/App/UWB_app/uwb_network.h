#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NETWORK_MAX_PEERS 7

typedef struct {
    uint16_t id;
    uint8_t  active;
} peer_t;

typedef struct {
    peer_t   peers[NETWORK_MAX_PEERS];
    uint8_t  count;
    uint16_t master_id;
    uint16_t own_id;
    bool     acknowledged;
} network_t;

void          network_init       (uint16_t own_id);
void          network_add_peer   (uint16_t id);
void          network_remove_peer(uint16_t id);
const peer_t *network_get_peers  (uint8_t *out_count);
void          network_set_master (uint16_t id);
uint16_t      network_get_master (void);
int           network_is_master  (void);
uint16_t      network_get_ownid  (void);
uint8_t network_get_count(void);
uint8_t network_fill_peer_ids(uint16_t *out_ids, uint8_t max_count);


void network_set_acknowledged(bool state);
bool network_is_acknowledged(void);
void network_update_peers_from_sync(uint16_t master_id, const uint16_t *peer_ids, uint8_t peer_count);
