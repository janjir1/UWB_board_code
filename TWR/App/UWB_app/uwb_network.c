#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"

#include "../Generic/my_print.h"
#include "DWM3000_setup.h"
#include "DWM3000_driver.h"

#include "../UWB_app/uwb_network.h"

static network_t net;

void network_init(uint16_t own_id)
{
    memset(&net, 0, sizeof(net));
    net.own_id = own_id;
    net.acknowledged = false;
}

void network_add_peer(uint16_t id)
{
    for (int i = 0; i < NETWORK_MAX_PEERS; i++)
        if (net.peers[i].active && net.peers[i].id == id) return;

    for (int i = 0; i < NETWORK_MAX_PEERS; i++) {
        if (!net.peers[i].active) {
            net.peers[i].id     = id;
            net.peers[i].active = 1;
            net.count++;
            return;
        }
    }
}

void network_remove_peer(uint16_t id)
{
    for (int i = 0; i < NETWORK_MAX_PEERS; i++) {
        if (net.peers[i].active && net.peers[i].id == id) {
            net.peers[i].active = 0;
            net.peers[i].id     = 0;
            net.count--;
            return;
        }
    }
}

const peer_t *network_get_peers(uint8_t *out_count)
{
    *out_count = net.count;
    return net.peers;
}

void network_set_master(uint16_t id)
{
    net.master_id = id;
}

uint16_t network_get_master(void)
{
    return net.master_id;
}

uint16_t network_get_ownid(void)
{
    return net.own_id;
}

uint8_t network_get_count(void)
{
    return net.count;
}


int network_is_master(void)
{
    return net.master_id == net.own_id;
}

void network_set_acknowledged(bool state)
{
    net.acknowledged = state;
}

bool network_is_acknowledged(void)
{
    return net.acknowledged;
}

uint8_t network_fill_peer_ids(uint16_t *out_ids, uint8_t max_count)
{
    uint8_t count = 0;
    for (int i = 0; i < net.count && count < max_count; i++) {
        if (net.peers[i].active && net.peers[i].id != net.own_id) {
            out_ids[count++] = net.peers[i].id;
        }
    }
    return count;
}

void network_update_peers_from_sync(uint16_t master_id, const uint16_t *peer_ids, uint8_t peer_count)
{
    // Clear existing peer list — master is the only truth
    memset(net.peers, 0, sizeof(net.peers));
    net.count = 0;

    // Add master itself as first peer
    net.peers[net.count].id     = master_id;
    net.peers[net.count].active = 1;
    net.count++;

    // Add all peers from sync message
    for (int i = 0; i < peer_count && net.count < NETWORK_MAX_PEERS; i++) {
        net.peers[net.count].id     = peer_ids[i];
        net.peers[net.count].active = 1;
        net.count++;
    }
}