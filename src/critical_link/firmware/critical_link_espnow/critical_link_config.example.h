#pragma once

// Set to 1 on the Ground-PC USB adapter and 0 on the vessel/miniPC adapter.
#define CRITICAL_LINK_ROLE_GROUND 1

#define CRITICAL_LINK_ESPNOW_CHANNEL 1
#define CRITICAL_LINK_SERIAL_BAUD 921600
#define CRITICAL_LINK_RETRY_INTERVAL_MS 15
#define CRITICAL_LINK_MAX_ATTEMPTS 3

// Replace both keys before deployment. Each value must contain exactly 16 bytes.
static constexpr uint8_t CRITICAL_LINK_PMK[16] = {
  'c', 'h', 'a', 'n', 'g', 'e', '-', 'p', 'm', 'k', '-', '1', '2', '3', '4', '5'
};

// The Ground gateway has one vessel peer.  The vessel gateway lists every
// permitted Ground gateway.  On the vessel, source_id must match the source
// ID encoded in each received protocol-v2 frame.  source_id is ignored for
// the vessel peer configured on the Ground side.
struct CriticalLinkPeerConfig
{
  uint8_t mac[6];
  uint8_t lmk[16];
  uint32_t source_id;
};

static constexpr CriticalLinkPeerConfig CRITICAL_LINK_PEERS[] = {
  {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {'c', 'h', 'a', 'n', 'g', 'e', '-', 'l', 'm', 'k', '-', '1', '2', '3', '4', '5'},
    100,
  },
};
constexpr size_t CRITICAL_LINK_PEER_COUNT =
  sizeof(CRITICAL_LINK_PEERS) / sizeof(CRITICAL_LINK_PEERS[0]);

// Set this to the ROS sender's source_id on Ground gateways.  It is unused on
// the vessel gateway, where each configured peer owns its own source_id.
#define CRITICAL_LINK_SOURCE_ID 100
