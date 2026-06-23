#include "MyMesh.h"
#include <algorithm>

/* ------------------------------ Config -------------------------------- */

#ifndef LORA_FREQ
  #define LORA_FREQ 915.0
#endif
#ifndef LORA_BW
  #define LORA_BW 250
#endif
#ifndef LORA_SF
  #define LORA_SF 10
#endif
#ifndef LORA_CR
  #define LORA_CR 5
#endif
#ifndef LORA_TX_POWER
  #define LORA_TX_POWER 20
#endif

#ifndef ADVERT_NAME
  #define ADVERT_NAME "repeater"
#endif
#ifndef ADVERT_LAT
  #define ADVERT_LAT 0.0
#endif
#ifndef ADVERT_LON
  #define ADVERT_LON 0.0
#endif

#ifndef ADMIN_PASSWORD
  #define ADMIN_PASSWORD "password"
#endif

#ifndef SERVER_RESPONSE_DELAY
  #define SERVER_RESPONSE_DELAY 300
#endif

#ifndef TXT_ACK_DELAY
  #define TXT_ACK_DELAY 200
#endif

#define FIRMWARE_VER_LEVEL       2

// ── lusofw feature thresholds (all overridable via build flags) ──────────────
#ifndef TEMP_HIGH_C
  #define TEMP_HIGH_C          72.0f
  #define TEMP_NORMAL_C        62.0f
  #define TEMP_TX_REDUCE_DBM   4
#endif
#ifndef TPC_SNR_HIGH
  #define TPC_SNR_HIGH         15.0f
  #define TPC_SNR_LOW           5.0f
  #define TPC_REDUCE_DBM        3
#endif
#ifndef ISOLATION_SILENCE_HOURS
  #define ISOLATION_SILENCE_HOURS      8
  #define ISOLATION_ADVERT_COOLDOWN_MS (4UL * 3600000UL)
  #define ISOLATION_MIN_UPTIME_SECS    7200UL
#endif
#ifndef WEEKLY_REBOOT_HOUR
  #define WEEKLY_REBOOT_HOUR              3
  #define WEEKLY_REBOOT_MIN              30
  #define WEEKLY_REBOOT_MIN_UPTIME_SECS  (6UL * 24 * 3600)
#endif
// ─────────────────────────────────────────────────────────────────────────────

#define REQ_TYPE_GET_STATUS         0x01 // same as _GET_STATS
#define REQ_TYPE_KEEP_ALIVE         0x02
#define REQ_TYPE_GET_TELEMETRY_DATA 0x03
#define REQ_TYPE_GET_ACCESS_LIST    0x05
#define REQ_TYPE_GET_NEIGHBOURS     0x06
#define REQ_TYPE_GET_OWNER_INFO     0x07     // FIRMWARE_VER_LEVEL >= 2

#define RESP_SERVER_LOGIN_OK        0 // response to ANON_REQ

#define ANON_REQ_TYPE_REGIONS      0x01
#define ANON_REQ_TYPE_OWNER        0x02
#define ANON_REQ_TYPE_BASIC        0x03   // just remote clock

#define CLI_REPLY_DELAY_MILLIS      600

#define LAZY_CONTACTS_WRITE_DELAY    5000

#define NEIGHBOUR_EXPIRATION_SECS    (60 * 60 * 24 * 2) // 2 days

#define ADVERTS_ALLOWED_START        2 // hours >=
#define ADVERTS_ALLOWED_END          7 // hours <= (02:00-07:59 UTC)
#define ADVERTS_ALLOWED_COUNT        1

void MyMesh::putNeighbour(const mesh::Identity &id, uint32_t timestamp, float snr) {
#if MAX_NEIGHBOURS // check if neighbours enabled
  uint32_t now = getRTCClock()->getCurrentTime();
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    // Cleanup old neighbours
    if (now >= neighbours[i].heard_timestamp && now - neighbours[i].heard_timestamp > NEIGHBOUR_EXPIRATION_SECS) {
      memset(&neighbours[i], 0, sizeof(NeighbourInfo));
    }
  }

  // find existing neighbour, else use least recently updated
  uint32_t oldest_timestamp = 0xFFFFFFFF;
  NeighbourInfo *neighbour = &neighbours[0];
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    // if neighbour already known, we should update it
    if (id.matches(neighbours[i].id)) {
      neighbour = &neighbours[i];
      break;
    }

    // otherwise we should update the least recently updated neighbour
    if (neighbours[i].heard_timestamp < oldest_timestamp) {
      neighbour = &neighbours[i];
      oldest_timestamp = neighbour->heard_timestamp;
    }
  }

  // update neighbour info — shift SNR history before overwriting
  neighbour->id = id;
  neighbour->advert_timestamp = timestamp;
  neighbour->heard_timestamp = getRTCClock()->getCurrentTime();
  neighbour->snr_h2 = neighbour->snr_h1;
  neighbour->snr_h1 = neighbour->snr;
  neighbour->snr = (int8_t)(snr * 4);
#endif
}

#ifdef ENABLE_CONSENSUS_TIME_SYNC
/**
 * Apply time consensus algorithm to synchronize local clock with mesh peers.
 * 
 * This function collects time offset samples from neighboring peers (recorded
 * when receiving their advertisements) and computes a consensus adjustment.
 * 
 * Algorithm overview:
 * 1. Collect valid time offset samples from time_samples[] array
 * 2. Sort offsets to enable outlier removal
 * 3. Calculate trimmed mean (discard 25% from each end) to reject outliers
 * 4. Apply adjustment with different rules for initial vs maintenance sync
 * 
 * Two sync modes:
 * - INITIAL SYNC: When local clock is before 2026-01-01 (1767225600), the RTC
 *   has not been set. Allow unlimited forward adjustment to catch up with the
 *   network, but never go backward.
 * - MAINTENANCE SYNC: Clock is valid, apply symmetric ±60 second limits to
 *   prevent large jumps while allowing correction of both fast and slow drift.
 * 
 * The trimmed mean approach (vs simple median) provides better noise rejection
 * when we have multiple samples, as it averages the central values while
 * discarding extreme outliers on both ends.
 * 
 * Time samples are collected in onAdvertRecv() from peer advertisements.
 * Each sample represents: peer_timestamp - local_timestamp at time of receipt.
 */
void MyMesh::applyTimeConsensus() {
  uint32_t now = getRTCClock()->getCurrentTime();
  
  // Determine sync mode based on whether clock has been set to a reasonable time
  // 1767225600 = 2026-01-01 00:00:00 UTC (matches filter in onAdvertRecv)
  bool is_initial_sync = (now < 1767225600);
  
  int32_t valid_offsets[TIME_SYNC_SAMPLES];
  int valid_count = 0;
  
  // Collect valid time offset samples (offset > 0 means peer is ahead of us)
  for (int i = 0; i < TIME_SYNC_SAMPLES; i++) {
    if (time_samples[i].sampled_at > 0) {
      valid_offsets[valid_count++] = time_samples[i].offset;
    }
  }
  
  // Minimum sample requirement depends on sync mode:
  // - Initial sync: 5 samples (sync faster to catch up with network)
  // - Maintenance sync: full TIME_SYNC_SAMPLES for more reliable consensus
  int min_samples = is_initial_sync ? 5 : TIME_SYNC_SAMPLES;
  if (valid_count < min_samples) return;

  // Sort offsets using std::sort (consistent with rest of codebase, O(n log n))
  std::sort(valid_offsets, valid_offsets + valid_count, [](int32_t a, int32_t b) { return a < b; });
  
  // Calculate trimmed mean: discard 25% from each end to remove outliers
  // With 8 samples: trim=2, use indices [2,3,4,5] = middle 4 samples
  // With 5 samples: trim=1, use indices [1,2,3] = middle 3 samples
  int trim = valid_count / 4;
  int32_t sum = 0;
  int trimmed_count = 0;
  for (int i = trim; i < valid_count - trim; i++) {
    sum += valid_offsets[i];
    trimmed_count++;
  }
  
  int32_t consensus = sum / trimmed_count;
  int32_t adjustment = consensus;
  
  if (is_initial_sync) {
    // INITIAL SYNC: Clock likely unset (still at default ~2024)
    // Allow large forward jump to sync with network
    // Never go backward - we might have partial time from a previous sync
    if (consensus <= 0) {
      adjustment = 0;
    }
    // Positive adjustments are uncapped - we need to catch up potentially years
  } else {
    // MAINTENANCE SYNC: Clock is valid, apply conservative limits
    // Cap adjustments to ±60 seconds to prevent large jumps from
    // malicious peers or temporary network issues
    if (adjustment > 60) adjustment = 60;
    if (adjustment < -60) adjustment = -60;
  }
  
  // Apply adjustment if it exceeds the deadband threshold (±5 seconds)
  // Small adjustments are ignored to prevent clock thrashing
  if (adjustment != 0 && (adjustment > 5 || adjustment < -5)) {
    getRTCClock()->setCurrentTime(now + adjustment);

    // Clear all samples to force fresh collection - old offsets are now stale
    memset(time_samples, 0, sizeof(time_samples));
    time_sample_idx = 0;
    
    MESH_DEBUG_PRINTLN("Time sync: adjusted %d sec (consensus=%d, samples=%d, initial=%d)", 
                       adjustment, consensus, valid_count, is_initial_sync);
  }
}
#endif

uint8_t MyMesh::handleLoginReq(const mesh::Identity& sender, const uint8_t* secret, uint32_t sender_timestamp, const uint8_t* data, bool is_flood) {
  ClientInfo* client = NULL;
  if (data[0] == 0) {   // blank password, just check if sender is in ACL
    client = acl.getClient(sender.pub_key, PUB_KEY_SIZE);
    if (client == NULL) {
    #if MESH_DEBUG
      MESH_DEBUG_PRINTLN("Login, sender not in ACL");
    #endif
    }
  }
  if (client == NULL) {
    uint8_t perms;
    if (strcmp((char *)data, _prefs.password) == 0) { // check for valid admin password
      perms = PERM_ACL_ADMIN;
    } else if (strcmp((char *)data, _prefs.guest_password) == 0) { // check guest password
      perms = PERM_ACL_GUEST;
    } else {
#if MESH_DEBUG
      MESH_DEBUG_PRINTLN("Invalid password: %s", data);
#endif
      return 0;
    }

    client = acl.putClient(sender, 0);  // add to contacts (if not already known)
    if (sender_timestamp <= client->last_timestamp) {
      MESH_DEBUG_PRINTLN("Possible login replay attack!");
      return 0;  // FATAL: client table is full -OR- replay attack
    }

    MESH_DEBUG_PRINTLN("Login success!");
    client->last_timestamp = sender_timestamp;
    client->last_activity = getRTCClock()->getCurrentTime();
    client->permissions &= ~0x03;
    client->permissions |= perms;
    memcpy(client->shared_secret, secret, PUB_KEY_SIZE);

    if (perms != PERM_ACL_GUEST) {   // keep number of FS writes to a minimum
      dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);
    }
  }

  if (is_flood) {
    client->out_path_len = OUT_PATH_UNKNOWN;  // need to rediscover out_path
  }

  uint32_t now = getRTCClock()->getCurrentTimeUnique();
  memcpy(reply_data, &now, 4);   // response packets always prefixed with timestamp
  reply_data[4] = RESP_SERVER_LOGIN_OK;
  reply_data[5] = 0;  // Legacy: was recommended keep-alive interval (secs / 16)
  reply_data[6] = client->isAdmin() ? 1 : 0;
  reply_data[7] = client->permissions;
  getRNG()->random(&reply_data[8], 4);   // random blob to help packet-hash uniqueness
  reply_data[12] = FIRMWARE_VER_LEVEL;  // New field

  return 13;  // reply length
}

uint8_t MyMesh::handleAnonRegionsReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
  if (anon_limiter.allow(rtc_clock.getCurrentTime())) {
    // request data has: {reply-path-len}{reply-path}
    reply_path_len = *data & 63;
    reply_path_hash_size = (*data >> 6) + 1;
    data++;

    memcpy(reply_path, data, ((uint8_t)reply_path_len) * reply_path_hash_size);
    // data += (uint8_t)reply_path_len * reply_path_hash_size;

    memcpy(reply_data, &sender_timestamp, 4);   // prefix with sender_timestamp, like a tag
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply_data[4], &now, 4);     // include our clock (for easy clock sync, and packet hash uniqueness)

    return 8 + region_map.exportNamesTo((char *) &reply_data[8], sizeof(reply_data) - 12, REGION_DENY_FLOOD);   // reply length
  }
  return 0;
}

uint8_t MyMesh::handleAnonOwnerReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
  if (anon_limiter.allow(rtc_clock.getCurrentTime())) {
    // request data has: {reply-path-len}{reply-path}
    reply_path_len = *data & 63;
    reply_path_hash_size = (*data >> 6) + 1;
    data++;

    memcpy(reply_path, data, ((uint8_t)reply_path_len) * reply_path_hash_size);
    // data += (uint8_t)reply_path_len * reply_path_hash_size;

    memcpy(reply_data, &sender_timestamp, 4);   // prefix with sender_timestamp, like a tag
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply_data[4], &now, 4);     // include our clock (for easy clock sync, and packet hash uniqueness)
    sprintf((char *) &reply_data[8], "%s\n%s", _prefs.node_name, _prefs.owner_info);

    return 8 + strlen((char *) &reply_data[8]);   // reply length
  }
  return 0;
}

uint8_t MyMesh::handleAnonClockReq(const mesh::Identity& sender, uint32_t sender_timestamp, const uint8_t* data) {
  if (anon_limiter.allow(rtc_clock.getCurrentTime())) {
    // request data has: {reply-path-len}{reply-path}
    reply_path_len = *data & 63;
    reply_path_hash_size = (*data >> 6) + 1;
    data++;

    memcpy(reply_path, data, ((uint8_t)reply_path_len) * reply_path_hash_size);
    // data += (uint8_t)reply_path_len * reply_path_hash_size;

    memcpy(reply_data, &sender_timestamp, 4);   // prefix with sender_timestamp, like a tag
    uint32_t now = getRTCClock()->getCurrentTime();
    memcpy(&reply_data[4], &now, 4);     // include our clock (for easy clock sync, and packet hash uniqueness)
    reply_data[8] = 0;  // features
#ifdef WITH_RS232_BRIDGE
    reply_data[8] |= 0x01;  // is bridge, type UART
#elif WITH_ESPNOW_BRIDGE
    reply_data[8] |= 0x03;  // is bridge, type ESP-NOW
#endif
    if (_prefs.disable_fwd) {   // is this repeater currently disabled
      reply_data[8] |= 0x80;  // is disabled
    }
    // TODO:  add some kind of moving-window utilisation metric, so can query 'how busy' is this repeater
    return 9;   // reply length
  }
  return 0;
}

int MyMesh::handleRequest(ClientInfo *sender, uint32_t sender_timestamp, uint8_t *payload, size_t payload_len) {
  // uint32_t now = getRTCClock()->getCurrentTimeUnique();
  // memcpy(reply_data, &now, 4);   // response packets always prefixed with timestamp
  memcpy(reply_data, &sender_timestamp, 4); // reflect sender_timestamp back in response packet (kind of like a 'tag')

  if (payload[0] == REQ_TYPE_GET_STATUS) {  // guests can also access this now
    RepeaterStats stats;
    stats.batt_milli_volts = board.getBattMilliVolts();
    stats.curr_tx_queue_len = _mgr->getOutboundTotal();
    stats.noise_floor = (int16_t)_radio->getNoiseFloor();
    stats.last_rssi = (int16_t)radio_driver.getLastRSSI();
    stats.n_packets_recv = radio_driver.getPacketsRecv();
    stats.n_packets_sent = radio_driver.getPacketsSent();
    stats.total_air_time_secs = getTotalAirTime() / 1000;
    stats.total_up_time_secs = uptime_millis / 1000;
    stats.n_sent_flood = getNumSentFlood();
    stats.n_sent_direct = getNumSentDirect();
    stats.n_recv_flood = getNumRecvFlood();
    stats.n_recv_direct = getNumRecvDirect();
    stats.err_events = _err_flags;
    stats.last_snr = (int16_t)(radio_driver.getLastSNR() * 4);
    stats.n_direct_dups = ((SimpleMeshTables *)getTables())->getNumDirectDups();
    stats.n_flood_dups = ((SimpleMeshTables *)getTables())->getNumFloodDups();
    stats.total_rx_air_time_secs = getReceiveAirTime() / 1000;
    stats.n_recv_errors = radio_driver.getPacketsRecvErrors();
    memcpy(&reply_data[4], &stats, sizeof(stats));

    return 4 + sizeof(stats); //  reply_len
  }
  if (payload[0] == REQ_TYPE_GET_TELEMETRY_DATA) {
    uint8_t perm_mask = ~(payload[1]); // NEW: first reserved byte (of 4), is now inverse mask to apply to permissions

    telemetry.reset();
    telemetry.addVoltage(TELEM_CHANNEL_SELF, (float)board.getBattMilliVolts() / 1000.0f);

    // query other sensors -- target specific
    if ((sender->permissions & PERM_ACL_ROLE_MASK) == PERM_ACL_GUEST) {
      perm_mask = 0x00;  // just base telemetry allowed
    }
    sensors.querySensors(perm_mask, telemetry);

	// This default temperature will be overridden by external sensors (if any)
    float temperature = board.getMCUTemperature();
    if(!isnan(temperature)) { // Supported boards with built-in temperature sensor. ESP32-C3 may return NAN
      telemetry.addTemperature(TELEM_CHANNEL_SELF, temperature); // Built-in MCU Temperature
    }

    uint8_t tlen = telemetry.getSize();
    memcpy(&reply_data[4], telemetry.getBuffer(), tlen);
    return 4 + tlen; // reply_len
  }
  if (payload[0] == REQ_TYPE_GET_ACCESS_LIST && sender->isAdmin()) {
    uint8_t res1 = payload[1];   // reserved for future  (extra query params)
    uint8_t res2 = payload[2];
    if (res1 == 0 && res2 == 0) {
      uint8_t ofs = 4;
      for (int i = 0; i < acl.getNumClients() && ofs + 7 <= sizeof(reply_data) - 4; i++) {
        auto c = acl.getClientByIdx(i);
        if (c->permissions == 0) continue;  // skip deleted entries
        memcpy(&reply_data[ofs], c->id.pub_key, 6); ofs += 6;  // just 6-byte pub_key prefix
        reply_data[ofs++] = c->permissions;
      }
      return ofs;
    }
  }
  if (payload[0] == REQ_TYPE_GET_NEIGHBOURS) {
    uint8_t request_version = payload[1];
    if (request_version == 0) {

      // reply data offset (after response sender_timestamp/tag)
      int reply_offset = 4;

      // get request params
      uint8_t count = payload[2]; // how many neighbours to fetch (0-255)
      uint16_t offset;
      memcpy(&offset, &payload[3], 2); // offset from start of neighbours list (0-65535)
      uint8_t order_by = payload[5]; // how to order neighbours. 0=newest_to_oldest, 1=oldest_to_newest, 2=strongest_to_weakest, 3=weakest_to_strongest
      uint8_t pubkey_prefix_length = payload[6]; // how many bytes of neighbour pub key we want
      // we also send a 4 byte random blob in payload[7...10] to help packet uniqueness

      MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS count=%d, offset=%d, order_by=%d, pubkey_prefix_length=%d", count, offset, order_by, pubkey_prefix_length);

      // clamp pub key prefix length to max pub key length
      if(pubkey_prefix_length > PUB_KEY_SIZE){
        pubkey_prefix_length = PUB_KEY_SIZE;
        MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS invalid pubkey_prefix_length=%d clamping to %d", pubkey_prefix_length, PUB_KEY_SIZE);
      }

      // create copy of neighbours list, skipping empty entries so we can sort it separately from main list
      int16_t neighbours_count = 0;
#if MAX_NEIGHBOURS
      NeighbourInfo* sorted_neighbours[MAX_NEIGHBOURS];
      for (int i = 0; i < MAX_NEIGHBOURS; i++) {
        auto neighbour = &neighbours[i];
        if (neighbour->heard_timestamp > 0) {
          sorted_neighbours[neighbours_count] = neighbour;
          neighbours_count++;
        }
      }

      // sort neighbours based on order
      if (order_by == 0) {
        // sort by newest to oldest
        MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS sorting newest to oldest");
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->heard_timestamp > b->heard_timestamp; // desc
        });
      } else if (order_by == 1) {
        // sort by oldest to newest
        MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS sorting oldest to newest");
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->heard_timestamp < b->heard_timestamp; // asc
        });
      } else if (order_by == 2) {
        // sort by strongest to weakest
        MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS sorting strongest to weakest");
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->snr > b->snr; // desc
        });
      } else if (order_by == 3) {
        // sort by weakest to strongest
        MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS sorting weakest to strongest");
        std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
          return a->snr < b->snr; // asc
        });
      }
#endif

      // build results buffer
      int results_count = 0;
      int results_offset = 0;
      uint8_t results_buffer[130];
      for(int index = 0; index < count && index + offset < neighbours_count; index++){
        
        // stop if we can't fit another entry in results
        int entry_size = pubkey_prefix_length + 4 + 1;
        if(results_offset + entry_size > sizeof(results_buffer)){
          MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS no more entries can fit in results buffer");
          break;
        }

#if MAX_NEIGHBOURS
        // add next neighbour to results
        auto neighbour = sorted_neighbours[index + offset];
        uint32_t heard_seconds_ago = getRTCClock()->getCurrentTime() - neighbour->heard_timestamp;
        memcpy(&results_buffer[results_offset], neighbour->id.pub_key, pubkey_prefix_length); results_offset += pubkey_prefix_length;
        memcpy(&results_buffer[results_offset], &heard_seconds_ago, 4); results_offset += 4;
        memcpy(&results_buffer[results_offset], &neighbour->snr, 1); results_offset += 1;
        results_count++;
#endif

      }

      // build reply
      MESH_DEBUG_PRINTLN("REQ_TYPE_GET_NEIGHBOURS neighbours_count=%d results_count=%d", neighbours_count, results_count);
      memcpy(&reply_data[reply_offset], &neighbours_count, 2); reply_offset += 2;
      memcpy(&reply_data[reply_offset], &results_count, 2); reply_offset += 2;
      memcpy(&reply_data[reply_offset], &results_buffer, results_offset); reply_offset += results_offset;

      return reply_offset;
    }
  } else if (payload[0] == REQ_TYPE_GET_OWNER_INFO) {
    sprintf((char *) &reply_data[4], "%s\n%s\n%s", FIRMWARE_VERSION, _prefs.node_name, _prefs.owner_info);
    return 4 + strlen((char *) &reply_data[4]);
  }
  return 0; // unknown command
}

mesh::Packet *MyMesh::createSelfAdvert() {
  uint8_t app_data[MAX_ADVERT_DATA_SIZE];
  uint8_t app_data_len = _cli.buildAdvertData(ADV_TYPE_REPEATER, app_data);

  return createAdvert(self_id, app_data, app_data_len);
}

File MyMesh::openAppend(const char *fname) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return _fs->open(fname, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  return _fs->open(fname, "a");
#else
  return _fs->open(fname, "a", true);
#endif
}


static uint8_t max_loop_minimal[] =  { 0, /* 1-byte */  4, /* 2-byte */  2, /* 3-byte */  1 };
static uint8_t max_loop_moderate[] = { 0, /* 1-byte */  2, /* 2-byte */  1, /* 3-byte */  1 };
static uint8_t max_loop_strict[] =   { 0, /* 1-byte */  1, /* 2-byte */  1, /* 3-byte */  1 };

bool MyMesh::isLooped(const mesh::Packet* packet, const uint8_t max_counters[]) {
  uint8_t hash_size = packet->getPathHashSize();
  uint8_t hash_count = packet->getPathHashCount();
  uint8_t n = 0;
  const uint8_t* path = packet->path;
  while (hash_count > 0) {      // count how many times this node is already in the path
    if (self_id.isHashMatch(path, hash_size)) n++;
    hash_count--;
    path += hash_size;
  }
  return n >= max_counters[hash_size];
}

void MyMesh::sendFloodReply(mesh::Packet* packet, unsigned long delay_millis, uint8_t path_hash_size) {
  if (recv_pkt_region && !recv_pkt_region->isWildcard()) {  // if _request_ packet scope is known, send reply with same scope
    TransportKey scope;
    if (region_map.getTransportKeysFor(*recv_pkt_region, &scope, 1) > 0) {
      sendFloodScoped(scope, packet, delay_millis, path_hash_size);
    } else {
      sendFlood(packet, delay_millis, path_hash_size);  // send un-scoped
    }
  } else {
    sendFlood(packet, delay_millis, path_hash_size);  // send un-scoped
  }
}

bool MyMesh::allowPacketForward(const mesh::Packet *packet) {
  if (_prefs.disable_fwd) return false;
  if (packet->isRouteFlood()) {
    if (packet->getPathHashCount() >= _prefs.flood_max) return false;
    if (packet->getRouteType() == ROUTE_TYPE_FLOOD && packet->getPathHashCount() >= _prefs.flood_max_unscoped) return false;
    // Adaptive flood.max.advert: fewer neighbours → allow more hops; dense hub → fewer hops
    if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT) {
      uint8_t eff_max_advert = _prefs.flood_max_advert;
#if MAX_NEIGHBOURS
      {
        int nbr = 0;
        uint32_t ts = getRTCClock()->getCurrentTime();
        for (int i = 0; i < MAX_NEIGHBOURS; i++) {
          if (neighbours[i].heard_timestamp > 0 && ts >= neighbours[i].heard_timestamp &&
              ts - neighbours[i].heard_timestamp < NEIGHBOUR_EXPIRATION_SECS) nbr++;
        }
        if      (nbr <= 2)  eff_max_advert = min(255, (int)eff_max_advert * 2);
        else if (nbr <= 5)  eff_max_advert = min(255, (int)eff_max_advert * 3 / 2);
        else if (nbr >= 9 && nbr <= 12) eff_max_advert = max(3, (int)eff_max_advert * 3 / 4);
        else if (nbr > 12)  eff_max_advert = max(2, (int)eff_max_advert / 2);
      }
#endif
      if (packet->getPathHashCount() >= eff_max_advert) return false;
    }
  }
  if (packet->isRouteFlood() && recv_pkt_region == NULL) {
    MESH_DEBUG_PRINTLN("allowPacketForward: unknown transport code, or wildcard not allowed for FLOOD packet");
    return false;
  }
  if (packet->isRouteFlood() && _prefs.loop_detect != LOOP_DETECT_OFF) {
    const uint8_t* maximums;
    if (_prefs.loop_detect == LOOP_DETECT_MINIMAL) {
      maximums = max_loop_minimal;
    } else if (_prefs.loop_detect == LOOP_DETECT_MODERATE) {
      maximums = max_loop_moderate;
    } else {
      maximums = max_loop_strict;
    }
    if (isLooped(packet, maximums)) {
      MESH_DEBUG_PRINTLN("allowPacketForward: FLOOD packet loop detected!");
      return false;
    }
  }

#ifdef DISABLE_LEGACY_ADVERT
  // Limit flood advert paket forwarding using a probabilistic reduction defined by P(h) = 0.308^(hops-1)
  // https://github.com/meshcore-dev/MeshCore/issues/1223
  if (packet->getPayloadType() == PAYLOAD_TYPE_ADVERT && packet->isRouteFlood()) {

    uint32_t now = getRTCClock()->getCurrentTime();
    DateTime dt = DateTime(now);
    uint8_t current_hour = dt.hour();

    if (current_hour >= ADVERTS_ALLOWED_START && current_hour <= ADVERTS_ALLOWED_END) {
      MESH_DEBUG_PRINTLN("Flood advert: within allowed advert window, allowing forward");
      return true; // Always adverts through during allowed hours
    }

    // Advert payload structure: [pub_key(32)][timestamp(4)][signature(64)][app_data...]
    const int app_data_offset = PUB_KEY_SIZE + 4 + SIGNATURE_SIZE; // 32 + 4 + 64 = 100

    // Extract advert type from app_data (lower 4 bits of first byte).
    uint8_t adv_type = (packet->payload_len > app_data_offset) ? (packet->payload[app_data_offset] & 0x0F) : 0xFF;

    // Reject adverts from repeaters whose pub_key starts with 0x01 (MOBILE NODE)
    if (packet->payload_len > 0 && adv_type == ADV_TYPE_REPEATER && packet->payload[0] == 0x01) {
      MESH_DEBUG_PRINTLN("Flood advert REJECTED: pub_key starts with 0x01 for repeater");
      return false;
    }

    if (packet->payload_len > app_data_offset && adv_type != ADV_TYPE_NONE && adv_type != ADV_TYPE_CHAT) {
      // Use local validated value to avoid modifying preferences in packet-forwarding logic
      float base_value = _prefs.flood_advert_base;
      if (base_value <= 0.0f || base_value > 1.0f) {
        MESH_DEBUG_PRINTLN("WARNING: Invalid flood_advert_base=%.3f, using default 0.308",
                           base_value);
        base_value = 0.308f;
      }

      // Adapt flood probability to active neighbour density.
      // A hub with many neighbours should be more selective (reduce congestion).
      // An edge node with few neighbours should be more aggressive (ensure coverage).
      // Scale is applied on top of the user-configured base value.
#if MAX_NEIGHBOURS
      {
        int active_count = 0;
        uint32_t now_ts = getRTCClock()->getCurrentTime();
        for (int i = 0; i < MAX_NEIGHBOURS; i++) {
          if (neighbours[i].heard_timestamp > 0 &&
              now_ts >= neighbours[i].heard_timestamp &&
              now_ts - neighbours[i].heard_timestamp < NEIGHBOUR_EXPIRATION_SECS) {
            active_count++;
          }
        }
        float scale;
        if      (active_count <= 2)  scale = 1.6f;  // very sparse  → propagate aggressively
        else if (active_count <= 5)  scale = 1.2f;  // sparse       → slightly more aggressive
        else if (active_count <= 8)  scale = 1.0f;  // moderate     → use configured value
        else if (active_count <= 12) scale = 0.75f; // dense        → more selective
        else                         scale = 0.60f; // very dense   → maximum selectivity
        base_value *= scale;
        base_value = (base_value < 0.15f) ? 0.15f : (base_value > 0.65f ? 0.65f : base_value);
        MESH_DEBUG_PRINTLN("Adaptive flood: neighbours=%d scale=%.2f base=%.3f",
                           active_count, scale, base_value);
      }
#endif

      if (packet->getPathHashCount() == 0) {
        MESH_DEBUG_PRINTLN("Flood advert: path_len=0, allowing forward");
        _flood_advert_accepted++;
        return true; // Always allow zero-hop adverts through
      }

      double_t roll_dice = (double)getRNG()->nextInt(0, 10000) / 10000.0;
      double_t forw_prob = pow(base_value, packet->path_len - 1);
      MESH_DEBUG_PRINTLN("Flood advert filter: path_len=%d, roll=%.3f, prob=%.3f, base=%.3f",
                         packet->path_len, roll_dice, forw_prob, base_value);

      if (roll_dice > forw_prob) {
        MESH_DEBUG_PRINTLN("Flood advert REJECTED by probabilistic filter");
        _flood_advert_rejected++;
        return false;
      } else {
        MESH_DEBUG_PRINTLN("Flood advert ACCEPTED for forwarding");
        _flood_advert_accepted++;
      }

    }
#ifdef MESH_DEBUG
    else if (packet->payload_len > app_data_offset) {
      MESH_DEBUG_PRINTLN("Flood advert filter SKIPPED: type=%d", adv_type);
    } else {
      MESH_DEBUG_PRINTLN("Flood advert filter SKIPPED: payload_len=%d too short (need >%d)",
                         packet->payload_len, app_data_offset);
    }
#endif
  }
#endif

  // all other packets
  return true;
}

const char *MyMesh::getLogDateTime() {
  static char tmp[32];
  uint32_t now = getRTCClock()->getCurrentTime();
  DateTime dt = DateTime(now);
  sprintf(tmp, "%02d:%02d:%02d - %d/%d/%d U", dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(),
          dt.year());
  return tmp;
}

void MyMesh::logRxRaw(float snr, float rssi, const uint8_t raw[], int len) {
#if MESH_PACKET_LOGGING
  Serial.print(getLogDateTime());
  Serial.print(" RAW: ");
  mesh::Utils::printHex(Serial, raw, len);
  Serial.println();
#endif
}

void MyMesh::logRx(mesh::Packet *pkt, int len, float score) {
#ifdef WITH_BRIDGE
  if (_prefs.bridge_pkt_src == 1) {
    bridge.sendPacket(pkt);
  }
#endif

  if (_logging) {
    File f = openAppend(PACKET_LOG_FILE);
    if (f) {
      f.print(getLogDateTime());
      f.printf(": RX, len=%d (type=%d, route=%s, payload_len=%d) SNR=%d RSSI=%d score=%d", len,
               pkt->getPayloadType(), pkt->isRouteDirect() ? "D" : "F", pkt->payload_len,
               (int)_radio->getLastSNR(), (int)_radio->getLastRSSI(), (int)(score * 1000));

      if (pkt->getPayloadType() == PAYLOAD_TYPE_PATH || pkt->getPayloadType() == PAYLOAD_TYPE_REQ ||
          pkt->getPayloadType() == PAYLOAD_TYPE_RESPONSE || pkt->getPayloadType() == PAYLOAD_TYPE_TXT_MSG) {
        f.printf(" [%02X -> %02X]\n", (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
      } else {
        f.printf("\n");
      }
      f.close();
    }
  }
}

void MyMesh::logTx(mesh::Packet *pkt, int len) {
#ifdef WITH_BRIDGE
  if (_prefs.bridge_pkt_src == 0) {
    bridge.sendPacket(pkt);
  }
#endif

  if (_logging) {
    File f = openAppend(PACKET_LOG_FILE);
    if (f) {
      f.print(getLogDateTime());
      f.printf(": TX, len=%d (type=%d, route=%s, payload_len=%d)", len, pkt->getPayloadType(),
               pkt->isRouteDirect() ? "D" : "F", pkt->payload_len);

      if (pkt->getPayloadType() == PAYLOAD_TYPE_PATH || pkt->getPayloadType() == PAYLOAD_TYPE_REQ ||
          pkt->getPayloadType() == PAYLOAD_TYPE_RESPONSE || pkt->getPayloadType() == PAYLOAD_TYPE_TXT_MSG) {
        f.printf(" [%02X -> %02X]\n", (uint32_t)pkt->payload[1], (uint32_t)pkt->payload[0]);
      } else {
        f.printf("\n");
      }
      f.close();
    }
  }
}

void MyMesh::logTxFail(mesh::Packet *pkt, int len) {
  if (_logging) {
    File f = openAppend(PACKET_LOG_FILE);
    if (f) {
      f.print(getLogDateTime());
      f.printf(": TX FAIL!, len=%d (type=%d, route=%s, payload_len=%d)\n", len, pkt->getPayloadType(),
               pkt->isRouteDirect() ? "D" : "F", pkt->payload_len);
      f.close();
    }
  }
}

int MyMesh::calcRxDelay(float score, uint32_t air_time) const {
  if (_prefs.rx_delay_base <= 0.0f) return 0;
  return (int)((pow(_prefs.rx_delay_base, 0.85f - score) - 1.0) * air_time);
}

uint32_t MyMesh::getRetransmitDelay(const mesh::Packet *packet) {
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * _prefs.tx_delay_factor);
  return getRNG()->nextInt(0, 5*t + 1);
}
uint32_t MyMesh::getDirectRetransmitDelay(const mesh::Packet *packet) {
  uint32_t t = (_radio->getEstAirtimeFor(packet->getPathByteLen() + packet->payload_len + 2) * _prefs.direct_tx_delay_factor);
  return getRNG()->nextInt(0, 5*t + 1);
}

bool MyMesh::filterRecvFloodPacket(mesh::Packet* pkt) {
  // just try to determine region for packet (apply later in allowPacketForward())
  if (pkt->getRouteType() == ROUTE_TYPE_TRANSPORT_FLOOD) {
    recv_pkt_region = region_map.findMatch(pkt, REGION_DENY_FLOOD);
  } else if (pkt->getRouteType() == ROUTE_TYPE_FLOOD) {
    if (region_map.getWildcard().flags & REGION_DENY_FLOOD) {
      recv_pkt_region = NULL;
    } else {
      recv_pkt_region =  &region_map.getWildcard();
    }
  } else {
    recv_pkt_region = NULL;
  }
  // do normal processing
  return false;
}

void MyMesh::onAnonDataRecv(mesh::Packet *packet, const uint8_t *secret, const mesh::Identity &sender,
                            uint8_t *data, size_t len) {
  if (packet->getPayloadType() == PAYLOAD_TYPE_ANON_REQ) { // received an initial request by a possible admin
                                                           // client (unknown at this stage)
    uint32_t timestamp;
    memcpy(&timestamp, data, 4);

    data[len] = 0;  // ensure null terminator
    uint8_t reply_len;

    reply_path_len = -1;
    if (data[4] == 0 || data[4] >= ' ') {   // is password, ie. a login request
      reply_len = handleLoginReq(sender, secret, timestamp, &data[4], packet->isRouteFlood());
    } else if (data[4] == ANON_REQ_TYPE_REGIONS && packet->isRouteDirect()) {
      reply_len = handleAnonRegionsReq(sender, timestamp, &data[5]);
    } else if (data[4] == ANON_REQ_TYPE_OWNER && packet->isRouteDirect()) {
      reply_len = handleAnonOwnerReq(sender, timestamp, &data[5]);
    } else if (data[4] == ANON_REQ_TYPE_BASIC && packet->isRouteDirect()) {
      reply_len = handleAnonClockReq(sender, timestamp, &data[5]);
    } else {
      reply_len = 0;  // unknown/invalid request type
    }

    if (reply_len == 0) return;   // invalid request

    if (packet->isRouteFlood()) {
      // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
      mesh::Packet* path = createPathReturn(sender, secret, packet->path, packet->path_len,
                                            PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
      if (path) sendFloodReply(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
    } else if (reply_path_len < 0) {
      mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
      if (reply) sendFloodReply(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
    } else {
      mesh::Packet* reply = createDatagram(PAYLOAD_TYPE_RESPONSE, sender, secret, reply_data, reply_len);
      uint8_t path_len = ((reply_path_hash_size - 1) << 6) | (reply_path_len & 63);
      if (reply) sendDirect(reply, reply_path,  path_len, SERVER_RESPONSE_DELAY);
    }
  }
}

int MyMesh::searchPeersByHash(const uint8_t *hash) {
  int n = 0;
  for (int i = 0; i < acl.getNumClients(); i++) {
    if (acl.getClientByIdx(i)->id.isHashMatch(hash)) {
      matching_peer_indexes[n++] = i; // store the INDEXES of matching contacts (for subsequent 'peer' methods)
    }
  }
  return n;
}

void MyMesh::getPeerSharedSecret(uint8_t *dest_secret, int peer_idx) {
  int i = matching_peer_indexes[peer_idx];
  if (i >= 0 && i < acl.getNumClients()) {
    // lookup pre-calculated shared_secret
    memcpy(dest_secret, acl.getClientByIdx(i)->shared_secret, PUB_KEY_SIZE);
  } else {
    MESH_DEBUG_PRINTLN("getPeerSharedSecret: Invalid peer idx: %d", i);
  }
}

static bool isShare(const mesh::Packet *packet) {
  if (packet->hasTransportCodes()) {
    return packet->transport_codes[0] == 0 && packet->transport_codes[1] == 0;  // codes { 0, 0 } means 'send to nowhere'
  }
  return false;
}

void MyMesh::onAdvertRecv(mesh::Packet *packet, const mesh::Identity &id, uint32_t timestamp,
                          const uint8_t *app_data, size_t app_data_len) {
  mesh::Mesh::onAdvertRecv(packet, id, timestamp, app_data, app_data_len); // chain to super impl

  // if this a zero hop advert (and not via 'Share'), add it to neighbours
  if (packet->getPathHashCount() == 0 && !isShare(packet)) {
    AdvertDataParser parser(app_data, app_data_len);
    if (parser.isValid() && parser.getType() == ADV_TYPE_REPEATER) { // just keep neigbouring Repeaters
      putNeighbour(id, timestamp, packet->getSNR());
    }
  }

#ifdef ENABLE_MASTER_TIME_SYNC
  static const uint8_t MASTER_TIME_SYNC_IDENTITY[PUB_KEY_SIZE] = {
    0x01, 0xB2, 0xF5, 0xDA, 0x46, 0xBC, 0x0A, 0x9C, 0x67, 0xFB, 0x8E, 0xDC, 0x36, 0x62, 0x57, 0xB6,
    0x04, 0x52, 0x73, 0xB8, 0x9F, 0x37, 0xF3, 0x08, 0x04, 0x4A, 0xD5, 0x57, 0x17, 0x34, 0xD4, 0x62
  };

  // ignore timestamps before year 2026 (Unix timestamp 1767225600)
  if (timestamp >= 1767225600 && packet->path_len < 8) {
    AdvertDataParser parser(app_data, app_data_len);
    if (parser.isValid() && parser.getType() == ADV_TYPE_NONE) {
      if (memcmp(id.pub_key, MASTER_TIME_SYNC_IDENTITY, PUB_KEY_SIZE) == 0) {
        uint32_t now = getRTCClock()->getCurrentTime();
        int32_t diff = (int32_t)timestamp - (int32_t)now;
        if (diff >= 30 || diff <= -30) {
          getRTCClock()->setCurrentTime(timestamp);
          DateTime dt = DateTime(timestamp);
          MESH_DEBUG_PRINTLN("Master time sync: clock updated to %02d:%02d:%02d - %d/%d/%d (diff=%d sec)",
                             dt.hour(), dt.minute(), dt.second(), dt.day(), dt.month(), dt.year(), diff);
        } else {
          MESH_DEBUG_PRINTLN("Master time sync: ignored small diff=%d sec (threshold=30)", diff);
        }
      } else {
        MESH_DEBUG_PRINTLN("Master time sync: invalid ID [%02X%02X], ignoring timestamp", id.pub_key[0],
                           id.pub_key[1]);
      }
    }
  }
#endif

#ifdef ENABLE_CONSENSUS_TIME_SYNC
  // limit time sync samples to 4 hops max
  // ignore timestamps before year 2026 (Unix timestamp 1767225600)
  if (timestamp >= 1767225600 && packet->path_len < 8 && !isShare(packet)) {
    uint32_t now = getRTCClock()->getCurrentTime();
    bool found = false;
    for (int i = 0; i < TIME_SYNC_SAMPLES; i++) {
      if (memcmp(time_samples[i].sender_prefix, id.pub_key, 4) == 0) {
        if (now - time_samples[i].sampled_at > 3600) {
          time_samples[i].offset = (int32_t)timestamp - (int32_t)now;
          time_samples[i].sampled_at = now;
          DateTime dt = DateTime(timestamp);
          MESH_DEBUG_PRINTLN("Time sample updated: [%02X%02X] %02d:%02d:%02d - %d/%d/%d offset=%d",
                             id.pub_key[0], id.pub_key[1], dt.hour(), dt.minute(), dt.second(), dt.day(),
                             dt.month(), dt.year(), time_samples[i].offset);
        }
        found = true;
        break;
      }
    }
    if (!found) {
      memcpy(time_samples[time_sample_idx].sender_prefix, id.pub_key, 4);
      time_samples[time_sample_idx].offset = (int32_t)timestamp - (int32_t)now;
      time_samples[time_sample_idx].sampled_at = now;
      DateTime dt = DateTime(timestamp);
      MESH_DEBUG_PRINTLN("Time sample added: [%02X%02X] %02d:%02d:%02d - %d/%d/%d offset=%d idx=%d",
                         id.pub_key[0], id.pub_key[1], dt.hour(), dt.minute(), dt.second(), dt.day(),
                         dt.month(), dt.year(), time_samples[time_sample_idx].offset, time_sample_idx);
      time_sample_idx = (time_sample_idx + 1) % TIME_SYNC_SAMPLES;
    }
#if MESH_DEBUG
  } else if (packet->path_len > 0 && !isShare(packet)) {
    // Log why advert was rejected for time sync
    if (timestamp < 1767225600) {
      DateTime dt = DateTime(timestamp);
      MESH_DEBUG_PRINTLN("Time sample rejected: [%02X%02X] timestamp too old (%d/%d/%d)",
                         id.pub_key[0], id.pub_key[1], dt.day(), dt.month(), dt.year());
    } else if (packet->path_len >= 6) {
      MESH_DEBUG_PRINTLN("Time sample rejected: [%02X%02X] too many hops (%d)",
                         id.pub_key[0], id.pub_key[1], packet->path_len);
    }
#endif
  }
#endif
}

void MyMesh::onPeerDataRecv(mesh::Packet *packet, uint8_t type, int sender_idx, const uint8_t *secret,
                            uint8_t *data, size_t len) {
  int i = matching_peer_indexes[sender_idx];
  if (i < 0 || i >= acl.getNumClients()) { // get from our known_clients table (sender SHOULD already be known in this context)
    MESH_DEBUG_PRINTLN("onPeerDataRecv: invalid peer idx: %d", i);
    return;
  }
  ClientInfo* client = acl.getClientByIdx(i);

  if (type == PAYLOAD_TYPE_REQ) { // request (from a Known admin client!)
    uint32_t timestamp;
    memcpy(&timestamp, data, 4);

    if (timestamp > client->last_timestamp) { // prevent replay attacks
      int reply_len = handleRequest(client, timestamp, &data[4], len - 4);
      if (reply_len == 0) return; // invalid command

      client->last_timestamp = timestamp;
      client->last_activity = getRTCClock()->getCurrentTime();

      if (packet->isRouteFlood()) {
        // let this sender know path TO here, so they can use sendDirect(), and ALSO encode the response
        mesh::Packet *path = createPathReturn(client->id, secret, packet->path, packet->path_len,
                                              PAYLOAD_TYPE_RESPONSE, reply_data, reply_len);
        if (path) sendFloodReply(path, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
      } else {
        mesh::Packet *reply =
            createDatagram(PAYLOAD_TYPE_RESPONSE, client->id, secret, reply_data, reply_len);
        if (reply) {
          if (client->out_path_len != OUT_PATH_UNKNOWN) { // we have an out_path, so send DIRECT
            sendDirect(reply, client->out_path, client->out_path_len, SERVER_RESPONSE_DELAY);
          } else {
            sendFloodReply(reply, SERVER_RESPONSE_DELAY, packet->getPathHashSize());
          }
        }
      }
    } else {
      MESH_DEBUG_PRINTLN("onPeerDataRecv: possible replay attack detected");
    }
  } else if (type == PAYLOAD_TYPE_TXT_MSG && len > 5 && client->isAdmin()) { // a CLI command
    uint32_t sender_timestamp;
    memcpy(&sender_timestamp, data, 4); // timestamp (by sender's RTC clock - which could be wrong)
    uint8_t flags = (data[4] >> 2);        // message attempt number, and other flags

    if (!(flags == TXT_TYPE_PLAIN || flags == TXT_TYPE_CLI_DATA)) {
      MESH_DEBUG_PRINTLN("onPeerDataRecv: unsupported text type received: flags=%02x", (uint32_t)flags);
    } else if (sender_timestamp >= client->last_timestamp) { // prevent replay attacks
      bool is_retry = (sender_timestamp == client->last_timestamp);
      client->last_timestamp = sender_timestamp;
      client->last_activity = getRTCClock()->getCurrentTime();

      // len can be > original length, but 'text' will be padded with zeroes
      data[len] = 0; // need to make a C string again, with null terminator

      if (flags == TXT_TYPE_PLAIN) { // for legacy CLI, send Acks
        uint32_t ack_hash; // calc truncated hash of the message timestamp + text + sender pub_key, to prove
                           // to sender that we got it
        mesh::Utils::sha256((uint8_t *)&ack_hash, 4, data, 5 + strlen((char *)&data[5]), client->id.pub_key,
                            PUB_KEY_SIZE);

        mesh::Packet *ack = createAck(ack_hash);
        if (ack) {
          if (client->out_path_len == OUT_PATH_UNKNOWN) {
            sendFloodReply(ack, TXT_ACK_DELAY, packet->getPathHashSize());
          } else {
            sendDirect(ack, client->out_path, client->out_path_len, TXT_ACK_DELAY);
          }
        }
      }

      uint8_t temp[166];
      char *command = (char *)&data[5];
      char *reply = (char *)&temp[5];
      if (is_retry) {
        *reply = 0;
      } else {
        handleCommand(sender_timestamp, command, reply);
      }
      int text_len = strlen(reply);
      if (text_len > 0) {
        uint32_t timestamp = getRTCClock()->getCurrentTimeUnique();
        if (timestamp == sender_timestamp) {
          // WORKAROUND: the two timestamps need to be different, in the CLI view
          timestamp++;
        }
        memcpy(temp, &timestamp, 4);        // mostly an extra blob to help make packet_hash unique
        temp[4] = (TXT_TYPE_CLI_DATA << 2); // NOTE: legacy was: TXT_TYPE_PLAIN

        auto reply = createDatagram(PAYLOAD_TYPE_TXT_MSG, client->id, secret, temp, 5 + text_len);
        if (reply) {
          if (client->out_path_len == OUT_PATH_UNKNOWN) {
            sendFloodReply(reply, CLI_REPLY_DELAY_MILLIS, packet->getPathHashSize());
          } else {
            sendDirect(reply, client->out_path, client->out_path_len, CLI_REPLY_DELAY_MILLIS);
          }
        }
      }
    } else {
      MESH_DEBUG_PRINTLN("onPeerDataRecv: possible replay attack detected");
    }
  }
}

bool MyMesh::onPeerPathRecv(mesh::Packet *packet, int sender_idx, const uint8_t *secret, uint8_t *path,
                            uint8_t path_len, uint8_t extra_type, uint8_t *extra, uint8_t extra_len) {
  // TODO: prevent replay attacks
  int i = matching_peer_indexes[sender_idx];

  if (i >= 0 && i < acl.getNumClients()) { // get from our known_clients table (sender SHOULD already be known in this context)
    MESH_DEBUG_PRINTLN("PATH to client, path_len=%d", (uint32_t)path_len);
    auto client = acl.getClientByIdx(i);

    // store a copy of path, for sendDirect()
    client->out_path_len = mesh::Packet::copyPath(client->out_path, path, path_len);
    client->last_activity = getRTCClock()->getCurrentTime();
  } else {
    MESH_DEBUG_PRINTLN("onPeerPathRecv: invalid peer idx: %d", i);
  }

  // NOTE: no reciprocal path send!!
  return false;
}

#define CTL_TYPE_NODE_DISCOVER_REQ   0x80
#define CTL_TYPE_NODE_DISCOVER_RESP  0x90

void MyMesh::onControlDataRecv(mesh::Packet *packet) {
#if !defined(ENABLE_STEALTH_MODE)
  uint8_t type = packet->payload[0] & 0xF0; // just test upper 4 bits
  if (type == CTL_TYPE_NODE_DISCOVER_REQ && packet->payload_len >= 6 && !_prefs.disable_fwd &&
      discover_limiter.allow(rtc_clock.getCurrentTime())) {
    int i = 1;
    uint8_t filter = packet->payload[i++];
    uint32_t tag;
    memcpy(&tag, &packet->payload[i], 4);
    i += 4;
    uint32_t since;
    if (packet->payload_len >= i + 4) { // optional since field
      memcpy(&since, &packet->payload[i], 4);
      i += 4;
    } else {
      since = 0;
    }

    if ((filter & (1 << ADV_TYPE_REPEATER)) != 0 && _prefs.discovery_mod_timestamp >= since) {
      bool prefix_only = packet->payload[0] & 1;
      uint8_t data[6 + PUB_KEY_SIZE];
      data[0] = CTL_TYPE_NODE_DISCOVER_RESP | ADV_TYPE_REPEATER; // low 4-bits for node type
      data[1] = packet->_snr;                                    // let sender know the inbound SNR ( x 4)
      memcpy(&data[2], &tag, 4); // include tag from request, for client to match to
      memcpy(&data[6], self_id.pub_key, PUB_KEY_SIZE);
      auto resp = createControlData(data, prefix_only ? 6 + 8 : 6 + PUB_KEY_SIZE);
      if (resp) {
        sendZeroHop(resp, getRetransmitDelay(resp) *
                              4); // apply random delay (widened x4), as multiple nodes can respond to this
      }
    }
  } else
#endif
  if (type == CTL_TYPE_NODE_DISCOVER_RESP && packet->payload_len >= 6) {
    uint8_t node_type = packet->payload[0] & 0x0F;
    if (node_type != ADV_TYPE_REPEATER) {
      return;
    }
    if (packet->payload_len < 6 + PUB_KEY_SIZE) {
      MESH_DEBUG_PRINTLN("onControlDataRecv: DISCOVER_RESP pubkey too short: %d",
                         (uint32_t)packet->payload_len);
      return;
    }

    if (pending_discover_tag == 0 || millisHasNowPassed(pending_discover_until)) {
      pending_discover_tag = 0;
      return;
    }
    uint32_t tag;
    memcpy(&tag, &packet->payload[2], 4);
    if (tag != pending_discover_tag) {
      return;
    }

    mesh::Identity id(&packet->payload[6]);
    if (id.matches(self_id)) {
      return;
    }
    putNeighbour(id, rtc_clock.getCurrentTime(), packet->getSNR());
  }
}

void MyMesh::sendNodeDiscoverReq() {
  uint8_t data[10];
  data[0] = CTL_TYPE_NODE_DISCOVER_REQ; // prefix_only=0
  data[1] = (1 << ADV_TYPE_REPEATER);
  getRNG()->random(&data[2], 4); // tag
  memcpy(&pending_discover_tag, &data[2], 4);
  pending_discover_until = futureMillis(60000);
  uint32_t since = 0;
  memcpy(&data[6], &since, 4);

  auto pkt = createControlData(data, sizeof(data));
  if (pkt) {
    sendZeroHop(pkt);
  }
}

MyMesh::MyMesh(mesh::MainBoard &board, mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
               mesh::RTCClock &rtc, mesh::MeshTables &tables)
    : mesh::Mesh(radio, ms, rng, rtc, *new StaticPoolPacketManager(32), tables),
      region_map(key_store), temp_map(key_store),
      _cli(board, rtc, sensors, region_map, acl, &_prefs, this),
      telemetry(MAX_PACKET_PAYLOAD - 4),
      discover_limiter(4, 120),  // max 4 every 2 minutes
      anon_limiter(4, 180)   // max 4 every 3 minutes
#if defined(WITH_RS232_BRIDGE)
      , bridge(&_prefs, WITH_RS232_BRIDGE, _mgr, &rtc)
#endif
#if defined(WITH_ESPNOW_BRIDGE)
      , bridge(&_prefs, _mgr, &rtc)
#endif
{
  last_millis = 0;
  uptime_millis = 0;
  
  adverts_sent = 0;
  next_advert_check = futureMillis(30000);
  next_local_advert = next_flood_advert = next_flood_advert_offset = 0;

  // boot advert: 3 zero-hop adverts at 60s/120s/180s after startup
  _boot_advert_count = 0;
  _next_boot_advert = futureMillis(60000);

  // boot node discover: populate neighbour table 90s after startup
  _boot_discover_sent = false;
  _next_boot_discover = futureMillis(90000);

  // battery-adaptive TX power
  _batt_tx_reduced = false;
  _next_batt_check = futureMillis(30000);

  // radio watchdog
  _radio_silence_count = 0;
  _radio_last_pkt_count = 0;
  _next_radio_watchdog = futureMillis(120000);

  // temperature-aware TX power
  _temp_tx_reduced = false;
  _next_temp_check = futureMillis(300000);

  // TPC
  _tpc_offset_dbm = 0;
  _next_tpc_check = futureMillis(1800000);

  // weekly reboot
  _next_reboot_check = futureMillis(3600000);

  // cached sensor readings (updated by timers, not every loop)
  _cached_batt_mv    = 0;
  _cached_temperature = NAN;

  // flood filter stats
  _flood_advert_accepted = 0;
  _flood_advert_rejected = 0;

  // isolation detection
  _isolation_silence_count = 0;
  _isolation_last_pkt_count = 0;
  _isolation_advert_pending = false;
  _next_isolation_check = futureMillis(3600000);
  _isolation_advert_cooldown = 0;

  // packet rate
  _pkt_rate_recv = 0;
  _pkt_rate_sent = 0;
  _rate_last_recv = 0;
  _rate_last_sent = 0;
  _next_rate_update = futureMillis(60000);

  // duty cycle rolling window
  memset(_airtime_window, 0, sizeof(_airtime_window));
  _airtime_window_idx   = 0;
  _airtime_window_count = 0;
  _next_airtime_snap    = futureMillis(300000);

  // reboot counter (loaded from flash in begin())
  _reboot_count = 0;

  // memory watchdog
  _heap_fail_count  = 0;
  _next_heap_check  = futureMillis(1800000); // first check 30min after boot

  // last RX tracking
  _last_rx_pkt_cnt = 0;
  _last_rx_millis  = millis();

  // hourly DC history
  memset(_hourly_dc_ms, 0, sizeof(_hourly_dc_ms));
  _airtime_at_hour_start = 0;
  _dc_last_rtc_hour      = 255; // 255 = uninitialized

  dirty_contacts_expiry = 0;
  set_radio_at = revert_radio_at = 0;
  _logging = false;
  region_load_active = false;

#if MAX_NEIGHBOURS
      memset(neighbours, 0, sizeof(neighbours));
#endif

#ifdef ENABLE_CONSENSUS_TIME_SYNC
      memset(time_samples, 0, sizeof(time_samples));
      time_sample_idx = 0;
      next_time_sync = 0;
#endif

  // defaults
  memset(&_prefs, 0, sizeof(_prefs));
  _prefs.airtime_factor = 9.0;   // default to 10% duty cycle
  _prefs.rx_delay_base = 0.0f;   // turn off by default, was 10.0;
  _prefs.tx_delay_factor = 0.5f; // was 0.25f
  _prefs.direct_tx_delay_factor = 0.3f; // was 0.2
  StrHelper::strncpy(_prefs.node_name, ADVERT_NAME, sizeof(_prefs.node_name));
  _prefs.node_lat = ADVERT_LAT;
  _prefs.node_lon = ADVERT_LON;
  StrHelper::strncpy(_prefs.password, ADMIN_PASSWORD, sizeof(_prefs.password));
  _prefs.freq = LORA_FREQ;
  _prefs.sf = LORA_SF;
  _prefs.bw = LORA_BW;
  _prefs.cr = LORA_CR;
  _prefs.tx_power_dbm = LORA_TX_POWER;
  _prefs.advert_interval = 0;        // lusofw: disabled by default
  _prefs.flood_advert_interval = 24; // lusofw: 24h (upstream default: 47h)
  _prefs.flood_advert_base = 0.308f; // lusofw: probabilistic flood filter
  _prefs.flood_max = 64;
  _prefs.flood_max_unscoped = 64;    // v1.16.0: new unscoped flood limit
  _prefs.flood_max_advert = 8;       // v1.16.0: new advert hop limit (default 8)
  _prefs.interference_threshold = 14; // lusofw: listen before talk enabled

  // bridge defaults
  _prefs.bridge_enabled = 1;    // enabled
  _prefs.bridge_delay   = 500;  // milliseconds
  _prefs.bridge_pkt_src = 0;    // logTx
  _prefs.bridge_baud = 57600;   // baud rate
  _prefs.bridge_channel = 1;    // channel 1

  StrHelper::strncpy(_prefs.bridge_secret, "LVSITANOS", sizeof(_prefs.bridge_secret));

  // loop detect defaults
  _prefs.loop_detect = LOOP_DETECT_MINIMAL;

  // GPS defaults
  _prefs.gps_enabled = 0;
  _prefs.gps_interval = 0;
  _prefs.advert_loc_policy = ADVERT_LOC_PREFS;

  _prefs.adc_multiplier = 0.0f; // 0.0f means use default board multiplier

#if defined(USE_SX1262) || defined(USE_SX1268)
#ifdef SX126X_RX_BOOSTED_GAIN
  _prefs.rx_boosted_gain = SX126X_RX_BOOSTED_GAIN;
#else
  _prefs.rx_boosted_gain = 1; // enabled by default;
#endif
#endif

  pending_discover_tag = 0;
  pending_discover_until = 0;

  memset(default_scope.key, 0, sizeof(default_scope.key));
}

void MyMesh::begin(FILESYSTEM *fs) {
  mesh::Mesh::begin();
  _fs = fs;
  // load persisted prefs
  _cli.loadPrefs(_fs);

  char oldVersion[32];
  FirmwareMigration::readVersion(_fs, oldVersion, sizeof(oldVersion));
  if (strcmp(oldVersion, LUSOFW_FIRMWARE_VERSION) != 0) {
    FirmwareMigration::applyDefaultsByVersion(oldVersion, LUSOFW_FIRMWARE_VERSION, _prefs);
    _cli.savePrefs(_fs);
    FirmwareMigration::writeVersion(_fs, LUSOFW_FIRMWARE_VERSION);
  }

  acl.load(_fs, self_id);
  // TODO: key_store.begin();
  region_map.load(_fs);

  // Persistent reboot counter — read, increment, write back
  {
    uint32_t cnt = 1;
    File fr = _fs->open("/reboot_cnt");
    if (fr && fr.size() >= 4) {
      uint32_t saved = 0;
      fr.read((uint8_t*)&saved, 4);
      cnt = saved + 1;
    }
    if (fr) fr.close();
    _reboot_count = cnt;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    File fw = _fs->open("/reboot_cnt", FILE_O_WRITE);
#else
    File fw = _fs->open("/reboot_cnt", "w");
#endif
    if (fw) { fw.write((uint8_t*)&cnt, 4); fw.close(); }
  }

  // Baseline airtime snapshot for duty cycle window
  _airtime_window[0]    = getTotalAirTime();
  _airtime_window_idx   = 1;
  _airtime_window_count = 1;
  _next_airtime_snap    = futureMillis(300000); // first update in 5 min

  // establish default-scope
  {
    RegionEntry* r = region_map.getDefaultRegion();
    if (r) {
      region_map.getTransportKeysFor(*r, &default_scope, 1);
    } else {
#ifdef DEFAULT_FLOOD_SCOPE_NAME
      r = region_map.findByName(DEFAULT_FLOOD_SCOPE_NAME);
      if (r == NULL) {
        r = region_map.putRegion(DEFAULT_FLOOD_SCOPE_NAME, 0);  // auto-create the default scope region
        if (r) { r->flags = 0; }   // Allow-flood
      }
      if (r) {
        region_map.setDefaultRegion(r);
        region_map.getTransportKeysFor(*r, &default_scope, 1);
      }
#endif
    }
  }

  // Ensure default region exists and allow flood
  auto region = region_map.findByName("#portugal");

  if (!region) {
    region = region_map.putRegion("#portugal", region_map.getWildcard().id);
  }
  
  if (region) {
    region->flags &= ~REGION_DENY_FLOOD; // Always clear the deny flood flag to allow flooding
  }

#if defined(WITH_BRIDGE)
  if (_prefs.bridge_enabled) {
    bridge.begin();
  }
#endif

  radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
  radio_driver.setTxPower(_prefs.tx_power_dbm);

  radio_driver.setRxBoostedGainMode(_prefs.rx_boosted_gain);
  MESH_DEBUG_PRINTLN("RX Boosted Gain Mode: %s",
                     radio_driver.getRxBoostedGainMode() ? "Enabled" : "Disabled");

#ifndef DISABLE_LEGACY_ADVERT
  updateAdvertTimer();
  updateFloodAdvertTimer();
#endif

#ifdef ENABLE_CONSENSUS_TIME_SYNC
  next_time_sync = futureMillis(300000);
#endif

  board.setAdcMultiplier(_prefs.adc_multiplier);

#if ENV_INCLUDE_GPS == 1
  applyGpsPrefs();
#endif
}

void MyMesh::sendFloodScoped(const TransportKey& scope, mesh::Packet* pkt, uint32_t delay_millis, uint8_t path_hash_size) {
  if (scope.isNull()) {
    sendFlood(pkt, delay_millis, path_hash_size);
  } else {
    uint16_t codes[2];
    codes[0] = scope.calcTransportCode(pkt);
    codes[1] = 0;  // REVISIT: set to 'home' Region, for sender/return region?
    sendFlood(pkt, codes, delay_millis, path_hash_size);
  }
}

void MyMesh::applyTempRadioParams(float freq, float bw, uint8_t sf, uint8_t cr, int timeout_mins) {
  set_radio_at = futureMillis(2000); // give CLI reply some time to be sent back, before applying temp radio params
  pending_freq = freq;
  pending_bw = bw;
  pending_sf = sf;
  pending_cr = cr;

  revert_radio_at = futureMillis(2000 + timeout_mins * 60 * 1000); // schedule when to revert radio params
}

bool MyMesh::formatFileSystem() {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return InternalFS.format();
#elif defined(RP2040_PLATFORM)
  return LittleFS.format();
#elif defined(ESP32)
  return SPIFFS.format();
#else
#error "need to implement file system erase"
  return false;
#endif
}

void MyMesh::sendSelfAdvertisement(int delay_millis, bool flood) {
  mesh::Packet *pkt = createSelfAdvert();
  if (pkt) {
    if (flood) {
      sendFloodScoped(default_scope, pkt, delay_millis, _prefs.path_hash_mode + 1);
    } else {
      sendZeroHop(pkt, delay_millis);
    }
  } else {
    MESH_DEBUG_PRINTLN("ERROR: unable to create advertisement packet!");
  }
}

void MyMesh::updateAdvertTimer() {
#ifndef DISABLE_LEGACY_ADVERT
  if (_prefs.advert_interval > 0) { // schedule local advert timer
    next_local_advert = futureMillis(((uint32_t)_prefs.advert_interval) * 2 * 60 * 1000);
  } else {
    next_local_advert = 0; // stop the timer
  }
#else
  next_local_advert = 0; // stop the timer
  MESH_DEBUG_PRINTLN("Local advert timer disabled (DISABLE_LEGACY_ADVERT mode)");
#endif
}

void MyMesh::updateFloodAdvertTimer() {
#ifndef DISABLE_LEGACY_ADVERT
  if (_prefs.flood_advert_interval > 0) { // schedule flood advert timer
    next_flood_advert = futureMillis((uint32_t)(_prefs.flood_advert_interval) * 60 * 60 * 1000);
  } else {
    next_flood_advert = 0; // stop the timer
  }
#else
  const uint32_t interval_upper_bound =
      ((ADVERTS_ALLOWED_END - ADVERTS_ALLOWED_START + 1) * 60 * 60 / ADVERTS_ALLOWED_COUNT) - 60;
  const uint32_t interval_lower_bound = 60; // 1 minute

  uint32_t interval = getRNG()->nextInt(interval_lower_bound, interval_upper_bound);
  next_flood_advert = futureMillis((interval + next_flood_advert_offset) * 1000);

  MESH_DEBUG_PRINTLN(
      "%s updateFloodAdvertTimer: lower=%u, upper=%u, selected=%u, offset=%u (sec)",
      getLogDateTime(), interval_lower_bound, interval_upper_bound, interval, next_flood_advert_offset);

  next_flood_advert_offset = interval_upper_bound - interval;
#endif
}

void MyMesh::dumpLogFile() {
#if defined(RP2040_PLATFORM)
  File f = _fs->open(PACKET_LOG_FILE, "r");
#else
  File f = _fs->open(PACKET_LOG_FILE);
#endif
  if (f) {
    while (f.available()) {
      int c = f.read();
      if (c < 0) break;
      Serial.print((char)c);
    }
    f.close();
  }
}

void MyMesh::setTxPower(int8_t power_dbm) {
  radio_driver.setTxPower(power_dbm);
}

#if defined(USE_SX1262) || defined(USE_SX1268)
void MyMesh::setRxBoostedGain(bool enable) {
  radio_driver.setRxBoostedGainMode(enable);
}
#endif

void MyMesh::formatNeighborsReply(char *reply) {
  char *dp = reply;

#if MAX_NEIGHBOURS
  // create copy of neighbours list, skipping empty entries so we can sort it separately from main list
  int16_t neighbours_count = 0;
  NeighbourInfo* sorted_neighbours[MAX_NEIGHBOURS];
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    auto neighbour = &neighbours[i];
    if (neighbour->heard_timestamp > 0) {
      sorted_neighbours[neighbours_count] = neighbour;
      neighbours_count++;
    }
  }

  // sort neighbours newest to oldest
  std::sort(sorted_neighbours, sorted_neighbours + neighbours_count, [](const NeighbourInfo* a, const NeighbourInfo* b) {
    return a->heard_timestamp > b->heard_timestamp; // desc
  });

  for (int i = 0; i < neighbours_count && dp - reply < 134; i++) {
    NeighbourInfo *neighbour = sorted_neighbours[i];

    // add new line if not first item
    if (i > 0) *dp++ = '\n';

    char hex[10];
    // get 4 bytes of neighbour id as hex
    mesh::Utils::toHex(hex, neighbour->id.pub_key, 4);

    // add next neighbour
    uint32_t secs_ago = getRTCClock()->getCurrentTime() - neighbour->heard_timestamp;
    // SNR trend: '+' improving, '-' degrading, '=' stable (threshold: 1 dB = 4 units)
    char trend = '=';
    if (neighbour->snr_h1 != 0) {
      if      (neighbour->snr > neighbour->snr_h1 + 4) trend = '+';
      else if (neighbour->snr < neighbour->snr_h1 - 4) trend = '-';
    }
    sprintf(dp, "%s:%d:%d:%c", hex, secs_ago, neighbour->snr, trend);
    while (*dp)
      dp++; // find end of string
  }
#endif
  if (dp == reply) { // no neighbours, need empty response
    strcpy(dp, "-none-");
    dp += 6;
  }
  *dp = 0; // null terminator
}

void MyMesh::removeNeighbor(const uint8_t *pubkey, int key_len) {
#if MAX_NEIGHBOURS
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    NeighbourInfo *neighbour = &neighbours[i];
    if (memcmp(neighbour->id.pub_key, pubkey, key_len) == 0) {
      neighbours[i] = NeighbourInfo(); // clear neighbour entry
    }
  }
#endif
}

void MyMesh::startRegionsLoad() {
  temp_map.resetFrom(region_map);   // rebuild regions in a temp instance
  memset(load_stack, 0, sizeof(load_stack));
  load_stack[0] = &temp_map.getWildcard();
  region_load_active = true;
}

bool MyMesh::saveRegions() {
  return region_map.save(_fs);
}

void MyMesh::onDefaultRegionChanged(const RegionEntry* r) {
  if (r) {
    region_map.getTransportKeysFor(*r, &default_scope, 1);
  } else {
    memset(default_scope.key, 0, sizeof(default_scope.key));
  }
}

void MyMesh::formatStatsReply(char *reply) {
  StatsFormatHelper::formatCoreStats(reply, board, *_ms, _err_flags, _mgr);
}

void MyMesh::formatRadioStatsReply(char *reply) {
  StatsFormatHelper::formatRadioStats(reply, _radio, radio_driver, getTotalAirTime(), getReceiveAirTime());
}

void MyMesh::formatPacketStatsReply(char *reply) {
  StatsFormatHelper::formatPacketStats(reply, radio_driver, getNumSentFlood(), getNumSentDirect(),
                                       getNumRecvFlood(), getNumRecvDirect());
  // Append flood advert filter efficiency (acc=accepted, rej=rejected)
  size_t len = strlen(reply);
  uint32_t total = _flood_advert_accepted + _flood_advert_rejected;
  uint8_t  pct   = (total > 0) ? (uint8_t)(_flood_advert_rejected * 100 / total) : 0;
  if (len < 130) {
    snprintf(reply + len, 160 - len, "\nFF:+%lu -%lu (%u%% filt)",
             _flood_advert_accepted, _flood_advert_rejected, pct);
  }
}

void MyMesh::saveIdentity(const mesh::LocalIdentity &new_id) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  IdentityStore store(*_fs, "");
#elif defined(ESP32)
  IdentityStore store(*_fs, "/identity");
#elif defined(RP2040_PLATFORM)
  IdentityStore store(*_fs, "/identity");
#else
#error "need to define saveIdentity()"
#endif
  store.save("_main", new_id);
}

void MyMesh::clearStats() {
  radio_driver.resetStats();
  resetStats();
  ((SimpleMeshTables *)getTables())->resetStats();
}

void MyMesh::handleCommand(uint32_t sender_timestamp, char *command, char *reply) {
  if (region_load_active) {
    if (StrHelper::isBlank(command)) {  // empty/blank line, signal to terminate 'load' operation
      region_map = temp_map;  // copy over the temp instance as new current map
      region_load_active = false;

      sprintf(reply, "OK - loaded %d regions", region_map.getCount());
    } else {
      char *np = command;
      while (*np == ' ') np++;   // skip indent
      int indent = np - command;

      char *ep = np;
      while (RegionMap::is_name_char(*ep)) ep++;
      if (*ep) { *ep++ = 0; }  // set null terminator for end of name

      while (*ep && *ep != 'F') ep++;  // look for (optional) flags

      if (indent > 0 && indent < 8 && strlen(np) > 0) {
        auto parent = load_stack[indent - 1];
        if (parent) {
          auto old = region_map.findByName(np);
          auto nw = temp_map.putRegion(np, parent->id, old ? old->id : 0);  // carry-over the current ID (if name already exists)
          if (nw) {
            nw->flags = old ? old->flags : (*ep == 'F' ? 0 : REGION_DENY_FLOOD);   // carry-over flags from curr

            load_stack[indent] = nw;  // keep pointers to parent regions, to resolve parent_id's
          }
        }
      }
      reply[0] = 0;
    }
    return;
  }

  while (*command == ' ') command++; // skip leading spaces

  if (strlen(command) > 4 && command[2] == '|') { // optional prefix (for companion radio CLI)
    memcpy(reply, command, 3);                    // reflect the prefix back
    reply += 3;
    command += 3;
  }

  // handle ACL related commands
  if (memcmp(command, "setperm ", 8) == 0) {   // format:  setperm {pubkey-hex} {permissions-int8}
    char* hex = &command[8];
    char* sp = strchr(hex, ' ');   // look for separator char
    if (sp == NULL) {
      strcpy(reply, "Err - bad params");
    } else {
      *sp++ = 0;   // replace space with null terminator

      uint8_t pubkey[PUB_KEY_SIZE];
      int hex_len = min(sp - hex, PUB_KEY_SIZE*2);
      if (mesh::Utils::fromHex(pubkey, hex_len / 2, hex)) {
        uint8_t perms = atoi(sp);
        if (acl.applyPermissions(self_id, pubkey, hex_len / 2, perms)) {
          dirty_contacts_expiry = futureMillis(LAZY_CONTACTS_WRITE_DELAY);   // trigger acl.save()
          strcpy(reply, "OK");
        } else {
          strcpy(reply, "Err - invalid params");
        }
      } else {
        strcpy(reply, "Err - bad pubkey");
      }
    }
  } else if (sender_timestamp == 0 && strcmp(command, "get acl") == 0) {
    Serial.println("ACL:");
    for (int i = 0; i < acl.getNumClients(); i++) {
      auto c = acl.getClientByIdx(i);
      if (c->permissions == 0) continue;  // skip deleted (or guest) entries

      Serial.printf("%02X ", c->permissions);
      mesh::Utils::printHex(Serial, c->id.pub_key, PUB_KEY_SIZE);
      Serial.printf("\n");
    }
    reply[0] = 0;
  } else if (memcmp(command, "discover.neighbors", 18) == 0) {
    const char* sub = command + 18;
    while (*sub == ' ') sub++;
    if (*sub != 0) {
      strcpy(reply, "Err - discover.neighbors has no options");
    } else {
      sendNodeDiscoverReq();
      strcpy(reply, "OK - Discover sent");
    }
  } else{
    _cli.handleCommand(sender_timestamp, command, reply);  // common CLI commands
  }
}

// Centralised TX power application.
// Priority (highest wins): battery absolute limit > TPC relative offset.
// Temperature is always a further reduction applied on top of both.
// All power management code MUST call this instead of radio_driver.setTxPower() directly.
void MyMesh::applyEffectiveTxPower() {
  int8_t power = _prefs.tx_power_dbm;
#if defined(BATT_LOW_TX_POWER_DBM)
  if (_batt_tx_reduced) {
    power = BATT_LOW_TX_POWER_DBM;  // absolute override — battery critical
  } else {
    power += _tpc_offset_dbm;       // relative reduction
  }
#else
  power += _tpc_offset_dbm;
#endif
  if (_temp_tx_reduced) power -= TEMP_TX_REDUCE_DBM;
  radio_driver.setTxPower(power);
  MESH_DEBUG_PRINTLN("TX power: %d dBm (cfg=%d tpc=%d batt=%d temp=%d)",
                     power, _prefs.tx_power_dbm, _tpc_offset_dbm,
                     _batt_tx_reduced, _temp_tx_reduced);
}

void MyMesh::loop() {
#ifdef WITH_BRIDGE
  bridge.loop();
#endif

  mesh::Mesh::loop();

#ifndef DISABLE_LEGACY_ADVERT
  if (next_flood_advert && millisHasNowPassed(next_flood_advert)) {
    mesh::Packet *pkt = createSelfAdvert();
    uint32_t delay_millis = 0;
    if (pkt) sendFloodScoped(default_scope, pkt, delay_millis, _prefs.path_hash_mode + 1);

    updateFloodAdvertTimer(); // schedule next flood advert
    updateAdvertTimer();      // also schedule local advert (so they don't overlap)
  } else if (next_local_advert && millisHasNowPassed(next_local_advert)) {
    mesh::Packet *pkt = createSelfAdvert();
    if (pkt) sendZeroHop(pkt);

    updateAdvertTimer(); // schedule next local advert
  }
#else
  if (next_advert_check && millisHasNowPassed(next_advert_check)) {
    next_advert_check = futureMillis(10 * 1000); // check every 10 seconds

    uint32_t now = getRTCClock()->getCurrentTime();
    DateTime dt = DateTime(now);
    uint8_t current_hour = dt.hour();

    if (current_hour >= ADVERTS_ALLOWED_START && current_hour <= ADVERTS_ALLOWED_END) {
      MESH_DEBUG_PRINTLN("%s AdvertWindowCheck: hour=%d, adverts_sent=%d/%d, scheduled=%d, wait=%d",
                         getLogDateTime(), current_hour, adverts_sent, ADVERTS_ALLOWED_COUNT,
                         (next_flood_advert > 0), (next_flood_advert ? (int)(next_flood_advert - millis()) : 0));


      if (next_flood_advert && millisHasNowPassed(next_flood_advert)) {
        MESH_DEBUG_PRINTLN("%s Sending flood advert (%d/%d)", getLogDateTime(), adverts_sent + 1, ADVERTS_ALLOWED_COUNT);
        mesh::Packet *pkt = createSelfAdvert();
        if (pkt) sendFlood(pkt);
        next_flood_advert = 0;
        adverts_sent++;
      }

      if (adverts_sent >= ADVERTS_ALLOWED_COUNT) {
        // already sent max allowed adverts in this period
        MESH_DEBUG_PRINTLN("%s Max advert count reached (%d) for current period, skipping", getLogDateTime(),
                           ADVERTS_ALLOWED_COUNT);
        return;
      }

      // checks if flood adverts are disabled, or if we already have one scheduled, before scheduling next one
      if (next_flood_advert == 0 && _prefs.flood_advert_interval > 0) {
        updateFloodAdvertTimer();
      }
    } else if (adverts_sent > 0) {
      adverts_sent = 0;
      next_flood_advert = 0;
      next_flood_advert_offset = 0;
    }
  }
#endif

#ifdef ENABLE_CONSENSUS_TIME_SYNC
  if (next_time_sync && millisHasNowPassed(next_time_sync)) {
    applyTimeConsensus();
    next_time_sync = futureMillis(300000);
  }
#endif

  if (set_radio_at && millisHasNowPassed(set_radio_at)) { // apply pending (temporary) radio params
    set_radio_at = 0;                                     // clear timer
    radio_driver.setParams(pending_freq, pending_bw, pending_sf, pending_cr);
    MESH_DEBUG_PRINTLN("Temp radio params");
  }

  if (revert_radio_at && millisHasNowPassed(revert_radio_at)) { // revert radio params to orig
    revert_radio_at = 0;                                        // clear timer
    radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
    MESH_DEBUG_PRINTLN("Radio params restored");
  }

  // Boot advert: 3 zero-hop adverts at 60s/120s/180s after startup.
  // Three transmissions maximise probability of being heard by at least one
  // neighbour without meaningfully impacting duty cycle (3 × ~200ms ≪ 10%).
  if (_boot_advert_count < 3 && _next_boot_advert && millisHasNowPassed(_next_boot_advert)) {
    _boot_advert_count++;
    sendSelfAdvertisement(500, false); // zero-hop only — no flood, no network amplification
    MESH_DEBUG_PRINTLN("Boot advert %d/3 sent (zero-hop)", _boot_advert_count);
    _next_boot_advert = (_boot_advert_count < 3) ? futureMillis(60000) : 0;
  }

  // Boot node discover: 90s after startup, actively query nearby repeaters
  // to populate the neighbour table without waiting for their adverts.
  if (!_boot_discover_sent && _next_boot_discover && millisHasNowPassed(_next_boot_discover)) {
    _boot_discover_sent = true;
    _next_boot_discover = 0;
    sendNodeDiscoverReq();
    MESH_DEBUG_PRINTLN("Boot node discover sent");
  }

  // Isolation detection: if no packet received for ISOLATION_SILENCE_HOURS
  // and uptime is sufficient, send one emergency zero-hop advert.
  // Thresholds defined at top of file.
  // Clear cooldown flag when timer expires
  if (_isolation_advert_cooldown && millisHasNowPassed(_isolation_advert_cooldown)) {
    _isolation_advert_pending = false;
    _isolation_advert_cooldown = 0;
  }
  if (millisHasNowPassed(_next_isolation_check)) {
    _next_isolation_check = futureMillis(3600000); // re-check every hour
    uint32_t pkt_now = radio_driver.getPacketsRecv();
    if (pkt_now > _isolation_last_pkt_count) {
      _isolation_silence_count = 0; // traffic seen — not isolated
    } else {
      _isolation_silence_count++;
      MESH_DEBUG_PRINTLN("Isolation check: %d/%d hours of silence",
                         _isolation_silence_count, ISOLATION_SILENCE_HOURS);
    }
    _isolation_last_pkt_count = pkt_now;

    bool uptime_ok = (uptime_millis / 1000 >= ISOLATION_MIN_UPTIME_SECS);
    if (_isolation_silence_count >= ISOLATION_SILENCE_HOURS &&
        !_isolation_advert_pending && uptime_ok) {
      MESH_DEBUG_PRINTLN("Isolation detected: sending emergency zero-hop advert");
      sendSelfAdvertisement(500, false); // zero-hop only — respects duty cycle budget
      _isolation_advert_pending = true;
      _isolation_advert_cooldown = futureMillis(ISOLATION_ADVERT_COOLDOWN_MS);
      _isolation_silence_count = 0;
    }
  }

#if defined(BATT_LOW_THRESHOLD_MV) && defined(BATT_LOW_TX_POWER_DBM)
  // Battery-adaptive TX power: reduce TX when battery is low to extend runtime.
  // Only active when board reports a valid voltage (> 1000 mV).
  // Thresholds set via build flags: BATT_LOW_THRESHOLD_MV and BATT_LOW_TX_POWER_DBM.
  if (millisHasNowPassed(_next_batt_check)) {
    _next_batt_check = futureMillis(60000);
    uint16_t batt_mv = board.getBattMilliVolts();
    _cached_batt_mv  = batt_mv; // cache — fillDisplayStatus() reads this
    if (batt_mv > 1000) {
      bool is_low = (batt_mv < BATT_LOW_THRESHOLD_MV);
      if (is_low && !_batt_tx_reduced) {
        _batt_tx_reduced = true;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("Batt low (%u mV): TX power reduced", batt_mv);
      } else if (!is_low && _batt_tx_reduced) {
        _batt_tx_reduced = false;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("Batt recovered (%u mV): TX power restored", batt_mv);
      }
    }
  }
#endif

  // Radio watchdog: detect and recover from a stuck SX1262 RX state.
  // Condition: no packets received for 5 consecutive 2-minute windows (10 min total)
  // AND the STARTRX_TIMEOUT error flag is set.
  // Recovery: re-apply radio parameters (effectively re-initialises the radio).
  if (millisHasNowPassed(_next_radio_watchdog)) {
    _next_radio_watchdog = futureMillis(120000); // check every 2 minutes
    uint32_t pkt_now = radio_driver.getPacketsRecv();
    if (pkt_now > _radio_last_pkt_count) {
      // received at least one packet since last check — radio is healthy
      _radio_silence_count = 0;
    } else if (_err_flags & ERR_EVENT_STARTRX_TIMEOUT) {
      // no new packets AND error flag set — radio may be stuck
      _radio_silence_count++;
      MESH_DEBUG_PRINTLN("Radio watchdog: silence=%d/5, STARTRX_TIMEOUT set", _radio_silence_count);
      if (_radio_silence_count >= 5) {
        MESH_DEBUG_PRINTLN("Radio watchdog: triggering soft recovery after %d min silence",
                           (int)(_radio_silence_count * 2));
        radio_driver.setParams(_prefs.freq, _prefs.bw, _prefs.sf, _prefs.cr);
        radio_driver.setTxPower(_prefs.tx_power_dbm);
        radio_driver.setRxBoostedGainMode(_prefs.rx_boosted_gain);
        _err_flags &= ~ERR_EVENT_STARTRX_TIMEOUT; // clear the stuck flag
        _radio_silence_count = 0;
        MESH_DEBUG_PRINTLN("Radio watchdog: recovery complete");
      }
    } else {
      // no new packets but no error — legitimate quiet period, reset counter
      _radio_silence_count = 0;
    }
    _radio_last_pkt_count = pkt_now;
  }

  // is pending dirty contacts write needed?
  if (dirty_contacts_expiry && millisHasNowPassed(dirty_contacts_expiry)) {
    acl.save(_fs);
    dirty_contacts_expiry = 0;
  }

  // Temperature-aware TX power.
  // The nRF52840 MCU temperature sensor gives a proxy for enclosure temp.
  // Thresholds: TEMP_HIGH_C / TEMP_NORMAL_C / TEMP_TX_REDUCE_DBM (defined at top of file).
  if (millisHasNowPassed(_next_temp_check)) {
    _next_temp_check = futureMillis(300000); // re-check every 5 minutes
    float temp = board.getMCUTemperature();
    _cached_temperature = temp; // cache — fillDisplayStatus() reads this
    if (!isnan(temp)) {
      if (temp >= TEMP_HIGH_C && !_temp_tx_reduced) {
        _temp_tx_reduced = true;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("Temp high (%.1f C): TX reduced by %d dBm", temp, TEMP_TX_REDUCE_DBM);
      } else if (temp < TEMP_NORMAL_C && _temp_tx_reduced) {
        _temp_tx_reduced = false;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("Temp normal (%.1f C): TX restored", temp);
      }
    }
  }

  // Automatic Transmit Power Control (TPC) via neighbour SNR average.
  // Thresholds: TPC_SNR_HIGH / TPC_SNR_LOW / TPC_REDUCE_DBM (defined at top of file).
#if MAX_NEIGHBOURS
  if (millisHasNowPassed(_next_tpc_check)) {
    _next_tpc_check = futureMillis(1800000); // re-evaluate every 30 minutes
    uint32_t now_ts = getRTCClock()->getCurrentTime();
    int   active = 0;
    float snr_sum = 0.0f;
    for (int i = 0; i < MAX_NEIGHBOURS; i++) {
      if (neighbours[i].heard_timestamp > 0 &&
          now_ts >= neighbours[i].heard_timestamp &&
          now_ts - neighbours[i].heard_timestamp < NEIGHBOUR_EXPIRATION_SECS) {
        snr_sum += neighbours[i].snr / 4.0f; // stored as x4
        active++;
      }
    }
    if (active < 2) {
      // Not enough neighbours for reliable TPC — restore full power to maximise coverage
      if (_tpc_offset_dbm != 0) {
        _tpc_offset_dbm = 0;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("TPC: fewer than 2 active neighbours → restoring full TX power");
      }
    } else {
      float avg_snr = snr_sum / active;
      if (avg_snr >= TPC_SNR_HIGH && _tpc_offset_dbm == 0) {
        _tpc_offset_dbm = -TPC_REDUCE_DBM;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("TPC: avg SNR=%.1f dB (%d nbrs) → reducing TX by %d dBm",
                           avg_snr, active, TPC_REDUCE_DBM);
      } else if (avg_snr < TPC_SNR_LOW && _tpc_offset_dbm != 0) {
        _tpc_offset_dbm = 0;
        applyEffectiveTxPower();
        MESH_DEBUG_PRINTLN("TPC: avg SNR=%.1f dB (%d nbrs) → restoring full TX power",
                           avg_snr, active);
      }
    }
  }
#endif

  // Weekly maintenance reboot at WEEKLY_REBOOT_HOUR:WEEKLY_REBOOT_MIN UTC.
  // Thresholds defined at top of file.
  if (millisHasNowPassed(_next_reboot_check)) {
    _next_reboot_check = futureMillis(3600000); // check every hour
    uint32_t now_ts = getRTCClock()->getCurrentTime();
    if (now_ts > 1767225600) { // only if RTC is valid (post-2026)
      DateTime dt = DateTime(now_ts);
      bool in_reboot_window = (dt.hour() == WEEKLY_REBOOT_HOUR &&
                               dt.minute() >= WEEKLY_REBOOT_MIN &&
                               dt.minute() < WEEKLY_REBOOT_MIN + 55);
      bool uptime_ok = (uptime_millis / 1000 >= WEEKLY_REBOOT_MIN_UPTIME_SECS);
      if (in_reboot_window && uptime_ok) {
        MESH_DEBUG_PRINTLN("Weekly maintenance reboot at %02d:%02d UTC (uptime=%llu s)",
                           dt.hour(), dt.minute(), uptime_millis / 1000);
        _cli.savePrefs(_fs); // persist preferences
        acl.save(_fs);       // persist authenticated contacts (bug fix: was missing)
        board.reboot();
      }
    }
  }

  // Packet rate: compute packets received/sent in the last minute
  if (millisHasNowPassed(_next_rate_update)) {
    _next_rate_update = futureMillis(60000);
    uint32_t recv_now = radio_driver.getPacketsRecv();
    uint32_t sent_now = radio_driver.getPacketsSent();
    _pkt_rate_recv    = recv_now - _rate_last_recv;
    _pkt_rate_sent    = sent_now - _rate_last_sent;
    _rate_last_recv   = recv_now;
    _rate_last_sent   = sent_now;
  }

  // Duty cycle rolling window: snapshot getTotalAirTime() every 5 minutes.
  // Window spans the last 12 snapshots (= 60 minutes).
  if (millisHasNowPassed(_next_airtime_snap)) {
    _next_airtime_snap = futureMillis(300000);
    _airtime_window[_airtime_window_idx] = getTotalAirTime();
    _airtime_window_idx = (_airtime_window_idx + 1) % 12;
    if (_airtime_window_count < 12) _airtime_window_count++;
  }

  // Track last received packet time (for display "Last RX")
  {
    uint32_t pkt_now = radio_driver.getPacketsRecv();
    if (pkt_now > _last_rx_pkt_cnt) {
      _last_rx_millis = millis();
      _last_rx_pkt_cnt = pkt_now;
    }
  }

  // Hourly TX duty cycle history (updates when RTC hour changes)
  {
    uint32_t rtc_now = getRTCClock()->getCurrentTime();
    if (rtc_now > 1767225600UL) { // only when RTC valid
      DateTime dt = DateTime(rtc_now);
      uint8_t h = dt.hour();
      if (_dc_last_rtc_hour == 255) { // first run
        _airtime_at_hour_start = getTotalAirTime();
        _dc_last_rtc_hour = h;
      } else if (h != _dc_last_rtc_hour) { // hour changed
        uint32_t used_ms = getTotalAirTime() - _airtime_at_hour_start;
        // 0-100 where 100 = 10% duty cycle (ETSI limit)
        _hourly_dc_ms[_dc_last_rtc_hour] = (uint16_t)min((uint32_t)65535, used_ms);
        _airtime_at_hour_start = getTotalAirTime();
        _dc_last_rtc_hour = h;
      }
    }
  }

  // Memory watchdog: proactive reboot if heap is critically low
  if (millisHasNowPassed(_next_heap_check)) {
    _next_heap_check = futureMillis(1800000); // check every 30 min
    void* test = malloc(4096); // try to allocate 4KB
    if (test) {
      free(test);
      _heap_fail_count = 0;
    } else {
      _heap_fail_count++;
      MESH_DEBUG_PRINTLN("Memory watchdog: heap low! fail=%d/3", _heap_fail_count);
      if (_heap_fail_count >= 3) { // 3 failures = 1.5h of low memory
        MESH_DEBUG_PRINTLN("Memory watchdog: proactive reboot");
        _cli.savePrefs(_fs);
        acl.save(_fs);
        board.reboot();
      }
    }
  }

  // update uptime
  uint32_t now = millis();
  uptime_millis += now - last_millis;
  last_millis = now;
}

// To check if there is pending work
bool MyMesh::hasPendingWork() const {
#if defined(WITH_BRIDGE)
  if (bridge.isRunning()) return true;  // bridge needs WiFi radio, can't sleep
#endif
  return _mgr->getOutboundTotal() > 0;
}

// Fill display status struct for UITask — called every loop iteration
void MyMesh::fillDisplayStatus(RepeaterStatus& s) {
  s.uptime_secs     = (uint32_t)(uptime_millis / 1000);
  s.pkts_recv       = radio_driver.getPacketsRecv();
  s.pkts_sent       = radio_driver.getPacketsSent();
  s.last_snr        = radio_driver.getLastSNR();
  s.batt_mv         = _cached_batt_mv;
  s.batt_pct        = (_cached_batt_mv >= 1000) ? voltToBattPct(_cached_batt_mv) : 255;
  s.temperature     = _cached_temperature;
  s.tx_power_cfg    = _prefs.tx_power_dbm;
  s.tpc_offset_dbm  = _tpc_offset_dbm;
  s.batt_tx_reduced = _batt_tx_reduced;
  s.temp_tx_reduced = _temp_tx_reduced;
  s.flood_accepted  = _flood_advert_accepted;
  s.flood_rejected  = _flood_advert_rejected;
  s.isolated        = _isolation_advert_pending;

  // TX active indicator
  s.tx_active = (_mgr->getOutboundTotal() > 0);

  // Last RX time
  s.last_rx_secs_ago = (uint32_t)((millis() - _last_rx_millis) / 1000);

  // Hourly DC history: last 12 hours, oldest first (0-100 = 0-10% DC)
  for (int i = 0; i < 12; i++) {
    uint8_t slot = (uint8_t)((_dc_last_rtc_hour + 24 - 11 + i) % 24);
    uint32_t used = _hourly_dc_ms[slot];
    // Scale: 3600000ms per hour, 10% = 360000ms → 100 units
    s.hourly_dc_pct[i] = (uint8_t)min((uint32_t)100, used / 3600);
  }

  // Packet rate (computed every 60s in loop())
  s.pkt_rate_recv = _pkt_rate_recv;
  s.pkt_rate_sent = _pkt_rate_sent;

  // Duty cycle: delta airtime over the 60-min rolling window
  if (_airtime_window_count >= 2) {
    uint8_t  newest = (_airtime_window_idx + 11) % 12;
    uint8_t  oldest = (_airtime_window_idx + 12 - _airtime_window_count) % 12;
    uint32_t air_ms = _airtime_window[newest] - _airtime_window[oldest];
    uint32_t win_ms = (uint32_t)(_airtime_window_count - 1) * 300000UL;
    s.duty_cycle_pct = (win_ms > 0) ? ((float)air_ms / win_ms * 100.0f) : 0.0f;
  } else {
    s.duty_cycle_pct = 0.0f;
  }

  // Persistent reboot counter
  s.reboot_count = _reboot_count;

  // RTC time (valid only after clock sync)
  uint32_t now_rtc = getRTCClock()->getCurrentTime();
  s.rtc_valid = (now_rtc > 1767225600UL); // valid after 2026-01-01
  if (s.rtc_valid) {
    DateTime dt = DateTime(now_rtc);
    s.rtc_hour = dt.hour();
    s.rtc_min  = dt.minute();
  } else {
    s.rtc_hour = s.rtc_min = 0;
  }

  // Compute effective TX power (same logic as the loop adjustments)
  int8_t eff = _prefs.tx_power_dbm + _tpc_offset_dbm;
#if defined(BATT_LOW_TX_POWER_DBM)
  if (_batt_tx_reduced) eff = BATT_LOW_TX_POWER_DBM;
#endif
  if (_temp_tx_reduced) eff -= TEMP_TX_REDUCE_DBM;
  s.tx_power_eff = eff;

  // Count active neighbours
  uint8_t count = 0;
#if MAX_NEIGHBOURS
  uint32_t now_ts = getRTCClock()->getCurrentTime();
  for (int i = 0; i < MAX_NEIGHBOURS; i++) {
    if (neighbours[i].heard_timestamp > 0 &&
        now_ts >= neighbours[i].heard_timestamp &&
        now_ts - neighbours[i].heard_timestamp < NEIGHBOUR_EXPIRATION_SECS) {
      count++;
    }
  }
#endif
  s.neighbour_count = count;
}
