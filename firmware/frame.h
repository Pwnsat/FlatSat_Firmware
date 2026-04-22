/* src - frame.h
 *
 * spacecan_lib - By astrobyte 17/02/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#ifndef SPACECAN_LIB_FRAME_H
#define SPACECAN_LIB_FRAME_H

#include <stdint.h>
#include <string.h>

/**
 *
  Object	      | CAN ID (hex)  | Originator
  Heartbeat	    |   700	        | Controller
  Sync	        |   080	        | Controller
  SCET Time	    |   180	        | Controller
  UTC Time	    |   200	        | Controller
  Request (REQ)	| 280 + Node ID	| Controller
  Reply (REP)	  | 300 + Node ID	| Responder
 */

#define MAX_PACKET_SIZE 256
#define MAX_DATA_LEN 8
#define PAYLOAD_HEADER_LEN 2
#define MAX_CHUNK_LEN (MAX_DATA_LEN - PAYLOAD_HEADER_LEN)
#define SC_MAX_FRAGMENTS 43

#define SC_REASSEMBLY_TIMEOUT_MS 500

#define CAN_FULL_MASK 0x7FF     // FRAME ID = 11 bits
#define CAN_FUNCTION_MASK 0x780 // 111 1000 0000
#define CAN_NODE_MASK 0x07F

#define CANID_HEARTBEAT 0x700
#define CANID_SYNC 0x080
#define CANID_SCET_TIME 0x180
#define CANID_UTC_TIME 0x200
#define CANID_REQ 0x280
#define CANID_REP 0x300
#define CANID_TM 0x10
#define CANID_TC 0x20

typedef struct {
  uint32_t can_id;
  uint8_t dlc;
  uint8_t buffer[MAX_DATA_LEN];
} __attribute__((packed)) spacecan_frame_t;

typedef struct {
  uint8_t total_frames;
  uint8_t sequence_number;
  uint8_t payload[(MAX_DATA_LEN - PAYLOAD_HEADER_LEN)];
} __attribute__((packed)) spacecan_packet_fragment_t;

typedef struct {
  uint8_t active;
  uint8_t total_frames;
  uint32_t last_can_id;
  uint8_t last_seq;
  uint8_t received_mask[SC_MAX_FRAGMENTS];
  uint8_t fragment_sizes[SC_MAX_FRAGMENTS];
  uint8_t buffer[MAX_PACKET_SIZE];
  uint32_t last_update_ms;
} __attribute__((packed)) spacecan_reassembly_ctx_t;

static inline uint16_t sc_frame_id_req(uint8_t node_id) {
  return CANID_REQ + (node_id & CAN_NODE_MASK);
}
static inline uint16_t sc_frame_id_rep(uint8_t node_id) {
  return CANID_REP + (node_id & CAN_NODE_MASK);
}
static inline uint8_t sc_frame_get_id_rep(uint16_t frame_id) {
  return (frame_id - CANID_REP) & CAN_NODE_MASK;
}

int sc_build_request(spacecan_frame_t *frame, uint8_t node_id,
                     const uint8_t *payload, size_t len);
int sc_build_reply(spacecan_frame_t *frame, uint8_t node_id,
                   const uint8_t *payload, size_t len);
int sc_fragment_packet(uint16_t can_id, const uint8_t *payload, size_t len,
                       spacecan_frame_t *frames, size_t *frame_count);
void sc_reassembly_init(spacecan_reassembly_ctx_t *ctx);
void sc_reassembly_reset(spacecan_reassembly_ctx_t *ctx);
int sc_reassembly_packets(spacecan_reassembly_ctx_t *ctx,
                          const spacecan_frame_t *frame, uint32_t now_ms,
                          uint8_t *out_buffer, size_t *out_len);

int sc_build_heartbeat(spacecan_frame_t *frame, uint8_t node_id, uint8_t state);
int sc_build_sync(spacecan_frame_t *frame, uint8_t node_id);
#endif // SPACECAN_LIB_FRAME_H
