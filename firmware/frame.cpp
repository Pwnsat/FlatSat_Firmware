/* src - frame.cpp
 *
 * spacecan_lib - By astrobyte 17/02/26.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 */
#include "frame.h"
#include <stdio.h>

int sc_build_request(spacecan_frame_t *frame, uint8_t node_id,
                     const uint8_t *payload, size_t len) {
  if (len > MAX_DATA_LEN) {
    return -1;
  }

  frame->can_id = sc_frame_id_req(node_id);
  frame->dlc = len;
  memset(frame->buffer, 0, MAX_DATA_LEN);
  memcpy(frame->buffer, payload, len);
  return 0;
}

int sc_build_reply(spacecan_frame_t *frame, uint8_t node_id,
                   const uint8_t *payload, size_t len) {
  if (len > MAX_DATA_LEN) {
    return -1;
  }

  frame->can_id = sc_frame_id_rep(node_id);
  frame->dlc = len;
  memset(frame->buffer, 0, MAX_DATA_LEN);
  memcpy(frame->buffer, payload, len);
  return 0;
}

int sc_fragment_packet(uint16_t can_id, const uint8_t *payload, size_t len,
                       spacecan_frame_t *frames, size_t *frame_count) {
  if (len == 0 || len > MAX_PACKET_SIZE) {
    return -1;
  }

  size_t needed = (len + MAX_CHUNK_LEN - 1) / MAX_CHUNK_LEN;
  if (needed == 0) {
    return -2;
  }

  *frame_count = needed;

  for (size_t i = 0; i < needed; i++) {
    size_t offset = i * MAX_CHUNK_LEN;
    size_t chunk =
        (len - offset >= MAX_CHUNK_LEN) ? MAX_CHUNK_LEN : (len - offset);

    frames[i].can_id = can_id;
    frames[i].dlc = PAYLOAD_HEADER_LEN + chunk;

    frames[i].buffer[0] = needed - 1; // Total frames - 1
    frames[i].buffer[1] = i;          // Sequence

    memset(&frames[i].buffer[2], 0, MAX_CHUNK_LEN);
    memcpy(&frames[i].buffer[2], &payload[offset], chunk);
  }

  return 0;
}

void sc_reassembly_init(spacecan_reassembly_ctx_t *ctx) {
  memset(ctx, 0, sizeof(*ctx));
}

void sc_reassembly_reset(spacecan_reassembly_ctx_t *ctx) {
  ctx->active = 0;
  memset(ctx->received_mask, 0, sizeof(ctx->received_mask));
}

int sc_reassembly_packets(spacecan_reassembly_ctx_t *ctx,
                          const spacecan_frame_t *frame, uint32_t now_ms,
                          uint8_t *out_buffer, size_t *out_len) {
  if (frame->dlc < 2) {
    return -1;
  }

  uint8_t total = frame->buffer[0] + 1;
  uint8_t seq = frame->buffer[1];

  if (total == 0 || total > SC_MAX_FRAGMENTS) {
    return -1;
  }

  if (!ctx->active) {
    ctx->active = 1;
    ctx->total_frames = total;
    memset(ctx->received_mask, 0, sizeof(ctx->received_mask));
  }

  if (now_ms - ctx->last_update_ms > SC_REASSEMBLY_TIMEOUT_MS) {
    sc_reassembly_reset(ctx);
  }

  if (seq >= ctx->total_frames) {
    return -1;
  }

  size_t payload_len = frame->dlc - PAYLOAD_HEADER_LEN;

  if (frame->can_id == ctx->last_can_id && frame->buffer[1] == ctx->last_seq) {
    return 0;
  }

  ctx->last_can_id = frame->can_id;
  ctx->last_seq = frame->buffer[1];
  ctx->last_update_ms = now_ms;
  ctx->received_mask[seq] = 1;
  ctx->fragment_sizes[seq] = payload_len;

  size_t offset = seq * MAX_CHUNK_LEN;

  memcpy(&ctx->buffer[offset], &frame->buffer[PAYLOAD_HEADER_LEN], payload_len);

  size_t final_size = 0;
  for (uint8_t i = 0; i < ctx->total_frames; i++) {
    if (!ctx->received_mask[i]) {
      return 0;
    }
    final_size += ctx->fragment_sizes[i];
  }

  memcpy(out_buffer, ctx->buffer, final_size);
  *out_len = final_size;

  sc_reassembly_reset(ctx);

  return 1;
}

int sc_build_heartbeat(spacecan_frame_t *frame, uint8_t node_id,
                       uint8_t state) {
  frame->can_id = CANID_HEARTBEAT | (node_id & CAN_NODE_MASK);
  frame->dlc = 1;
  frame->buffer[0] = state;
  return 0;
}

int sc_build_sync(spacecan_frame_t *frame, uint8_t node_id) {
  frame->can_id = CANID_SYNC | (node_id & CAN_NODE_MASK);
  frame->dlc = 1;
  return 0;
}
