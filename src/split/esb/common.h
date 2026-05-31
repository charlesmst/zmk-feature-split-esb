/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>

#include <zmk/split/transport/types.h>
#include "app_esb.h"

#define ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX "ZmKe"

/* High bit of the source field in esb_key_state_payload signals a key+button
 * state packet vs a generic event packet.  Peripheral IDs are always < 128,
 * so BIT(7) is free for use as a discriminant without adding a header byte.
 *
 * KEY_STATE packets carry a full bitmap of pressed key positions (position N →
 * byte N/8 bit N%8) plus a button-state byte (bit i = INPUT_BTN_0 + i).
 * Sending full state on every change makes the data idempotent over ESB:
 * duplicates XOR to zero diff and emit no phantom events.
 */
#define ESB_SOURCE_KEY_STATE_FLAG BIT(7)

struct esb_msg_prefix {
    uint8_t magic_prefix[sizeof(ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX) - 1];
    uint8_t payload_size;
} __packed;

struct esb_command_payload {
    uint8_t source;
    struct zmk_split_transport_central_command cmd;
} __packed;

struct esb_command_envelope {
    struct esb_msg_prefix prefix;
    struct esb_command_payload payload;
} __packed;

struct esb_event_payload {
    uint8_t source;
    struct zmk_split_transport_peripheral_event event;
} __packed;

struct esb_event_envelope {
    struct esb_msg_prefix prefix;
    struct esb_event_payload payload;
} __packed;

/* 128 key positions encoded as a bitmask. */
#define ESB_KEY_STATE_LEN 16

/* Button bitmap: bit i corresponds to INPUT_BTN_0 + i (up to 8 buttons). */
#define ESB_BTN_STATE_LEN 8

struct esb_key_state_payload {
    uint8_t source;               /* peripheral_id | ESB_SOURCE_KEY_STATE_FLAG */
    uint8_t state[ESB_KEY_STATE_LEN];
    uint8_t button_state;         /* bit i = INPUT_BTN_0 + i pressed */
} __packed;

struct esb_key_state_envelope {
    struct esb_msg_prefix prefix;
    struct esb_key_state_payload payload;
} __packed;

struct esb_msg_postfix {
    uint32_t crc;
} __packed;

#define ESB_MSG_EXTRA_SIZE (sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix))

typedef void (*zmk_split_esb_process_tx_callback_t)(void);

struct zmk_split_esb_async_state {
    atomic_t state;

    uint8_t *rx_bufs[2];
    size_t rx_bufs_len;
    size_t rx_size_process_trigger;

    struct ring_buf *tx_buf;
    struct ring_buf *rx_buf;

    zmk_split_esb_process_tx_callback_t process_tx_callback;

    const struct device *uart;

    struct k_work_delayable restart_rx_work;
    struct k_work *process_tx_work;
    const struct gpio_dt_spec *dir_gpio;
};

void zmk_split_esb_async_tx(struct zmk_split_esb_async_state *state);

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state);

int zmk_split_esb_get_item(struct ring_buf *rx_buf, uint8_t *env, size_t env_size);

/* --- Coalesce-on-pull movement transport ------------------------------------
 *
 * Like a real polled mouse: rather than queueing one packet per movement
 * report, the peripheral keeps a single per-axis accumulator and lets at most
 * ONE movement packet be outstanding at a time.  New motion (and any motion
 * from a failed packet) just sums into the accumulator; the accumulator is
 * drained into one fresh packet whenever the radio is free again (the previous
 * movement packet's TX callback, or a flush/suspend).  Movement is sent with
 * ACK and retransmit count 0 (single-shot); key/button state and sensor/battery
 * keep the normal ACK + retransmit reliability.
 *
 * This module owns the in-flight bookkeeping (which is generic to the radio);
 * the per-axis accumulator and packet building live in the peripheral, driven
 * through the movement-done callback below.
 */

/* Relative axes we coalesce: X, Y, WHEEL, HWHEEL. */
#define ESB_REL_AXES 4

/* Upper bound on accumulated-but-unsent motion per axis, so a long RF outage
 * cannot produce a large cursor jump on recovery. */
#define ESB_REL_ACCUM_CLAMP 1024

/* Index for a relative axis code, or -1 if not tracked. */
int zmk_split_esb_rel_axis_index(uint16_t code);

struct esb_rel_class {
    bool movement_only;     /* every envelope is REL movement -> single-shot TX */
    bool contains_movement; /* >=1 REL movement envelope -> notify movement lane */
};

/* Classify a built TX payload by walking its envelopes. */
struct esb_rel_class zmk_split_esb_classify_rel(const uint8_t *buf, size_t len);

/* Notified when the outstanding movement packet should be considered finished:
 * `failed` == true means it was not delivered (lost ack, dropped, or suspend).
 *
 * NOTE: ESB coalesces TX completion flags, so TX events are NOT 1:1 with
 * payloads and cannot be counted.  This is therefore driven off the radio
 * becoming idle (whole TX FIFO drained) rather than per-packet correlation,
 * and the peripheral guards it against spurious calls with its own
 * mv_outstanding flag + a watchdog. */
typedef void (*zmk_split_esb_movement_done_cb_t)(bool failed);
void zmk_split_esb_register_movement_done_cb(zmk_split_esb_movement_done_cb_t cb);

/* Fire the registered movement-done callback (no-op if none registered). */
void zmk_split_esb_movement_complete(bool failed);
