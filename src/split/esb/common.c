/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "common.h"

#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

static int rel_axis_index(uint16_t code) {
    switch (code) {
    case INPUT_REL_X:
        return 0;
    case INPUT_REL_Y:
        return 1;
    case INPUT_REL_WHEEL:
        return 2;
    case INPUT_REL_HWHEEL:
        return 3;
    default:
        return -1;
    }
}

bool zmk_split_esb_classify_rel(const uint8_t *buf, size_t len, int32_t deltas[ESB_REL_AXES]) {
    int32_t acc[ESB_REL_AXES] = {0};
    size_t off = 0;
    bool any = false;
    bool movement_only = true;

    while (off + sizeof(struct esb_msg_prefix) <= len) {
        struct esb_msg_prefix prefix;
        memcpy(&prefix, &buf[off], sizeof(prefix));

        if (memcmp(prefix.magic_prefix, ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                   sizeof(prefix.magic_prefix)) != 0) {
            movement_only = false;
            break;
        }

        size_t env_len = sizeof(struct esb_msg_prefix) + prefix.payload_size;
        if (off + env_len + sizeof(struct esb_msg_postfix) > len) {
            movement_only = false;
            break;
        }

        /* KEY_STATE packets carry the discriminant in the high bit of source. */
        uint8_t source = buf[off + sizeof(struct esb_msg_prefix)];
        if ((source & ESB_SOURCE_KEY_STATE_FLAG) != 0) {
            movement_only = false;
            break;
        }

        struct esb_event_envelope env;
        memset(&env, 0, sizeof(env));
        memcpy(&env, &buf[off], MIN(env_len, sizeof(env)));

        if (env.payload.event.type != ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT ||
            env.payload.event.data.input_event.type != INPUT_EV_REL) {
            movement_only = false;
            break;
        }

        int idx = rel_axis_index(env.payload.event.data.input_event.code);
        if (idx < 0) {
            movement_only = false;
            break;
        }

        acc[idx] += env.payload.event.data.input_event.value;
        any = true;
        off += env_len + sizeof(struct esb_msg_postfix);
    }

    movement_only = movement_only && any && (off == len);

    if (deltas) {
        if (movement_only) {
            memcpy(deltas, acc, sizeof(acc));
        } else {
            memset(deltas, 0, sizeof(int32_t) * ESB_REL_AXES);
        }
    }
    return movement_only;
}

/* FIFO of per-payload movement deltas, one slot per payload written to ESB and
 * not yet acked/failed.  Bounded by the ESB TX FIFO depth; on the (unexpected)
 * overflow the oldest is reconciled as lost rather than dropped. */
#define ESB_INFLIGHT_FIFO_LEN 32

static struct {
    int32_t d[ESB_REL_AXES];
} inflight_q[ESB_INFLIGHT_FIFO_LEN];
static uint8_t inflight_head;
static uint8_t inflight_tail;
static uint8_t inflight_count;

/* Accumulated lost movement awaiting merge into the next movement event. */
static int32_t pending_rel[ESB_REL_AXES];

static void fold_into_pending(const int32_t d[ESB_REL_AXES]) {
    for (int i = 0; i < ESB_REL_AXES; i++) {
        int32_t v = pending_rel[i] + d[i];
        v = CLAMP(v, -ESB_REL_ACCUM_CLAMP, ESB_REL_ACCUM_CLAMP);
        pending_rel[i] = v;
    }
}

void zmk_split_esb_inflight_push(const int32_t deltas[ESB_REL_AXES]) {
    unsigned int key = irq_lock();
    if (inflight_count == ESB_INFLIGHT_FIFO_LEN) {
        fold_into_pending(inflight_q[inflight_head].d);
        inflight_head = (inflight_head + 1) % ESB_INFLIGHT_FIFO_LEN;
        inflight_count--;
    }
    memcpy(inflight_q[inflight_tail].d, deltas, sizeof(int32_t) * ESB_REL_AXES);
    inflight_tail = (inflight_tail + 1) % ESB_INFLIGHT_FIFO_LEN;
    inflight_count++;
    irq_unlock(key);
}

void zmk_split_esb_inflight_resolve(bool failed) {
    unsigned int key = irq_lock();
    if (inflight_count == 0) {
        irq_unlock(key);
        return;
    }
    if (failed) {
        fold_into_pending(inflight_q[inflight_head].d);
    }
    inflight_head = (inflight_head + 1) % ESB_INFLIGHT_FIFO_LEN;
    inflight_count--;
    irq_unlock(key);
}

void zmk_split_esb_inflight_reset(void) {
    unsigned int key = irq_lock();
    while (inflight_count > 0) {
        fold_into_pending(inflight_q[inflight_head].d);
        inflight_head = (inflight_head + 1) % ESB_INFLIGHT_FIFO_LEN;
        inflight_count--;
    }
    irq_unlock(key);
}

int32_t zmk_split_esb_take_pending_rel(uint16_t code) {
    int idx = rel_axis_index(code);
    if (idx < 0) {
        return 0;
    }
    unsigned int key = irq_lock();
    int32_t v = pending_rel[idx];
    pending_rel[idx] = 0;
    irq_unlock(key);
    return v;
}

void zmk_split_esb_async_tx(struct zmk_split_esb_async_state *state) {
    size_t tx_buf_len = ring_buf_size_get(state->tx_buf);
    // LOG_DBG("tx_buf_len %u, CONFIG_ESB_MAX_PAYLOAD_LENGTH %u", 
    //         tx_buf_len, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
    if (!tx_buf_len || tx_buf_len > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
        return;
    }
    // LOG_DBG("tx_buf_len %d", tx_buf_len);

    uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
    size_t claim_len = 0;
    while (claim_len < tx_buf_len) {
        uint8_t *b;
        uint32_t buf_len = ring_buf_get_claim(state->tx_buf, &b, tx_buf_len - claim_len);
        if (buf_len <= 0) {
            break;
        }
        memcpy(&buf[claim_len], b, buf_len);
        claim_len += buf_len;
    }
    if (claim_len <= 0) {
        return;
    }
    // LOG_DBG("tx_buf_len: %d, claim_len: %d", tx_buf_len, claim_len);
    // LOG_HEXDUMP_DBG(buf, claim_len, "buf");

    static app_esb_data_t my_data;
    my_data.data = buf;
    my_data.len = claim_len;
    /* Request an ACK for every packet when enabled; the per-packet retransmit
     * decision (single-shot for movement vs. retransmit for the rest) is made
     * at write time in pull_packet_from_tx_msgq(). */
    my_data.noack = !IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK);
    zmk_split_esb_send(&my_data); // callback > zmk_split_esb_cb()

    // LOG_DBG("ESB TX Buf finish %d", claim_len);
    ring_buf_get_finish(state->tx_buf, claim_len);
}

static K_SEM_DEFINE(esb_cb_sem, 1, 1);

void zmk_split_esb_cb(app_esb_event_t *event, struct zmk_split_esb_async_state *state) {
    switch(event->evt_type) {
        case APP_ESB_EVT_TX_SUCCESS:
            // LOG_DBG("ESB TX sent");
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_async_tx(state);
            }
            break;
        case APP_ESB_EVT_TX_FAIL:
            // LOG_WRN("ESB TX failed");
            if (!ring_buf_is_empty(state->tx_buf)) {
                zmk_split_esb_async_tx(state);
            }
            break;
        case APP_ESB_EVT_RX: {
            // LOG_DBG("ESB RX received: %d", event->data_length);

            // lock it for a safe result from ring_buf_space_get()
            int ret = k_sem_take(&esb_cb_sem, K_FOREVER);
            if (ret) {
                LOG_WRN("Shouldn't be called FOREVER");
                break;
            }

            if (ring_buf_space_get(state->rx_buf) < event->data_length) {
                LOG_WRN("No room to receive from peripheral (have %d but only space for %d/%d)",
                        event->data_length, ring_buf_space_get(state->rx_buf),
                        ring_buf_capacity_get(state->rx_buf));
                k_sem_give(&esb_cb_sem);
                break;
            }

            size_t received = ring_buf_put(state->rx_buf, event->buf, event->data_length);
            if (received < event->data_length) {
                LOG_ERR("RX overrun! %d < %d", received, event->data_length);
                k_sem_give(&esb_cb_sem);
                break;
            }

            k_sem_give(&esb_cb_sem);

            // LOG_DBG("RX + %3d and now buffer is %3d", received, ring_buf_size_get(state->rx_buf));
            if (state->process_tx_callback) {
                state->process_tx_callback();
            } else if (state->process_tx_work) {
                k_work_submit(state->process_tx_work);
            }

            break;
        }
        default:
            LOG_ERR("Unknown APP ESB event!");
            break;
    }
}

int zmk_split_esb_get_item(struct ring_buf *rx_buf, uint8_t *env, size_t env_size) {
    while (ring_buf_size_get(rx_buf) > sizeof(struct esb_msg_prefix) + sizeof(struct esb_msg_postfix)) {
        struct esb_msg_prefix prefix;

        __ASSERT_EVAL(
            (void)ring_buf_peek(rx_buf, (uint8_t *)&prefix, sizeof(prefix)),
            uint32_t peek_read = ring_buf_peek(rx_buf, (uint8_t *)&prefix, sizeof(prefix)),
            peek_read == sizeof(prefix), "Somehow read less than we expect from the RX buffer");

        if (memcmp(&prefix.magic_prefix, &ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                   sizeof(prefix.magic_prefix)) != 0) {
            uint8_t discarded_byte;
            ring_buf_get(rx_buf, &discarded_byte, 1);

            LOG_WRN("Prefix mismatch, discarding byte %0x", discarded_byte);
            
            continue;
        }

        size_t payload_to_read = sizeof(prefix) + prefix.payload_size;

        if (payload_to_read > env_size) {
            LOG_WRN("Invalid message with payload %d bigger than expected max %d", payload_to_read,
                    env_size);
            return -EINVAL;
        }

        if (ring_buf_size_get(rx_buf) < payload_to_read + sizeof(struct esb_msg_postfix)) {
            return -EAGAIN;
        }

        // Now that prefix matches, read it out so we can read the rest of the payload.
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, env, payload_to_read),
                      uint32_t read = ring_buf_get(rx_buf, env, payload_to_read),
                      read == payload_to_read,
                      "Somehow read less than we expect from the RX buffer");

        struct esb_msg_postfix postfix;
        __ASSERT_EVAL((void)ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
                      uint32_t read = ring_buf_get(rx_buf, (uint8_t *)&postfix, sizeof(postfix)),
                      read == sizeof(postfix),
                      "Somehow read less of the postfix than we expect from the RX buffer");

        // LOG_HEXDUMP_DBG(&postfix, sizeof(postfix), "postfix");

        uint32_t crc = crc32_ieee(env, payload_to_read);

        if (crc != postfix.crc) {
            LOG_WRN("Data corruption in received peripheral event, ignoring %d vs %d", crc,
                    postfix.crc);
            return -EINVAL;
        }

        return 0;
    }

    return -EAGAIN;
}
