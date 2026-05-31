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

int zmk_split_esb_rel_axis_index(uint16_t code) {
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

struct esb_rel_class zmk_split_esb_classify_rel(const uint8_t *buf, size_t len) {
    struct esb_rel_class cls = {.movement_only = false, .contains_movement = false};
    size_t off = 0;
    bool any = false;
    bool only = true;

    while (off + sizeof(struct esb_msg_prefix) <= len) {
        struct esb_msg_prefix prefix;
        memcpy(&prefix, &buf[off], sizeof(prefix));

        if (memcmp(prefix.magic_prefix, ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
                   sizeof(prefix.magic_prefix)) != 0) {
            only = false;
            break;
        }

        size_t env_len = sizeof(struct esb_msg_prefix) + prefix.payload_size;
        if (off + env_len + sizeof(struct esb_msg_postfix) > len) {
            only = false;
            break;
        }

        /* KEY_STATE packets carry the discriminant in the high bit of source. */
        uint8_t source = buf[off + sizeof(struct esb_msg_prefix)];
        bool is_movement = false;
        if ((source & ESB_SOURCE_KEY_STATE_FLAG) == 0) {
            struct esb_event_envelope env;
            memset(&env, 0, sizeof(env));
            memcpy(&env, &buf[off], MIN(env_len, sizeof(env)));

            is_movement =
                env.payload.event.type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT &&
                env.payload.event.data.input_event.type == INPUT_EV_REL &&
                zmk_split_esb_rel_axis_index(env.payload.event.data.input_event.code) >= 0;
        }

        if (is_movement) {
            cls.contains_movement = true;
        } else {
            only = false;
        }
        any = true;
        off += env_len + sizeof(struct esb_msg_postfix);
    }

    cls.movement_only = only && any && (off == len);
    return cls;
}

/* FIFO of per-payload movement flags, one slot per payload written to ESB and
 * not yet acked/failed, resolved in TX-callback order.  Bounded by the ESB TX
 * FIFO depth. */
#define ESB_INFLIGHT_FIFO_LEN 32

static bool inflight_q[ESB_INFLIGHT_FIFO_LEN];
static uint8_t inflight_head;
static uint8_t inflight_tail;
static uint8_t inflight_count;

static zmk_split_esb_movement_done_cb_t m_movement_done_cb;

void zmk_split_esb_register_movement_done_cb(zmk_split_esb_movement_done_cb_t cb) {
    m_movement_done_cb = cb;
}

static void notify_movement_done(bool failed) {
    if (m_movement_done_cb) {
        m_movement_done_cb(failed);
    }
}

void zmk_split_esb_inflight_push(bool movement) {
    unsigned int key = irq_lock();
    bool overflow_movement = false;
    if (inflight_count == ESB_INFLIGHT_FIFO_LEN) {
        overflow_movement = inflight_q[inflight_head];
        inflight_head = (inflight_head + 1) % ESB_INFLIGHT_FIFO_LEN;
        inflight_count--;
    }
    inflight_q[inflight_tail] = movement;
    inflight_tail = (inflight_tail + 1) % ESB_INFLIGHT_FIFO_LEN;
    inflight_count++;
    irq_unlock(key);

    if (overflow_movement) {
        notify_movement_done(true);
    }
}

void zmk_split_esb_inflight_resolve(bool failed) {
    unsigned int key = irq_lock();
    bool movement = false;
    bool had = inflight_count > 0;
    if (had) {
        movement = inflight_q[inflight_head];
        inflight_head = (inflight_head + 1) % ESB_INFLIGHT_FIFO_LEN;
        inflight_count--;
    }
    irq_unlock(key);

    if (had && movement) {
        notify_movement_done(failed);
    }
}

void zmk_split_esb_inflight_reset(void) {
    for (;;) {
        unsigned int key = irq_lock();
        if (inflight_count == 0) {
            irq_unlock(key);
            break;
        }
        bool movement = inflight_q[inflight_head];
        inflight_head = (inflight_head + 1) % ESB_INFLIGHT_FIFO_LEN;
        inflight_count--;
        irq_unlock(key);

        if (movement) {
            notify_movement_done(true);
        }
    }
}

void zmk_split_esb_movement_lost(void) { notify_movement_done(true); }

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
    int sret = zmk_split_esb_send(&my_data); // callback > zmk_split_esb_cb()
    if (sret != 0) {
        /* Not queued (msgq full): the payload is dropped here and will never
         * get a TX callback, so account for any movement it carried. */
        if (zmk_split_esb_classify_rel(buf, claim_len).contains_movement) {
            zmk_split_esb_movement_lost();
        }
    }

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
