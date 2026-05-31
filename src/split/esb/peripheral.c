/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "app_esb.h"
#include "common.h"

#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_event_envelope) + sizeof(struct esb_msg_postfix)) *                        \
     CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS)
#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix)) *                      \
     CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS)

RING_BUF_DECLARE(chosen_rx_buf, RX_BUFFER_SIZE);
RING_BUF_DECLARE(chosen_tx_buf, TX_BUFFER_SIZE);

static K_SEM_DEFINE(esb_send_evt_sem, 1, 1);

static const uint8_t peripheral_id = CONFIG_ZMK_SPLIT_ESB_PERIPHERAL_ID;

/* Pressed-key bitmap, mirroring the BLE split's position_state[]. */
static uint8_t position_state[ESB_KEY_STATE_LEN];

/* Button bitmap: bit i = INPUT_BTN_0 + i is pressed. */
static uint8_t button_state;

static void publish_commands_work(struct k_work *work);

K_WORK_DEFINE(publish_commands, publish_commands_work);

static void process_tx_cb(void);
K_MSGQ_DEFINE(cmd_msg_queue, sizeof(struct zmk_split_transport_central_command), 3, 4);

uint8_t async_rx_buf[RX_BUFFER_SIZE / 2][2];

static struct zmk_split_esb_async_state async_state = {
    .rx_bufs = {async_rx_buf[0], async_rx_buf[1]},
    .rx_bufs_len = RX_BUFFER_SIZE / 2,
    .rx_size_process_trigger = sizeof(struct esb_command_envelope),
    .process_tx_callback = process_tx_cb,
    .rx_buf = &chosen_rx_buf,
    .tx_buf = &chosen_tx_buf,
};

static void begin_tx(void) {
    zmk_split_esb_async_tx(&async_state);
}

void zmk_split_esb_on_ptx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_peripheral_event *evt) {
    switch (evt->type) {
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT:
        return sizeof(evt->data.input_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_SENSOR_EVENT:
        return sizeof(evt->data.sensor_event);
    case ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_BATTERY_EVENT:
        return sizeof(evt->data.battery_event);
    default:
        return -ENOTSUP;
    }
}

/* Serialise the current key+button state as a KEY_STATE packet.
 * Idempotent: the central XORs with its previous state, so duplicates emit no events. */
static int send_position_state(void) {
    int ret = k_sem_take(&esb_send_evt_sem, K_FOREVER);
    if (ret) {
        LOG_WRN("Shouldn't be called FOREVER");
        return 0;
    }

    size_t payload_size = sizeof(struct esb_key_state_payload);

    if (ring_buf_space_get(&chosen_tx_buf) < ESB_MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to send key state (have %d but only space for %d/%d)",
                ESB_MSG_EXTRA_SIZE + payload_size, ring_buf_space_get(&chosen_tx_buf),
                ring_buf_capacity_get(&chosen_tx_buf));
        k_sem_give(&esb_send_evt_sem);
        return -ENOSPC;
    }

    struct esb_key_state_envelope env = {
        .prefix = {
            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
            .payload_size = payload_size,
        },
        .payload = {
            .source = peripheral_id | ESB_SOURCE_KEY_STATE_FLAG,
            .button_state = button_state,
        },
    };
    memcpy(env.payload.state, position_state, ESB_KEY_STATE_LEN);

    size_t pfx_len = sizeof(env.prefix) + payload_size;

    size_t put = ring_buf_put(&chosen_tx_buf, (uint8_t *)&env, pfx_len);
    if (put != pfx_len) {
        LOG_WRN("Failed to put key state message (%d vs %d)", put, pfx_len);
    }

    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, pfx_len)};
    put = ring_buf_put(&chosen_tx_buf, (uint8_t *)&postfix, sizeof(postfix));
    if (put != sizeof(postfix)) {
        LOG_WRN("Failed to put key state postfix (%d vs %d)", put, sizeof(postfix));
    }

    begin_tx();
    k_sem_give(&esb_send_evt_sem);
    return 0;
}

/* Append one peripheral-event envelope to the TX ring buffer.  Caller holds
 * esb_send_evt_sem and is responsible for the begin_tx(). */
static int put_event_locked(const struct zmk_split_transport_peripheral_event *event) {
    ssize_t data_size = get_payload_data_size(event);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    size_t payload_size =
        data_size + sizeof(peripheral_id) + sizeof(enum zmk_split_transport_peripheral_event_type);

    if (ring_buf_space_get(&chosen_tx_buf) < ESB_MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to send event to the central (have %d but only space for %d/%d)",
                ESB_MSG_EXTRA_SIZE + payload_size, ring_buf_space_get(&chosen_tx_buf),
                ring_buf_capacity_get(&chosen_tx_buf));
        return -ENOSPC;
    }

    struct esb_event_envelope env = {
        .prefix = {
            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
            .payload_size = payload_size,
        },
        .payload = {
            .source = peripheral_id,
            .event = *event,
        },
    };

    size_t pfx_len = sizeof(env.prefix) + payload_size;

    size_t put = ring_buf_put(&chosen_tx_buf, (uint8_t *)&env, pfx_len);
    if (put != pfx_len) {
        LOG_WRN("Failed to put event message (%d vs %d)", put, pfx_len);
    }

    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, pfx_len)};
    put = ring_buf_put(&chosen_tx_buf, (uint8_t *)&postfix, sizeof(postfix));
    if (put != sizeof(postfix)) {
        LOG_WRN("Failed to put event postfix (%d vs %d)", put, sizeof(postfix));
    }
    return 0;
}

/* --- Coalesce-on-pull movement lane (see common.h) -------------------------
 *
 * Relative pointer movement is summed into mv_accum and drained into a single
 * outstanding packet whenever the radio is free.  mv_inflight holds the deltas
 * of the packet currently in flight so they can be restored if it fails. */
static const uint16_t mv_axis_code[ESB_REL_AXES] = {INPUT_REL_X, INPUT_REL_Y, INPUT_REL_WHEEL,
                                                    INPUT_REL_HWHEEL};

static int32_t mv_accum[ESB_REL_AXES];
static int32_t mv_inflight[ESB_REL_AXES];
static bool mv_outstanding;
static uint8_t mv_reg;

static void revert_movement(void) {
    unsigned int key = irq_lock();
    for (int i = 0; i < ESB_REL_AXES; i++) {
        int32_t v = mv_accum[i] + mv_inflight[i];
        mv_accum[i] = CLAMP(v, -ESB_REL_ACCUM_CLAMP, ESB_REL_ACCUM_CLAMP);
        mv_inflight[i] = 0;
    }
    mv_outstanding = false;
    irq_unlock(key);
}

/* Drain the accumulator into one packet, unless one is already outstanding. */
static void maybe_flush_movement(void) {
    int32_t snap[ESB_REL_AXES];
    uint8_t reg;

    unsigned int key = irq_lock();
    if (mv_outstanding) {
        irq_unlock(key);
        return;
    }
    bool any = false;
    int last = -1;
    for (int i = 0; i < ESB_REL_AXES; i++) {
        snap[i] = mv_accum[i];
        if (snap[i]) {
            any = true;
            last = i;
        }
    }
    if (!any) {
        irq_unlock(key);
        return;
    }
    memcpy(mv_inflight, snap, sizeof(snap));
    memset(mv_accum, 0, sizeof(mv_accum));
    reg = mv_reg;
    mv_outstanding = true;
    irq_unlock(key);

    int ret = k_sem_take(&esb_send_evt_sem, K_FOREVER);
    if (ret) {
        LOG_WRN("Shouldn't be called FOREVER");
        revert_movement();
        return;
    }

    /* One REL envelope per non-zero axis; sync set on the last so the central
     * emits a single combined input report. */
    bool ok = true;
    for (int i = 0; i < ESB_REL_AXES; i++) {
        if (!snap[i]) {
            continue;
        }
        struct zmk_split_transport_peripheral_event ev = {
            .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,
            .data = {.input_event = {
                         .reg = reg,
                         .sync = (i == last),
                         .type = INPUT_EV_REL,
                         .code = mv_axis_code[i],
                         .value = snap[i],
                     }}};
        if (put_event_locked(&ev) != 0) {
            ok = false;
            break;
        }
    }
    if (ok) {
        begin_tx();
    }
    k_sem_give(&esb_send_evt_sem);

    if (!ok) {
        revert_movement();
    }
}

static void mv_flush_work_cb(struct k_work *work) { maybe_flush_movement(); }
static K_WORK_DEFINE(mv_flush_work, mv_flush_work_cb);

/* Called (in ESB callback / flush context) when the outstanding movement packet
 * resolves.  On failure its deltas are returned to the accumulator; either way
 * the next pull is kicked from a thread context. */
static void movement_done(bool failed) {
    unsigned int key = irq_lock();
    if (failed) {
        for (int i = 0; i < ESB_REL_AXES; i++) {
            int32_t v = mv_accum[i] + mv_inflight[i];
            mv_accum[i] = CLAMP(v, -ESB_REL_ACCUM_CLAMP, ESB_REL_ACCUM_CLAMP);
        }
    }
    memset(mv_inflight, 0, sizeof(mv_inflight));
    mv_outstanding = false;
    irq_unlock(key);

    k_work_submit(&mv_flush_work);
}

static int
split_peripheral_esb_report_event(const struct zmk_split_transport_peripheral_event *event) {
    if (event->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT) {
        uint32_t pos = event->data.key_position_event.position;
        if (pos >= (uint32_t)(ESB_KEY_STATE_LEN * 8)) {
            LOG_WRN("Key position %u out of bitmap range (%u)", pos, ESB_KEY_STATE_LEN * 8);
            return -EINVAL;
        }
        if (event->data.key_position_event.pressed) {
            position_state[pos / 8] |= BIT(pos % 8);
        } else {
            position_state[pos / 8] &= ~BIT(pos % 8);
        }
        return send_position_state();
    }

    /* Route input button press/release through the idempotent state bitmap. */
    if (event->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT) {
        const struct zmk_split_transport_peripheral_event *ie = event;
        if (ie->data.input_event.type == INPUT_EV_KEY &&
            ie->data.input_event.code >= INPUT_BTN_0 &&
            ie->data.input_event.code < INPUT_BTN_0 + ESB_BTN_STATE_LEN) {
            uint8_t bit = ie->data.input_event.code - INPUT_BTN_0;
            if (ie->data.input_event.value) {
                button_state |= BIT(bit);
            } else {
                button_state &= ~BIT(bit);
            }
            return send_position_state();
        }
    }

    /* Coalesce-on-pull: relative pointer movement is summed into one accumulator
     * and sent as a single packet whenever the radio is free, instead of
     * queueing one packet per movement report. */
    if (event->type == ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT &&
        event->data.input_event.type == INPUT_EV_REL) {
        int idx = zmk_split_esb_rel_axis_index(event->data.input_event.code);
        if (idx >= 0) {
            unsigned int key = irq_lock();
            mv_accum[idx] += event->data.input_event.value;
            mv_reg = event->data.input_event.reg;
            irq_unlock(key);
            maybe_flush_movement();
            return 0;
        }
    }

    /* Generic path for sensor, battery, and any untracked input events. */
    int ret = k_sem_take(&esb_send_evt_sem, K_FOREVER);
    if (ret) {
        LOG_WRN("Shouldn't be called FOREVER");
        return 0;
    }

    ret = put_event_locked(event);
    if (ret == 0) {
        begin_tx();
    }
    k_sem_give(&esb_send_evt_sem);
    return ret;
}

static zmk_split_transport_peripheral_status_changed_cb_t transport_status_cb;
static bool is_enabled = false;

static int split_peripheral_esb_set_enabled(bool enabled) {
    is_enabled = enabled;
    return zmk_split_esb_set_enable(enabled);
}

static int
split_peripheral_esb_set_status_callback(zmk_split_transport_peripheral_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_peripheral_esb_get_status(void) {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = split_peripheral_esb_report_event,
    .set_enabled = split_peripheral_esb_set_enabled,
    .set_status_callback = split_peripheral_esb_set_status_callback,
    .get_status = split_peripheral_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(esb_peripheral, &peripheral_api,
                                        CONFIG_ZMK_SPLIT_ESB_PRIORITY);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_peripheral, split_peripheral_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static int zmk_split_esb_peripheral_init(void) {
    zmk_split_esb_register_movement_done_cb(movement_done);
    int ret = zmk_split_esb_init(APP_ESB_MODE_PTX, zmk_split_esb_on_ptx_esb_callback);
    if (ret < 0) {
        LOG_ERR("zmk_split_esb_init failed (ret %d)", ret);
        return ret;
    }
    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static void process_tx_cb(void) {
    while (ring_buf_size_get(&chosen_rx_buf) > ESB_MSG_EXTRA_SIZE) {
        struct esb_command_envelope env;
        int item_err = zmk_split_esb_get_item(&chosen_rx_buf, (uint8_t *)&env,
                                                sizeof(struct esb_command_envelope));
        switch (item_err) {
        case 0:
            if (env.payload.cmd.type == ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS) {
                begin_tx();
            } else {
                if (env.payload.source != peripheral_id) {
                    LOG_WRN("Ignoring command type %d for source %d (expect %d)", 
                            env.payload.cmd.type, env.payload.source, peripheral_id);
                    return;
                }

                int ret = k_msgq_put(&cmd_msg_queue, &env.payload.cmd, K_NO_WAIT);
                if (ret < 0) {
                    LOG_WRN("Failed to queue command for processing (%d)", ret);
                    return;
                }

                k_work_submit(&publish_commands);
            }
            break;
        case -EAGAIN:
            return;
        default:
            LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
            return;
        }
    }
}

static void publish_commands_work(struct k_work *work) {
    struct zmk_split_transport_central_command cmd;

    while (k_msgq_get(&cmd_msg_queue, &cmd, K_NO_WAIT) >= 0) {
        zmk_split_transport_peripheral_command_handler(&esb_peripheral, cmd);
    }
}
