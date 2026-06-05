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

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/central.h>
#include <zmk/split/transport/types.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/endpoints_types.h>
#include <zmk/events/endpoint_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "app_esb.h"
#include "common.h"

#define RX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_event_envelope) + sizeof(struct esb_msg_postfix)) *                        \
     CONFIG_ZMK_SPLIT_ESB_EVENT_BUFFER_ITEMS)
#define TX_BUFFER_SIZE                                                                             \
    ((sizeof(struct esb_command_envelope) + sizeof(struct esb_msg_postfix)) *                      \
     CONFIG_ZMK_SPLIT_ESB_CMD_BUFFER_ITEMS)

RING_BUF_DECLARE(rx_buf, RX_BUFFER_SIZE);
RING_BUF_DECLARE(tx_buf, TX_BUFFER_SIZE);

static K_SEM_DEFINE(esb_send_cmd_sem, 1, 1);

static void publish_events_work(struct k_work *work);

K_WORK_DEFINE(publish_events, publish_events_work);

uint8_t async_rx_buf[RX_BUFFER_SIZE / 2][2];

static struct zmk_split_esb_async_state async_state = {
    .process_tx_work = &publish_events,
    .rx_bufs = {async_rx_buf[0], async_rx_buf[1]},
    .rx_bufs_len = RX_BUFFER_SIZE / 2,
    .rx_size_process_trigger = ESB_MSG_EXTRA_SIZE + 1,
    .rx_buf = &rx_buf,
    .tx_buf = &tx_buf,
};

static void begin_tx(void) {
    zmk_split_esb_async_tx(&async_state);
}

static ssize_t get_payload_data_size(const struct zmk_split_transport_central_command *cmd) {
    switch (cmd->type) {
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_POLL_EVENTS:
        return 0;
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_INVOKE_BEHAVIOR:
        return sizeof(cmd->data.invoke_behavior);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_PHYSICAL_LAYOUT:
        return sizeof(cmd->data.set_physical_layout);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_SET_HID_INDICATORS:
        return sizeof(cmd->data.set_hid_indicators);
    case ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_TRANSPORT_CHANGED:
        return sizeof(cmd->data.set_transport);
    default:
        return -ENOTSUP;
    }
}

static int split_central_esb_send_command(uint8_t source,
                                          struct zmk_split_transport_central_command cmd) {

    ssize_t data_size = get_payload_data_size(&cmd);
    if (data_size < 0) {
        LOG_WRN("Failed to determine payload data size %d", data_size);
        return data_size;
    }

    // lock it for a safe result from ring_buf_space_get()
    int ret = k_sem_take(&esb_send_cmd_sem, K_FOREVER);
    if (ret) {
        LOG_WRN("Shouldn't be called FOREVER");
        return 0;
    }

    // Data + type + source
    size_t payload_size =
        data_size + sizeof(source) + sizeof(enum zmk_split_transport_central_command_type);

    if (ring_buf_space_get(&tx_buf) < ESB_MSG_EXTRA_SIZE + payload_size) {
        LOG_WRN("No room to send command to the peripheral %d (have %d but only space for %d/%d)",
                source, ESB_MSG_EXTRA_SIZE + payload_size, ring_buf_space_get(&tx_buf),
                ring_buf_capacity_get(&tx_buf));
        k_sem_give(&esb_send_cmd_sem);
        return -ENOSPC;
    }

    struct esb_command_envelope env = {
        .prefix = {
            .magic_prefix = ZMK_SPLIT_ESB_ENVELOPE_MAGIC_PREFIX,
            .payload_size = payload_size,
        },
        .payload = {
            .source = source,
            .cmd = cmd,
        },
    };

    size_t pfx_len = sizeof(env.prefix) + payload_size;
    // LOG_HEXDUMP_DBG(&env, pfx_len, "Payload");

    size_t put = ring_buf_put(&tx_buf, (uint8_t *)&env, pfx_len);
    if (put != pfx_len) {
        LOG_WRN("Failed to put the whole message (%d vs %d)", put, pfx_len);
    }

    struct esb_msg_postfix postfix = {.crc = crc32_ieee((void *)&env, pfx_len)};

    put = ring_buf_put(&tx_buf, (uint8_t *)&postfix, sizeof(postfix));
    if (put != sizeof(postfix)) {
        LOG_WRN("Failed to put the postfix (%d vs %d)", put, sizeof(postfix));
    }

    begin_tx();

    k_sem_give(&esb_send_cmd_sem);
    return 0;
}

void zmk_split_esb_on_prx_esb_callback(app_esb_event_t *event) {
    zmk_split_esb_cb(event, &async_state);
}

static int split_central_esb_get_available_source_ids(uint8_t *sources) {
    sources[0] = 0;

    return 1;
}

static zmk_split_transport_central_status_changed_cb_t transport_status_cb;
static bool is_enabled;

static int split_central_esb_set_enabled(bool enabled) {
    is_enabled = enabled;
    return zmk_split_esb_set_enable(enabled);
}

static int
split_central_esb_set_status_callback(zmk_split_transport_central_status_changed_cb_t cb) {
    transport_status_cb = cb;
    return 0;
}

static struct zmk_split_transport_status split_central_esb_get_status() {
    return (struct zmk_split_transport_status){
        .available = true,
        .enabled = is_enabled,
        .connections = ZMK_SPLIT_TRANSPORT_CONNECTIONS_STATUS_ALL_CONNECTED,
    };
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_esb_send_command,
    .get_available_source_ids = split_central_esb_get_available_source_ids,
    .set_enabled = split_central_esb_set_enabled,
    .set_status_callback = split_central_esb_set_status_callback,
    .get_status = split_central_esb_get_status,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(esb_central, &central_api, CONFIG_ZMK_SPLIT_ESB_PRIORITY);

/* Per-source key-position and button state tracked by the central.
 * Source IDs are uint8_t so the table covers all possible IDs. */
static uint8_t peripheral_position_state[UINT8_MAX + 1][ESB_KEY_STATE_LEN];
static uint8_t peripheral_button_state[UINT8_MAX + 1];

/* XOR received state against tracked state, emit events for changed bits, update. */
static void process_key_state(uint8_t source, const uint8_t *new_state, uint8_t new_buttons) {
    for (int i = 0; i < ESB_KEY_STATE_LEN; i++) {
        uint8_t changed = new_state[i] ^ peripheral_position_state[source][i];
        peripheral_position_state[source][i] = new_state[i];

        for (int j = 0; j < 8; j++) {
            if (!(changed & BIT(j))) {
                continue;
            }
            uint32_t position = (uint32_t)((i * 8) + j);
            bool pressed = (new_state[i] & BIT(j)) != 0;
            struct zmk_split_transport_peripheral_event ev = {
                .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_KEY_POSITION_EVENT,
                .data = {.key_position_event = {.position = position, .pressed = pressed}},
            };
            zmk_split_transport_central_peripheral_event_handler(&esb_central, source, ev);
        }
    }

    uint8_t changed_buttons = new_buttons ^ peripheral_button_state[source];
    peripheral_button_state[source] = new_buttons;

    for (int i = 0; i < ESB_BTN_STATE_LEN; i++) {
        if (!(changed_buttons & BIT(i))) {
            continue;
        }
        bool pressed = (new_buttons & BIT(i)) != 0;
        struct zmk_split_transport_peripheral_event ev = {
            .type = ZMK_SPLIT_TRANSPORT_PERIPHERAL_EVENT_TYPE_INPUT_EVENT,
            .data = {.input_event = {
                .reg = 0,
                .sync = 1,
                .type = INPUT_EV_KEY,
                .code = INPUT_BTN_0 + i,
                .value = pressed ? 1 : 0,
            }},
        };
        zmk_split_transport_central_peripheral_event_handler(&esb_central, source, ev);
    }
}

/* ---- Output-transport broadcast to ESB peripherals ------------------------
 *
 * Peripherals switch their hardware sensor rate depending on whether the central
 * is currently outputting over USB or BLE. The central can only reach peripherals
 * via ESB ACK payloads on a shared pipe — whichever peripheral transmits next
 * consumes the next payload — so reliable *addressed* delivery is impossible.
 *
 * Instead of pushing an addressed, retried message (which polluted the RX hot
 * path and slowed the mouse), we replicate one tiny idempotent value: the current
 * transport. After it changes we open a short "convergence window" during which a
 * timer queues a single TRANSPORT_CHANGED broadcast at a modest cadence. Every
 * peripheral processes the broadcast with no source check and applies it only when
 * the value actually changes, so duplicate deliveries are free and a missed copy
 * is simply picked up from the next one. After the window we go quiet again —
 * steady state adds nothing to the link. None of this runs on the event hot path.
 *
 * The window is also (re)opened whenever a peripheral is first heard or heard
 * again after a gap (a (re)connect), so a peripheral that was silent during the
 * change still converges as soon as it starts transmitting. */
#define ESB_BROADCAST_TICK_MS       30
#define ESB_BROADCAST_WINDOW_MS     600
#define ESB_SOURCE_RECONNECT_GAP_MS 1500

static uint8_t current_transport = ZMK_TRANSPORT_USB;
static atomic_t broadcast_window_until;
static int64_t source_last_seen[UINT8_MAX + 1];

static void esb_broadcast_tick_work(struct k_work *work) {
    if (k_uptime_get() >= atomic_get(&broadcast_window_until)) {
        return;
    }
    /* Keep at most one broadcast pending and let any real command go first. */
    if (ring_buf_is_empty(&tx_buf)) {
        struct zmk_split_transport_central_command cmd = {
            .type = ZMK_SPLIT_TRANSPORT_CENTRAL_CMD_TYPE_TRANSPORT_CHANGED,
            .data = {.set_transport = {.transport = current_transport}},
        };
        int r = split_central_esb_send_command(0, cmd);
        LOG_WRN("ESB-C bcast tx transport=%d ret=%d", current_transport, r);
    } else {
        LOG_WRN("ESB-C bcast skipped (tx_buf not empty)");
    }
}
static K_WORK_DEFINE(esb_broadcast_tick, esb_broadcast_tick_work);

static void esb_broadcast_timer_fn(struct k_timer *timer) {
    if (k_uptime_get() >= atomic_get(&broadcast_window_until)) {
        k_timer_stop(timer);
        return;
    }
    k_work_submit(&esb_broadcast_tick);
}
static K_TIMER_DEFINE(esb_broadcast_timer, esb_broadcast_timer_fn, NULL);

static void esb_open_broadcast_window(void) {
    atomic_set(&broadcast_window_until, k_uptime_get() + ESB_BROADCAST_WINDOW_MS);
    k_timer_start(&esb_broadcast_timer, K_NO_WAIT, K_MSEC(ESB_BROADCAST_TICK_MS));
}

/* Called from the RX path for every peripheral packet. Cheap: a timestamp
 * compare/store, plus a window (re)open on first sight or after a gap. */
static void esb_note_source_seen(uint8_t source) {
    int64_t now = k_uptime_get();
    int64_t prev = source_last_seen[source];
    source_last_seen[source] = now;
    if (prev == 0 || (now - prev) > ESB_SOURCE_RECONNECT_GAP_MS) {
        esb_open_broadcast_window();
    }
}

static int esb_central_on_endpoint_changed(const zmk_event_t *eh) {
    const struct zmk_endpoint_changed *ev = as_zmk_endpoint_changed(eh);
    if (!ev) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    current_transport = ev->endpoint.transport;
    LOG_WRN("ESB-C endpoint_changed transport=%d, opening broadcast window", current_transport);
    esb_open_broadcast_window();
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(esb_central_endpoint, esb_central_on_endpoint_changed);
ZMK_SUBSCRIPTION(esb_central_endpoint, zmk_endpoint_changed);

static void notify_transport_status(void) {
    if (transport_status_cb) {
        transport_status_cb(&esb_central, split_central_esb_get_status());
    }
}

static void notify_status_work_cb(struct k_work *_work) { notify_transport_status(); }

static K_WORK_DEFINE(notify_status_work, notify_status_work_cb);

static int zmk_split_esb_central_init(void) {
    int ret = zmk_split_esb_init(APP_ESB_MODE_PRX, zmk_split_esb_on_prx_esb_callback);
    if (ret) {
        LOG_ERR("zmk_split_esb_init failed (err %d)", ret);
        return ret;
    }
    /* Seed the broadcast value from the endpoint already selected at boot, so a
     * peripheral that connects before the first endpoint_changed still converges. */
    current_transport = zmk_endpoints_selected().transport;
    k_work_submit(&notify_status_work);
    return 0;
}

SYS_INIT(zmk_split_esb_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static void publish_events_work(struct k_work *work) {
    /* Buffer sized for the larger of the two inbound envelope types. */
    union {
        struct esb_event_envelope event_env;
        struct esb_key_state_envelope key_state_env;
    } env_buf;

    while (ring_buf_size_get(&rx_buf) > ESB_MSG_EXTRA_SIZE) {
        int item_err =
            zmk_split_esb_get_item(&rx_buf, (uint8_t *)&env_buf, sizeof(env_buf));
        switch (item_err) {
        case 0: {
            uint8_t raw_source = env_buf.key_state_env.payload.source;
            uint8_t source = raw_source & ~ESB_SOURCE_KEY_STATE_FLAG;
            esb_note_source_seen(source);
            if (raw_source & ESB_SOURCE_KEY_STATE_FLAG) {
                process_key_state(source, env_buf.key_state_env.payload.state,
                                  env_buf.key_state_env.payload.button_state);
            } else {
                zmk_split_transport_central_peripheral_event_handler(
                    &esb_central, env_buf.event_env.payload.source,
                    env_buf.event_env.payload.event);
            }
            break;
        }
        case -EAGAIN:
            return;
        default:
            LOG_WRN("Issue fetching an item from the RX buffer: %d", item_err);
            return;
        }
    }
}
