/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "app_esb.h"
#include "timeslot.h"
#include <string.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <esb.h>

#include <zmk/events/activity_state_changed.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(app_esb, CONFIG_ZMK_SPLIT_ESB_LOG_LEVEL);


#define DT_DRV_COMPAT zmk_esb_split
#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define HAS_BASE_ADDR_0 (DT_INST_NODE_HAS_PROP(0, base_addr_0))
#define HAS_BASE_ADDR_1 (DT_INST_NODE_HAS_PROP(0, base_addr_1))
#define HAS_ADDR_PREFIX (DT_INST_NODE_HAS_PROP(0, addr_prefix))

#define BASE_ADDR_0_LEN (DT_INST_PROP_LEN(0, base_addr_0))
#define BASE_ADDR_1_LEN (DT_INST_PROP_LEN(0, base_addr_1))
#define ADDR_PREFIX_LEN (DT_INST_PROP_LEN(0, addr_prefix))

#if (!HAS_BASE_ADDR_0 || BASE_ADDR_0_LEN != 4)
#error "zmk,esb-split :: base-addr-0 must include 4 bytes"
#endif

#if (!HAS_BASE_ADDR_1 || BASE_ADDR_1_LEN != 4)
#error "zmk,esb-split :: base-addr-1 must include 4 bytes"
#endif

#if (!HAS_ADDR_PREFIX || ADDR_PREFIX_LEN != 8)
#error "zmk,esb-split :: base-addr-0 must include 8 bytes"
#endif

uint8_t esb_base_addr_0[4] = DT_INST_PROP(0, base_addr_0);
uint8_t esb_base_addr_1[4] = DT_INST_PROP(0, base_addr_1);
uint8_t esb_addr_prefix[4] = DT_INST_PROP(0, addr_prefix);

#else
#error "Need to create a node with compatible of 'zmk,esb-split` with `all `address` property set."
#endif

static app_esb_callback_t m_callback;

struct queued_tx_payload {
    struct esb_payload payload;
    uint16_t message_id;
    uint32_t queued_at;
    bool drop_if_stale;
    bool superseded_by_newer;
};

struct retry_entry {
    uint16_t msg_id;
    uint8_t left;
    uint8_t max;
    bool superseded_by_newer;
    struct esb_payload payload;
};

static uint32_t m_msgq_full_last_time;
static uint16_t m_current_tx_msg_id;
static bool m_current_tx_drop_if_stale;
static uint16_t m_latest_superseding_msg_id;
static bool m_pending_stale_valid;
static struct queued_tx_payload m_pending_stale;
static struct retry_entry m_retry_table[CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS];

static void clear_retry_table(void) {
    memset(m_retry_table, 0, sizeof(m_retry_table));
    m_current_tx_msg_id = 0;
    m_current_tx_drop_if_stale = false;
    m_latest_superseding_msg_id = 0;
}

static int find_retry_by_msg_id(uint16_t message_id) {
    for (int i = 0; i < CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS; i++) {
        if (m_retry_table[i].msg_id == message_id) {
            return i;
        }
    }

    return -1;
}

static int find_empty_retry_slot(void) { return find_retry_by_msg_id(0); }

static int add_retry_entry(uint16_t message_id, uint8_t max, bool superseded_by_newer,
                           const struct esb_payload *payload) {
    if (message_id == 0) {
        return -EINVAL;
    }

    int idx = find_retry_by_msg_id(message_id);
    if (idx < 0) {
        idx = find_empty_retry_slot();
    }

    if (idx < 0) {
        return idx;
    }

    struct retry_entry *entry = &m_retry_table[idx];
    entry->msg_id = message_id;
    entry->left = max;
    entry->max = max;
    entry->superseded_by_newer = superseded_by_newer;

    if (max > 0 && payload) {
        entry->payload = *payload;
    } else {
        memset(&entry->payload, 0, sizeof(entry->payload));
    }

    return idx;
}

static void remove_retry_entry_by_msg_id(uint16_t message_id) {
    int idx = find_retry_by_msg_id(message_id);
    if (idx >= 0) {
        memset(&m_retry_table[idx], 0, sizeof(m_retry_table[idx]));
    }
}

static uint8_t get_retry_left_by_msg_id(uint16_t message_id) {
    int idx = find_retry_by_msg_id(message_id);
    return (idx >= 0) ? m_retry_table[idx].left : 0;
}

static uint8_t decrement_retry_by_msg_id(uint16_t message_id) {
    int idx = find_retry_by_msg_id(message_id);
    if (idx >= 0 && m_retry_table[idx].left > 0) {
        m_retry_table[idx].left--;
    }

    return (idx >= 0) ? m_retry_table[idx].left : 0;
}

static bool retry_was_superseded_by_newer(uint16_t message_id) {
    int idx = find_retry_by_msg_id(message_id);
    return idx >= 0 && m_retry_table[idx].superseded_by_newer &&
           message_id != m_latest_superseding_msg_id;
}

// Define a buffer of payloads to store TX payloads in between timeslots
K_MSGQ_DEFINE(m_msgq_tx_payloads, sizeof(struct queued_tx_payload),
              CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS, 4);

static app_esb_mode_t m_mode;
static bool m_active = false;
static bool m_enabled = false;

/* --- Option 5: jitter retransmit delay ±12.5% --- */
static uint32_t m_jitter_rng;

static uint16_t jittered_retransmit_delay(void) {
    if (m_jitter_rng == 0) {
        uint32_t seed = k_uptime_get_32() ^ (k_cycle_get_32() * 2654435761u);
        m_jitter_rng = seed ? seed : 0xA3C59B1Du;
    }
    m_jitter_rng ^= m_jitter_rng << 13;
    m_jitter_rng ^= m_jitter_rng >> 17;
    m_jitter_rng ^= m_jitter_rng << 5;
    const uint32_t base = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY;
    const uint32_t span = base >> 3; /* 12.5% */
    const int8_t off = (int8_t)(m_jitter_rng & 0xFFu);
    const int32_t delta = ((int32_t)span * off) / 128;
    int32_t d = (int32_t)base + delta;
    if (d < 100) d = 100;
    if (d > UINT16_MAX) d = UINT16_MAX;
    return (uint16_t)d;
}

/* --- Option 7: HFXO persistent hold per ESB session --- */
static struct onoff_client m_hfxo_cli;
static bool m_hfxo_held;

static void hfxo_hold(void) {
    if (m_hfxo_held) {
        return;
    }
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!mgr) {
        return;
    }
    sys_notify_init_spinwait(&m_hfxo_cli.notify);
    if (onoff_request(mgr, &m_hfxo_cli) >= 0) {
        int res;
        int err;
        do {
            err = sys_notify_fetch_result(&m_hfxo_cli.notify, &res);
        } while (err);
        m_hfxo_held = (res == 0);
    }
}

static void hfxo_release_hold(void) {
    if (!m_hfxo_held) {
        return;
    }
    struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (mgr) {
        (void)onoff_release(mgr);
    }
    m_hfxo_held = false;
}

static int pull_packet_from_tx_msgq(void);

static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type);

static void event_handler(struct esb_evt const *event) {
    app_esb_event_t m_event;
    switch (event->evt_id) {
        case ESB_EVENT_TX_SUCCESS:
            // LOG_DBG("TX SUCCESS, tx_attempts: %d", event->tx_attempts);
            // LOG_DBG("give d1");
            if (!m_current_tx_drop_if_stale) {
                remove_retry_entry_by_msg_id(m_current_tx_msg_id);
            }
            m_current_tx_msg_id = 0;
            m_current_tx_drop_if_stale = false;
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_SUCCESS;
            m_callback(&m_event);
            pull_packet_from_tx_msgq();
            break;
        case ESB_EVENT_TX_FAILED: {
            LOG_WRN("TX FAILED, tx_attempts: %d", event->tx_attempts);
            bool should_retry = false;

            if (!m_current_tx_drop_if_stale && !retry_was_superseded_by_newer(m_current_tx_msg_id)) {
                should_retry = get_retry_left_by_msg_id(m_current_tx_msg_id) > 0;
                if (should_retry) {
                    decrement_retry_by_msg_id(m_current_tx_msg_id);
                }
            }

            if (should_retry) {
                int start_ret = esb_start_tx();
                if (start_ret == 0) {
                    break;
                }

                LOG_WRN("Failed to restart failed ESB payload for retry (%d)", start_ret);
            }

            esb_flush_tx();
            remove_retry_entry_by_msg_id(m_current_tx_msg_id);
            m_current_tx_msg_id = 0;
            m_current_tx_drop_if_stale = false;
            // Forward an event to the application
            m_event.evt_type = APP_ESB_EVT_TX_FAIL;
            m_callback(&m_event);
            pull_packet_from_tx_msgq();
            break;
        }
        case ESB_EVENT_RX_RECEIVED: {
            // LOG_DBG("RX SUCCESS");
            struct esb_payload rx_payload;
            while (esb_read_rx_payload(&rx_payload) == 0) {
                uint8_t buf[CONFIG_ESB_MAX_PAYLOAD_LENGTH];
                // LOG_DBG("Chunk %d, len: %d", rx_payload.pid, rx_payload.length);
                memcpy(buf, rx_payload.data, rx_payload.length);
                // LOG_DBG("Packet len: %d", rx_payload.length);
                m_event.evt_type = APP_ESB_EVT_RX;
                m_event.buf = buf;
                m_event.data_length = rx_payload.length;
                m_callback(&m_event);
            }
            break;
        }
    }
}

static int clocks_start(void) {
    int err;
    int res;
    struct onoff_manager *clk_mgr;
    struct onoff_client clk_cli;

    clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
    if (!clk_mgr) {
        LOG_ERR("Unable to get the Clock manager");
        return -ENXIO;
    }

    sys_notify_init_spinwait(&clk_cli.notify);

    err = onoff_request(clk_mgr, &clk_cli);
    if (err < 0) {
        LOG_ERR("Clock request failed: %d", err);
        return err;
    }

    do {
        err = sys_notify_fetch_result(&clk_cli.notify, &res);
        if (!err && res) {
            LOG_ERR("Clock could not be started: %d", res);
            return res;
        }
    } while (err);

    LOG_DBG("HF clock started");
    return 0;
}

static int esb_initialize(app_esb_mode_t mode) {
    int err;
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.protocol = ESB_PROTOCOL_ESB_DPL;
    config.retransmit_delay = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_DELAY;
    config.retransmit_count = CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT;
    config.bitrate = ESB_BITRATE_2MBPS_BLE;
    config.tx_output_power = ESB_TX_POWER_4DBM;
    config.use_fast_ramp_up = true;
    config.event_handler = event_handler;
    config.mode = (mode == APP_ESB_MODE_PTX) ? ESB_MODE_PTX : ESB_MODE_PRX;
    config.tx_mode = ESB_TXMODE_MANUAL_START;
    config.selective_auto_ack = true;

    err = esb_init(&config);

    if (err) {
        return err;
    }

    /* Channel 80 = 2480 MHz — above WiFi 1/6/11/13, clear of the crowded 2.4 GHz band */
    err = esb_set_rf_channel(80);
    if (err) {
        return err;
    }

    err = esb_set_base_address_0(esb_base_addr_0);
    if (err) {
        return err;
    }

    err = esb_set_base_address_1(esb_base_addr_1);
    if (err) {
        return err;
    }

    err = esb_set_prefixes(esb_addr_prefix, ARRAY_SIZE(esb_addr_prefix));
    if (err) {
        return err;
    }

    NVIC_SetPriority(RADIO_IRQn, 0);

    if (mode == APP_ESB_MODE_PRX) {
        esb_start_rx();
    }

    return 0;
}

#define ESB_TX_FIFO_REQUE_MAX (CONFIG_ZMK_SPLIT_ESB_PROTO_MSGQ_ITEMS \
                               * CONFIG_ZMK_SPLIT_ESB_PROTO_TX_RETRANSMIT_COUNT)

static bool stale_payload_expired(const struct queued_tx_payload *payload) {
    return payload->drop_if_stale &&
           (k_uptime_get_32() - payload->queued_at >
            CONFIG_ZMK_SPLIT_ESB_DROP_STALE_INPUT_MAX_AGE_MS);
}

static int get_next_tx_payload(struct queued_tx_payload *payload) {
    if (k_msgq_peek(&m_msgq_tx_payloads, payload) == 0) {
        return 0;
    }

    if (m_pending_stale_valid) {
        *payload = m_pending_stale;
        if (stale_payload_expired(payload)) {
            m_pending_stale_valid = false;
            return -EAGAIN;
        }
        return 0;
    }

    return -ENOMSG;
}

static void finish_current_tx_payload(const struct queued_tx_payload *payload) {
    if (payload->drop_if_stale) {
        m_pending_stale_valid = false;
    } else {
        struct queued_tx_payload discarded;
        k_msgq_get(&m_msgq_tx_payloads, &discarded, K_NO_WAIT);
    }
}

static int pull_packet_from_tx_msgq(void) {
    int ret = 0;
    int esb_ret;
    struct queued_tx_payload tx_payload;
    static uint8_t que_was_fulled = 0;

    if (!esb_is_idle()) {
        return -EBUSY;
    }

    while (get_next_tx_payload(&tx_payload) == 0) {
        esb_set_retransmit_delay(jittered_retransmit_delay());
        ret = esb_write_payload(&tx_payload.payload);

        if (ret == -ENOMEM) {
            LOG_WRN("esb_tx_fifo: queue full %d", que_was_fulled);

            // *** deprecated pre-emptive queuing logic ***
            // LOG_DBG("esb_tx_fifo: queue full, popping first message and queueing again");
            // ret = esb_pop_tx();
            // if (ret) {
            //     LOG_ERR("esb_tx_fifo: popping first message and queueing failed (%d)", ret);
            // }
            // ret = esb_write_payload(&tx_payload);
            // if (ret) {
            //     LOG_ERR("esb_write_payload failed (%d)", ret);
            // }

            // force dequeue, guarding for phantom PRX.
            que_was_fulled++;
            if (que_was_fulled >= ESB_TX_FIFO_REQUE_MAX) {
                esb_flush_tx();
                // dequeue FIFO msg
                finish_current_tx_payload(&tx_payload);
                remove_retry_entry_by_msg_id(tx_payload.message_id);
            }
            break;

        } else if (ret == -EMSGSIZE) {
            LOG_WRN("esb_tx_fifo: tx_payload size too large (%d) > CONFIG_ESB_MAX_PAYLOAD_LENGTH (%d)",
                    tx_payload.payload.length, CONFIG_ESB_MAX_PAYLOAD_LENGTH);
            // dequeue FIFO msg
            finish_current_tx_payload(&tx_payload);
            remove_retry_entry_by_msg_id(tx_payload.message_id);

        } else if (ret) {
            LOG_WRN("esb_write_payload failed (%d)", ret);
            if (tx_payload.drop_if_stale || get_retry_left_by_msg_id(tx_payload.message_id) == 0) {
                finish_current_tx_payload(&tx_payload);
                remove_retry_entry_by_msg_id(tx_payload.message_id);
            }
            break;

        } else {
            // LOG_DBG("Payload len: %d", tx_payload.length);
            esb_ret = esb_start_tx();
            if (esb_ret == -EBUSY) {
                LOG_DBG("ESB busy, will retry on next event");
                esb_flush_tx();
                return -EBUSY;
            } else if (esb_ret == -ENODATA) {
                LOG_DBG("ESB TX FIFO empty");
                return 0;
            } else if (esb_ret < 0) {
                LOG_ERR("esb_start_tx failed (%d)", esb_ret);
                esb_flush_tx();
                return esb_ret;
            }
            m_current_tx_msg_id = tx_payload.message_id;
            m_current_tx_drop_if_stale = tx_payload.drop_if_stale;
            // dequeue FIFO msg
            finish_current_tx_payload(&tx_payload);
            que_was_fulled = 0;
            break;
        }
    }

    return ret;
}

int zmk_split_esb_init(app_esb_mode_t mode, app_esb_callback_t callback) {
    int ret;
    m_callback = callback;
    m_mode = mode;
    ret = clocks_start();
    if (ret < 0) {
        return ret;
    }
    LOG_INF("Timeslothandler init");
    zmk_split_esb_timeslot_init(on_timeslot_start_stop);
    return 0;
}

int zmk_split_esb_set_enable(bool enabled) {
    m_enabled = enabled;
    if (enabled) {
        zmk_split_esb_timeslot_open_session();
        return 0;
    } else {
        zmk_split_esb_timeslot_close_session();
        return 0;
    }
}

int zmk_split_esb_send(app_esb_data_t *tx_packet) {
    int ret = 0;
    struct queued_tx_payload tx_payload = {
        .message_id = tx_packet->message_id,
        .queued_at = k_uptime_get_32(),
        .drop_if_stale = tx_packet->drop_if_stale,
        .superseded_by_newer = tx_packet->superseded_by_newer,
        .payload = {
            .pipe = 0,
#if IS_ENABLED(CONFIG_ZMK_SPLIT_ESB_PROTO_TX_ACK)
            .noack = false,
#else
            .noack = true,
#endif
            .length = tx_packet->len,
        },
    };

    memcpy(tx_payload.payload.data, tx_packet->data, tx_packet->len);
    if (!tx_payload.payload.length) {
        LOG_WRN("bypass queuing null payload");
        return 0;
    }

    if (tx_packet->drop_if_stale) {
        m_pending_stale = tx_payload;
        m_pending_stale_valid = true;
        if (m_active) {
            pull_packet_from_tx_msgq();
        }
        return 0;
    }

    if (tx_packet->superseded_by_newer) {
        m_latest_superseding_msg_id = tx_packet->message_id;
    }

    ret = k_msgq_put(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);

    // *** deprecated pre-emptive queuing logic ***
    // if (ret == -EAGAIN || ret == -ENOMSG) {
    //     LOG_WRN("esb tx_payload_q full, popping first message and queueing again");
    //     struct esb_payload dicarded_payload;
    //     k_msgq_get(&m_msgq_tx_payloads, &dicarded_payload, K_NO_WAIT);
    //     ret = k_msgq_put(&m_msgq_tx_payloads, &tx_payload, K_NO_WAIT);
    // }

    if (ret == 0) {
        int idx = add_retry_entry(tx_packet->message_id, tx_packet->max_retry,
                                  tx_packet->superseded_by_newer, &tx_payload.payload);
        if (idx < 0) {
            LOG_WRN("Failed to add retry entry for msg %d (%d)", tx_packet->message_id, idx);
        }
        m_msgq_full_last_time = 0;
    } else if (ret == -ENOMSG) {
        uint32_t now = k_uptime_get_32();
        if (!m_msgq_full_last_time) {
            m_msgq_full_last_time = now;
        }
        if (now - m_msgq_full_last_time > CONFIG_ZMK_SPLIT_ESB_MSGQ_FULL_TIMEOUT_MS) {
            LOG_WRN("Msgq full for %dms, clearing msgq and retry table",
                    CONFIG_ZMK_SPLIT_ESB_MSGQ_FULL_TIMEOUT_MS);
            k_msgq_purge(&m_msgq_tx_payloads);
            clear_retry_table();
            m_msgq_full_last_time = 0;
        }
    } else {
        LOG_WRN("Failed to queue esb tx_payload_q (%d)", ret);
    }
    if (m_active) {
        pull_packet_from_tx_msgq();
    }
    return ret;
}

static int app_esb_suspend(void) {
    m_active = false;
    hfxo_release_hold();
    if(m_mode == APP_ESB_MODE_PTX) {
        uint32_t irq_key = irq_lock();

        irq_disable(RADIO_IRQn);
        NVIC_DisableIRQ(RADIO_IRQn);

        NRF_RADIO->SHORTS = 0;

        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while(NRF_RADIO->EVENTS_DISABLED == 0);

        NRF_TIMER2->TASKS_STOP = 1;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;
        
        esb_disable();

        NVIC_ClearPendingIRQ(RADIO_IRQn);

        irq_unlock(irq_key);
    }
    else {
        esb_stop_rx();
    }

    // Todo: Figure out how to use the esb_suspend() function 
    // rather than having to disable at the end of every timeslot
    //esb_suspend();
    return 0;
}

static int app_esb_resume(void) {
    hfxo_hold();
    if(m_mode == APP_ESB_MODE_PTX) {
        int err = esb_initialize(m_mode);
        m_active = true;
        pull_packet_from_tx_msgq();
        return err;
    }
    else {
        int err = esb_initialize(m_mode);
        m_active = true;
        pull_packet_from_tx_msgq();
        return err;
    }
}

/* Callback function signalling that a timeslot is started or stopped */
static void on_timeslot_start_stop(zmk_split_esb_timeslot_callback_type_t type) {
    switch (type) {
        case APP_TS_STARTED:
            app_esb_resume();
            break;
        case APP_TS_STOPPED:
            app_esb_suspend();
            break;
    }
}

static int on_activity_state(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        return 0;
    }

    if (m_mode == APP_ESB_MODE_PTX) {
        if (state_ev->state != ZMK_ACTIVITY_ACTIVE && m_enabled) {
            zmk_split_esb_set_enable(false);
        }
        else if (state_ev->state == ZMK_ACTIVITY_ACTIVE && !m_enabled) {
            zmk_split_esb_set_enable(true);
        }
    }

    return 0;
}

ZMK_LISTENER(zmk_split_esb_idle_sleeper, on_activity_state);
ZMK_SUBSCRIPTION(zmk_split_esb_idle_sleeper, zmk_activity_state_changed);
